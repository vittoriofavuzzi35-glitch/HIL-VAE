#include <math.h>
#include <string.h>
#include "mappa.h"

/* M_PI non e' in C99 e sui toolchain embedded spesso non c'e'. */
#define PI_F 3.14159265358979323846f

#define C_ MAPPA_CELLE
#define NC (MAPPA_CELLE * MAPPA_CELLE)

/* ------------------------------------------------------------------ *
 * Tabelle fisse. La griglia e' ancorata al drone: le coordinate del
 * centro di ogni cella nel body frame non cambiano mai, e' il MONDO che
 * scorre sotto. Si calcolano una volta sola.
 * ------------------------------------------------------------------ */
static float raggio_cella[NC];
static unsigned char settore_cella[NC];
static unsigned char cella_in_raggio[NC];
static float cos_b[MAPPA_N_RAGGI], sin_b[MAPPA_N_RAGGI];
static int tabelle_pronte = 0;

static void prepara_tabelle(void)
{
    const float meta = (C_ - 1) * 0.5f;
    for (int i = 0; i < C_; ++i) {
        float y = (i - meta) * MAPPA_RIS;
        for (int j = 0; j < C_; ++j) {
            float x = (j - meta) * MAPPA_RIS;
            int k = i * C_ + j;
            float r = sqrtf(x * x + y * y);
            raggio_cella[k] = r;
            /* atan2 da' 0 in avanti e cresce verso sinistra; si porta in
             * [0, 2pi) e si divide in settori, come fa torch.remainder. */
            float a = atan2f(y, x);
            if (a < 0.0f) a += 2.0f * PI_F;
            int s = (int)(a / (2.0f * PI_F) * MAPPA_SETTORI_POL);
            if (s < 0) s = 0;
            if (s > MAPPA_SETTORI_POL - 1) s = MAPPA_SETTORI_POL - 1;
            settore_cella[k] = (unsigned char)s;
            /* Gli angoli della griglia stanno a estensione/2*sqrt(2), fuori
             * dal cerchio inscritto: includerli darebbe una copertura
             * angolare non uniforme. */
            cella_in_raggio[k] = (r <= MAPPA_RAGGIO_POL);
        }
    }
    /* Azimut dei settori orizzontali. Un solo sensore, montato dritto in
     * avanti: yaw di montaggio nullo, restano i centri dei settori dentro
     * il FoV. frac va da -0.375 a +0.375, cioe' da -20.6 a +20.6 gradi. */
    const float fov = MAPPA_FOV_H_DEG * PI_F / 180.0f;
    for (int j = 0; j < MAPPA_N_RAGGI; ++j) {
        float frac = (j + 0.5f) / MAPPA_N_RAGGI - 0.5f;
        float b = frac * fov;
        cos_b[j] = cosf(b);
        sin_b[j] = sinf(b);
    }
    tabelle_pronte = 1;
}

void mappa_init_tabelle(void)
{
    if (!tabelle_pronte) prepara_tabelle();
}

void mappa_reset(mappa_t *m, float prior, float decadimento,
                 float x, float y, float yaw)
{
    if (!tabelle_pronte) prepara_tabelle();
    m->prior = prior;
    m->decadimento = decadimento;
    for (int k = 0; k < NC; ++k) m->griglia[k] = prior;
    m->prec_x = x; m->prec_y = y; m->prec_yaw = yaw;
    m->avviata = 1;
}

/* ------------------------------------------------------------------ *
 * Passo 1 — rototraslazione secondo il moto proprio.
 *
 * Sia B0 il body frame all'update precedente e B1 quello attuale. Per ogni
 * cella della griglia NUOVA si va a leggere la griglia VECCHIA nel punto
 * corrispondente, con interpolazione bilineare. E' quello che fanno
 * affine_grid + grid_sample in Python, riscritto a mano.
 *
 * Le celle che entrano da fuori finestra prendono il PRIOR, non zero:
 * di quella zona la mappa non sa nulla. In Python il trucco e' campionare
 * "lo scarto dal prior" e risommarlo, perche' grid_sample sa riempire solo
 * con zeri; qui si puo' scrivere direttamente il prior, ed e' lo stesso.
 * ------------------------------------------------------------------ */
static void ruota_trasla(mappa_t *m, float x, float y, float yaw)
{
    float dxw = x - m->prec_x;
    float dyw = y - m->prec_y;
    float dyaw = yaw - m->prec_yaw;

    /* Spostamento espresso nel frame PRECEDENTE. */
    float c0 = cosf(m->prec_yaw), s0 = sinf(m->prec_yaw);
    float dx =  c0 * dxw + s0 * dyw;
    float dy = -s0 * dxw + c0 * dyw;

    float meta = MAPPA_ESTENSIONE * 0.5f;
    float cd = cosf(dyaw), sd = sinf(dyaw);
    float tx = dx / meta, ty = dy / meta;

    float *vecchia = m->lavoro;
    memcpy(vecchia, m->griglia, NC * sizeof(float));

    for (int i = 0; i < C_; ++i) {
        /* align_corners=False: il centro del pixel i sta a (2i+1)/H - 1 */
        float yn = (2.0f * i + 1.0f) / C_ - 1.0f;
        for (int j = 0; j < C_; ++j) {
            float xn = (2.0f * j + 1.0f) / C_ - 1.0f;

            float xs = cd * xn - sd * yn + tx;
            float ys = sd * xn + cd * yn + ty;

            /* Da coordinate normalizzate a indici di pixel, sempre con la
             * convenzione align_corners=False. */
            float xp = ((xs + 1.0f) * C_ - 1.0f) * 0.5f;
            float yp = ((ys + 1.0f) * C_ - 1.0f) * 0.5f;

            int x0 = (int)floorf(xp), y0 = (int)floorf(yp);
            float fx = xp - x0, fy = yp - y0;

            float acc = 0.0f;
            for (int dyy = 0; dyy < 2; ++dyy) {
                for (int dxx = 0; dxx < 2; ++dxx) {
                    int xi = x0 + dxx, yi = y0 + dyy;
                    float w = (dxx ? fx : 1.0f - fx) * (dyy ? fy : 1.0f - fy);
                    /* fuori finestra = scarto nullo dal prior */
                    float v = (xi >= 0 && xi < C_ && yi >= 0 && yi < C_)
                              ? vecchia[yi * C_ + xi] - m->prior
                              : 0.0f;
                    acc += w * v;
                }
            }
            m->griglia[i * C_ + j] = acc + m->prior;
        }
    }
    m->prec_x = x; m->prec_y = y; m->prec_yaw = yaw;
}

/* ------------------------------------------------------------------ *
 * Passo 3 — scrittura delle letture ToF.
 *
 * Per ogni direzione: le celle attraversate prima dell'impatto diventano
 * piu' libere, la cella colpita piu' occupata. Una lettura pari al max
 * range non marca NULLA come occupato: significa "non ho visto niente fin
 * li'", non "c'e' un muro a 8.8 m".
 *
 * L'aggiornamento e' additivo, non per sovrascrittura, cosi' una cella
 * colpita da piu' raggi accumula evidenza (scatter_add_ in Python).
 * ------------------------------------------------------------------ */
static void inserisci_letture(mappa_t *m, const float *distanze)
{
    const float meta = (C_ - 1) * 0.5f;
    const float mezza_cella = MAPPA_RIS * 0.5f;

    for (int b = 0; b < MAPPA_N_RAGGI; ++b) {
        float d = distanze[b];
        int eco = (d < MAPPA_MAX_RANGE - 1e-3f);
        for (int k = 0; k < MAPPA_N_CAMPIONI; ++k) {
            float r = (k + 0.5f) * MAPPA_RIS;
            float px = cos_b[b] * r;
            float py = sin_b[b] * r;

            /* Il +0.5 e' l'arrotondamento al centro cella; il troncamento
             * a intero replica .long(), che tronca verso zero. Qui il
             * valore e' sempre >= 0, quindi troncare e' floor. */
            int j = (int)(px / MAPPA_RIS + meta + 0.5f);
            int i = (int)(py / MAPPA_RIS + meta + 0.5f);
            if (i < 0 || i >= C_ || j < 0 || j >= C_) continue;

            float delta = 0.0f;
            if (r < d - mezza_cella)
                delta = -MAPPA_MISS;
            else if (eco && r >= d - mezza_cella && r < d + mezza_cella)
                delta = MAPPA_HIT;
            m->griglia[i * C_ + j] += delta;
        }
    }
    for (int k = 0; k < NC; ++k) {
        if (m->griglia[k] >  MAPPA_CLAMP) m->griglia[k] =  MAPPA_CLAMP;
        if (m->griglia[k] < -MAPPA_CLAMP) m->griglia[k] = -MAPPA_CLAMP;
    }
}

/* ------------------------------------------------------------------ *
 * Passo 4 — riassunto polare.
 *
 * Per ogni settore, la distanza della cella OCCUPATA piu' vicina. Un
 * settore senza celle occupate riporta il raggio massimo, cioe' "per quanto
 * ne so, libero fino al bordo della mappa" — non "libero fino a 8.8 m".
 * ------------------------------------------------------------------ */
static void riassunto_polare(const mappa_t *m, float *polare)
{
    for (int s = 0; s < MAPPA_SETTORI_POL; ++s)
        polare[s] = MAPPA_RAGGIO_POL;
    for (int k = 0; k < NC; ++k) {
        if (!cella_in_raggio[k]) continue;
        if (m->griglia[k] <= MAPPA_SOGLIA_OCC) continue;
        int s = settore_cella[k];
        if (raggio_cella[k] < polare[s]) polare[s] = raggio_cella[k];
    }
}

void mappa_passo(mappa_t *m, float x, float y, float yaw,
                 const float *distanze, float *polare)
{
    /* mappa_reset va chiamata prima: e' li' che arrivano prior e
     * decadimento, e indovinarli qui sarebbe il difetto di partenza. */
    if (!m->avviata) return;

    ruota_trasla(m, x, y, yaw);

    /* Decadimento verso il PRIOR, non verso zero: una cella che nessuno
     * conferma non torna "non so", scivola verso "diffida". */
    for (int k = 0; k < NC; ++k)
        m->griglia[k] = (m->griglia[k] - m->prior) * m->decadimento + m->prior;

    inserisci_letture(m, distanze);
    riassunto_polare(m, polare);
}
