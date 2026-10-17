#include <math.h>
#include <string.h>
#include "osservazione.h"

#define PI_F 3.14159265358979323846f

static float wrap_pi(float a) { return atan2f(sinf(a), cosf(a)); }

static int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void osservazione_reset(osservazione_t *o, const osservazione_config_t *cfg,
                        const osservazione_ingresso_t *in)
{
    o->cfg = *cfg;
    memset(o->err_integrale, 0, sizeof(o->err_integrale));
    o->wp_idx_prec = in->wp_idx;
    mappa_reset(&o->mappa, cfg->map_prior, cfg->map_decay,
                in->pos_env[0], in->pos_env[1], in->yaw_mappa);
    o->attesa_ancora = 1;      /* vedi la nota in osservazione.h */
    o->avviato = 1;
}

/* I 4 raggi per la mappa: minimo sulla verticale dei settori.
 * Layout dei 12 settori: indice = v * settori_h + h, come produce
 * _pool_sectors appiattendo (sv, sh). */
static void raggi_da_settori(const osservazione_t *o, const float *settori,
                             float *raggi)
{
    int sv = o->cfg.tof_sectori_v, sh = o->cfg.tof_settori_h;
    for (int h = 0; h < sh; ++h) {
        float m = settori[h];
        for (int v = 1; v < sv; ++v) {
            float s = settori[v * sh + h];
            if (s < m) m = s;
        }
        raggi[h] = m;
    }
}

/* Le 20 cifre di anteprima: dove si va adesso, e come gira la rotta dopo.
 *
 * ATTENUAZIONE: avvicinandosi al bersaglio i blocchi successivi svaniscono,
 * perche' anticipare e fermarsi sono richieste incompatibili e insieme
 * producevano un ciclo limite. Il coseno si attenua verso 1, non verso 0:
 * (sin, cos) e' una rotazione, e (0,0) non significa "nessuna rotazione" ma
 * un vettore degenere che la rete non ha mai visto. */
static int scrivi_anteprima(const osservazione_t *o,
                            const osservazione_ingresso_t *in, float *u)
{
    const int nwp = o->cfg.n_waypoints;
    const int H = o->cfg.wp_preview_horizon;
    int n = 0;

    int i0 = clampi(in->wp_idx, 0, nwp - 1);
    float e0 = wrap_pi(in->wp_yaw[i0] - in->yaw);
    for (int k = 0; k < 3; ++k) u[n++] = in->wp_pos[i0][k] - in->pos_env[k];
    u[n++] = sinf(e0);
    u[n++] = cosf(e0);

    float att = o->cfg.preview_attenuazione_dist;
    float fattore = 1.0f;
    if (att > 0.0f) {
        float dx = in->wp_pos[i0][0] - in->pos_env[0];
        float dy = in->wp_pos[i0][1] - in->pos_env[1];
        float dz = in->wp_pos[i0][2] - in->pos_env[2];
        float d0 = sqrtf(dx * dx + dy * dy + dz * dz);
        fattore = d0 / att;
        if (fattore > 1.0f) fattore = 1.0f;
    }

    for (int k = 1; k < H; ++k) {
        int ic = clampi(in->wp_idx + k, 0, nwp - 1);
        int ip = clampi(in->wp_idx + k - 1, 0, nwp - 1);
        float dyaw = wrap_pi(in->wp_yaw[ic] - in->wp_yaw[ip]);
        float sd = sinf(dyaw), cd = cosf(dyaw);
        for (int j = 0; j < 3; ++j) {
            float dp = in->wp_pos[ic][j] - in->wp_pos[ip][j];
            u[n++] = (att > 0.0f) ? dp * fattore : dp;
        }
        if (att > 0.0f) {
            u[n++] = sd * fattore;
            u[n++] = 1.0f + (cd - 1.0f) * fattore;
        } else {
            u[n++] = sd;
            u[n++] = cd;
        }
    }
    return n;   /* 5 + 5*(H-1) = 20 con H = 4 */
}

/* Integrale leaky dell'errore, con anti-windup a cancello.
 *
 * Alimentato SOLO entro integral_gate_dist dal bersaglio. Lontano, l'errore
 * e' dell'ordine dei metri e l'integrale satura contro il clamp in frazioni
 * di secondo, per poi scaricarsi lentamente proprio durante l'assestamento.
 * Col cancello chiuso NON si azzera: smette solo di essere alimentato e
 * continua a decadere con la sua costante di tempo. */
static void aggiorna_integrale(osservazione_t *o,
                               const osservazione_ingresso_t *in)
{
    int i0 = clampi(in->wp_idx, 0, o->cfg.n_waypoints - 1);
    float err[3], d2 = 0.0f;
    for (int k = 0; k < 3; ++k) {
        err[k] = in->pos_env[k] - in->wp_pos[i0][k];
        d2 += err[k] * err[k];
    }
    float yaw_err = wrap_pi(in->yaw - in->wp_yaw[i0]);

    float gate = 1.0f;
    if (o->cfg.integral_gate_dist > 0.0f)
        gate = (sqrtf(d2) < o->cfg.integral_gate_dist) ? 1.0f : 0.0f;

    const float a = o->cfg.integral_alpha, c = o->cfg.integral_clamp;
    const float dt = o->cfg.step_dt;
    for (int k = 0; k < 3; ++k)
        o->err_integrale[k] =
            clampf(a * o->err_integrale[k] + err[k] * dt * gate, -c, c);
    o->err_integrale[3] =
        clampf(a * o->err_integrale[3] + yaw_err * dt * gate, -c, c);
}

void osservazione_passo(osservazione_t *o, const osservazione_ingresso_t *in,
                        int primo_passo, float *u)
{
    if (primo_passo) {
        /* Stanza nuova: mappa al prior, integrale a zero, posa riancorata.
         * Nessun inserimento, per la ragione spiegata in osservazione.h. */
        osservazione_config_t cfg = o->cfg;
        osservazione_reset(o, &cfg, in);
    } else {
        if (o->attesa_ancora) {
            /* Prima posa affidabile dopo il reset: ci si ancora qui, cosi'
             * il primo spostamento vale zero invece di partire dal punto
             * sbagliato. Nel simulatore lo spostamento vero di questo passo
             * vale 40 ms di volo, trascurabile accanto al metro e mezzo che
             * si eviterebbe di sbagliare. */
            o->mappa.prec_x = in->pos_env[0];
            o->mappa.prec_y = in->pos_env[1];
            o->mappa.prec_yaw = in->yaw_mappa;
            o->attesa_ancora = 0;
        }

        /* --- avanzamento del waypoint: l'integrale si ATTENUA ---
         * Nel simulatore, quando la coda avanza, _update_waypoint
         * moltiplica l'integrale per integral_keep_on_advance: il bersaglio
         * e' un altro, e l'errore accumulato contro il precedente e' in buona
         * parte storia vecchia. Non azzerarlo del tutto perche' un disturbo
         * costante come il vento resta, ed e' l'unico canale da cui la
         * policy puo' accorgersene.
         *
         * Va applicato QUI, prima di emettere: nel simulatore succede in
         * _pre_physics_step del passo precedente, quindi il suo effetto e'
         * gia' dentro l'osservazione che stiamo ricostruendo. */
        if (in->wp_idx != o->wp_idx_prec)
            for (int k = 0; k < 4; ++k)
                o->err_integrale[k] *= o->cfg.integral_keep_on_advance;

        /* --- la mappa PRIMA di emettere ---
         * Nel simulatore _update_map() sta in _get_dones(), cioe' dopo la
         * fisica e prima di _get_observations(): l'osservazione vede la
         * mappa gia' aggiornata con la posa e le letture di QUESTO istante. */
        float raggi[MAPPA_N_RAGGI];
        float polare[MAPPA_SETTORI_POL];
        raggi_da_settori(o, in->tof_cur, raggi);
        mappa_passo(&o->mappa, in->pos_env[0], in->pos_env[1], in->yaw_mappa,
                    raggi, polare);
    }

    int n = 0;
    for (int k = 0; k < 3; ++k) u[n++] = in->pos_env[k];
    u[n++] = sinf(in->yaw);
    u[n++] = cosf(in->yaw);
    for (int k = 0; k < 3; ++k) u[n++] = in->lin_vel_b[k];
    for (int k = 0; k < 3; ++k) u[n++] = in->ang_vel_b[k];
    for (int k = 0; k < 3; ++k) u[n++] = in->grav_b[k];
    for (int k = 0; k < OSS_N_AZIONI; ++k) u[n++] = in->prev_azioni[k];

    n += scrivi_anteprima(o, in, u + n);

    for (int k = 0; k < 3; ++k) u[n++] = in->room_max[k] - in->pos_env[k];
    for (int k = 0; k < 3; ++k) u[n++] = in->pos_env[k] - in->room_min[k];

    for (int k = 0; k < 4; ++k)
        u[n++] = o->err_integrale[k] / o->cfg.integral_obs_scale;
    /* qui n == OSS_BASE == 48 */

    /* I settori che entrano sono i RITARDATI: e' cio' che il drone sa
     * davvero nell'istante in cui decide, non la verita' fisica. */
    for (int k = 0; k < OSS_N_TOF; ++k)
        u[n++] = in->tof_obs[k] / o->cfg.tof_obs_scale;

    for (int k = 0; k < OSS_N_MAPPA; ++k)
        u[n++] = o->mappa.griglia[k] / o->cfg.map_logodds_clamp;

    /* --- l'integrale DOPO aver emesso ---
     * Nel simulatore l'aggiornamento sta in _pre_physics_step, cioe' prima
     * della fisica: il suo effetto compare nell'osservazione del passo
     * SUCCESSIVO, non in questa. Metterlo prima dell'emissione sfaserebbe
     * di un passo, con un errore piccolo e plausibile. */
    /* Non si integra sulla posa del passo di reset, per la stessa ragione. */
    if (!primo_passo)
        aggiorna_integrale(o, in);
    o->wp_idx_prec = in->wp_idx;
}
