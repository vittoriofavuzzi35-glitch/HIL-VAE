#include <math.h>
#include "settori.h"

/* Finestra i-esima del pooling adattivo, come la definisce PyTorch:
 *   inizio = floor(i * n_in / n_out)      fine = ceil((i+1) * n_in / n_out) */
static void finestre(int n_in, int n_out, int (*f)[2])
{
    for (int i = 0; i < n_out; ++i) {
        f[i][0] = (int)floor((double)i * n_in / n_out);
        f[i][1] = (int)ceil((double)(i + 1) * n_in / n_out);
    }
}

int settori_init(settori_t *s, int zone_v, int zone_h,
                 int settori_v, int settori_h, float max_range)
{
    if (zone_v <= 0 || zone_h <= 0 ||
        zone_v > SETTORI_MAX_ZONE_V || zone_h > SETTORI_MAX_ZONE_H ||
        settori_v <= 0 || settori_h <= 0 ||
        settori_v > SETTORI_MAX_USCITA || settori_h > SETTORI_MAX_USCITA ||
        settori_v > zone_v || settori_h > zone_h)
        return -1;
    s->zone_v = zone_v; s->zone_h = zone_h;
    s->settori_v = settori_v; s->settori_h = settori_h;
    s->max_range = max_range;
    finestre(zone_v, settori_v, s->fin_v);
    finestre(zone_h, settori_h, s->fin_h);
    return 0;
}

void settori_calcola(const settori_t *s, const float *depth,
                     const unsigned char *valido, float *uscita)
{
    for (int sv = 0; sv < s->settori_v; ++sv) {
        for (int sh = 0; sh < s->settori_h; ++sh) {
            float m = INFINITY;
            for (int v = s->fin_v[sv][0]; v < s->fin_v[sv][1]; ++v) {
                for (int h = s->fin_h[sh][0]; h < s->fin_h[sh][1]; ++h) {
                    int k = v * s->zone_h + h;
                    /* Zona non valida: esce dal minimo, non lo falsa. */
                    if (!valido[k]) continue;
                    if (depth[k] < m) m = depth[k];
                }
            }
            /* Settore interamente cieco: "non ho rilevato nulla fin dove
             * arrivo", che non e' "via libera per sempre" ma e' l'unica
             * affermazione onesta possibile. */
            uscita[sv * s->settori_h + sh] = isfinite(m) ? m : s->max_range;
        }
    }
}
