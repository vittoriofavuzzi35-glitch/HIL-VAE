/* Dal sensore ai 12 settori — il ponte fra il VL53L9CX e la rete.
 *
 * Ingresso: una griglia H x W di distanze per zona e la loro validita',
 * cioe' esattamente cio' che il sensore produce. Uscita: sv x sh settori in
 * metri, nell'ordine [v * sh + h] che la rete si aspetta.
 *
 * TRE COSE CHE SI SBAGLIANO IN SILENZIO
 *
 * 1. Il pooling e' il MINIMO, non la media. Per schivare conta l'ostacolo
 *    piu' vicino in una direzione; una media diluirebbe un palo sottile fra
 *    zone che vedono lontano, proprio il caso in cui bisogna reagire.
 *
 * 2. Le finestre sono SOVRAPPOSTE. Con 18 colonne verso 4 settori la
 *    divisione non e' intera, e il simulatore usa un pooling adattivo:
 *
 *      colonne 18 -> 4:  [0,5) [4,9) [9,14) [13,18)   condivise: 4 e 13
 *      righe   14 -> 3:  [0,5) [4,10) [9,14)          condivise: 4 e 9
 *
 *    Un reshape a blocchi disgiunti darebbe 12 numeri plausibili e diversi
 *    da quelli su cui la rete e' stata addestrata. Le finestre qui sono
 *    calcolate, non scritte a mano, cosi' valgono per qualunque risoluzione
 *    dia il sensore vero.
 *
 * 3. Una zona non valida non vale zero e non vale max_range: ESCE dal
 *    minimo. Se tutte le zone di un settore sono invalide, allora il
 *    settore vale max_range — in assenza di informazione l'unica cosa
 *    onesta e' "non ho rilevato nulla fin dove arrivo".
 */
#ifndef SETTORI_H
#define SETTORI_H

#define SETTORI_MAX_ZONE_V  64
#define SETTORI_MAX_ZONE_H  64
#define SETTORI_MAX_USCITA  16

typedef struct {
    int zone_v, zone_h;          /* risoluzione del sensore */
    int settori_v, settori_h;    /* 3 x 4 per uni_arco_v6 */
    float max_range;             /* metri */
    int fin_v[SETTORI_MAX_USCITA][2];
    int fin_h[SETTORI_MAX_USCITA][2];
} settori_t;

/* Calcola le finestre adattive. Torna 0 se ok, -1 se le dimensioni non
 * stanno nei limiti o se i settori sono piu' delle zone. */
int settori_init(settori_t *s, int zone_v, int zone_h,
                 int settori_v, int settori_h, float max_range);

/* depth: zone_v * zone_h distanze in METRI, indice v * zone_h + h.
 * valido: stessa forma, non zero dove la misura e' buona.
 * uscita: settori_v * settori_h metri. */
void settori_calcola(const settori_t *s, const float *depth,
                     const unsigned char *valido, float *uscita);

#endif
