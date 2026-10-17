/* Mappa di occupazione egocentrica in log-odds — versione di bordo.
 *
 * Porting in C di obstacle_map_env.py per uni_arco_v6 (UN sensore frontale).
 * I valori qui sotto NON sono scelte: sono copiati dalla configurazione con
 * cui la policy e' stata addestrata. Cambiarne uno significa dare alla rete
 * un ingresso diverso da quello che ha visto in addestramento.
 *
 * Provenienza di ogni costante:
 *   obstacle_map_env_cfg.py   extent, celle, log-odds, decadimento, polare
 *   uni_arco_v6_env_cfg.py    prior (0.10, sovrascrive lo 0.0 della base)
 *   obstacle_pos_env_cfg.py   geometria del ToF
 *   uni_arco_env_cfg.py       sensor_names = ("front",)
 */
#ifndef MAPPA_H
#define MAPPA_H

#define MAPPA_CELLE        32
#define MAPPA_ESTENSIONE   4.0f
#define MAPPA_RIS          (MAPPA_ESTENSIONE / MAPPA_CELLE)   /* 0.125 m */

/* VALORI DI BORDO. Il drone monta uni_arco_v6 e questi sono i suoi. NON
 * sono pero' cablati nell'algoritmo: mappa_reset li riceve, perche' bracci
 * diversi hanno prior diversi (uni_vae_v3 usa 0.15) e una costante
 * compilata li' dentro produrrebbe una mappa sbagliata senza dare nessun
 * errore — e' successo, ed e' costato una notte di corse buttate. */
#define MAPPA_PRIOR        0.10f    /* uni_arco_v6, non lo 0.0 della base */
#define MAPPA_HIT          0.85f
#define MAPPA_MISS         0.40f
#define MAPPA_CLAMP        4.0f
#define MAPPA_SOGLIA_OCC   0.5f

/* decadimento = exp(-dt / tau), dt = step_dt * tof_update_decimation.
 * step_dt = 0.04 s (25 Hz, decimation 4), tof_update_decimation = 1, tau = 4 s.
 * Il valore e' scritto esplicitamente invece di essere calcolato a ogni avvio:
 * cosi' e' verificabile a occhio contro il log di addestramento. */
#define MAPPA_DT           0.04f
#define MAPPA_TAU          4.0f
#define MAPPA_DECADIMENTO  0.99004983f   /* exp(-0.01) */

#define MAPPA_SETTORI_POL  36
#define MAPPA_RAGGIO_POL   2.0f          /* = estensione/2, NON tof_max_range */

/* ToF: un sensore frontale, 4 settori orizzontali su 55 gradi di FoV.
 * I 3 settori verticali sono gia' compressi col minimo prima di arrivare
 * qui, come fa sectors.amin(dim=2) in Python. */
#define MAPPA_N_RAGGI      4
#define MAPPA_FOV_H_DEG    55.0f
#define MAPPA_MAX_RANGE    8.8f

/* Campioni lungo ogni raggio: ceil(raggio_polare / risoluzione) = 16.
 * Passo pari alla cella — piu' fitto sarebbe ridondante, piu' rado
 * lascerebbe buchi nel corridoio libero. */
#define MAPPA_N_CAMPIONI   16

typedef struct {
    float griglia[MAPPA_CELLE * MAPPA_CELLE];  /* log-odds, indice i*C+j */
    /* Copia di lavoro per la rototraslazione: si legge la griglia vecchia
     * mentre si scrive quella nuova. STA QUI DENTRO, non in una variabile
     * static della funzione, perche' una static sarebbe condivisa fra i
     * thread — a bordo il drone e' uno solo e non si noterebbe, ma in
     * validazione mille droni girano in parallelo e le mappe si
     * mescolerebbero fra loro. Costa 4 KB per drone. */
    float lavoro[MAPPA_CELLE * MAPPA_CELLE];
    float prec_x, prec_y, prec_yaw;            /* posa all'update precedente */
    float prior;                               /* ricevuti da mappa_reset, */
    float decadimento;                         /* non compilati dentro */
    int   avviata;
} mappa_t;

/* Riporta la mappa allo stato di partenza e ancora la posa.
 *
 * Le celle vanno al PRIOR, non a zero: e' quello che fa _reset_idx in
 * Python (obstacle_map_env.py:534). Con prior 0.10 la differenza non e'
 * cosmetica — una mappa a zero direbbe "non so" dove la policy addestrata
 * si aspetta "diffida".
 *
 * La posa va passata perche' altrimenti il primo aggiornamento leggerebbe
 * lo spostamento dallo spawn come moto reale e trascinerebbe la mappa. */
/* Prepara le tabelle fisse (raggi, settori, azimut). La chiama gia'
 * mappa_reset alla prima invocazione, ma se piu' thread partono insieme
 * quella prima invocazione e' simultanea e si pestano i piedi: chi usa piu'
 * mappe in parallelo chiami questa UNA volta prima di aprire il parallelo. */
void mappa_init_tabelle(void);

void mappa_reset(mappa_t *m, float prior, float decadimento,
                 float x, float y, float yaw);

/* Un aggiornamento completo: rototraslazione, decadimento, inserimento
 * delle letture, riassunto polare.
 *
 *   x, y, yaw   posa nel mondo (dal Vicon)
 *   distanze    MAPPA_N_RAGGI letture in metri, gia' ridotte col minimo
 *               sulla verticale, in ordine di settore orizzontale
 *   polare      uscita, MAPPA_SETTORI_POL distanze in metri
 */
void mappa_passo(mappa_t *m, float x, float y, float yaw,
                 const float *distanze, float *polare);

#endif
