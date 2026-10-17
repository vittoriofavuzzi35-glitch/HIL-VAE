/* Costruzione del vettore di osservazione di bordo.
 *
 * DUE LAYOUT, scelti a compilazione. Non e' una comodita': sono i due
 * rami della tesi, e devono poter convivere nello stesso albero senza
 * che toccare l'uno muova l'altro.
 *
 *   senza OSS_SENZA_SETTORI          uni_arco_v6, min pooling
 *     [   0:  48]  propriocettive
 *     [  48:  60]  12 settori ToF RITARDATI / tof_obs_scale
 *     [  60:1084]  mappa 32x32 in log-odds / map_logodds_clamp
 *                                                      totale 1084
 *
 *   con OSS_SENZA_SETTORI            uni_vae_senza_settori
 *     [   0:  48]  propriocettive
 *     [  48:1072]  mappa 32x32 in log-odds
 *                                                      totale 1072
 *     piu' 64 latenti che aggiunge nodo_bordo_vae -> 1136
 *
 * I SETTORI SPARISCONO SOLO DALL'OSSERVAZIONE, non dalla catena: la
 * mappa continua a costruirsi dai settori CORRENTI (tof_cur), su
 * entrambi i rami. Quello che non entra piu' e' la lettura RITARDATA.
 * Di conseguenza sul ramo VAE il ritardo di un passo non serve: esisteva
 * solo per questo vettore.
 *
 * PERCHE' IL LAYOUT SI SCRIVE QUI E NON SI RICOPIA. Il vettore si riempie
 * in sequenza con u[n++], quindi togliere un blocco sposta da se' tutto
 * quello che segue. Il pericolo non e' qui: e' in chi INDICIZZA il
 * vettore dall'esterno — preprocessore*.h e le costanti di
 * nodo_bordo_vae.c — che vanno rigenerati insieme. Un disallineamento fra
 * layout e rete non da' nessun errore: da' numeri plausibili.
 *
 * COSA ARRIVA DA FUORI e cosa si mantiene qui dentro. Da fuori: la posa e le
 * velocita' (Vicon), la coda di waypoint con l'indice corrente (logica di
 * missione, che sul drone sta in un nodo a parte), i limiti della stanza, le
 * letture del ToF. Qui dentro: l'integrale dell'errore, le azioni precedenti
 * e la mappa, perche' sono stato che dipende solo dalla storia di questa
 * catena e duplicarlo altrove significherebbe poterlo desincronizzare.
 *
 * LE COSTANTI NON SONO SCRITTE QUI. Arrivano in osservazione_config_t, letta
 * dall'ambiente a runtime. Ricopiarle a mano e' il modo piu' facile di
 * introdurre una differenza silenziosa: basta che una cfg figlia ne
 * sovrascriva una e il C resta indietro senza che niente lo segnali.
 */
#ifndef OSSERVAZIONE_H
#define OSSERVAZIONE_H

#include "mappa.h"

#define OSS_BASE       48
#define OSS_N_TOF      12
#define OSS_N_MAPPA    (MAPPA_CELLE * MAPPA_CELLE)

/* Quanti settori finiscono NELL'OSSERVAZIONE. Zero sul ramo VAE; la
 * mappa ne usa dodici comunque, su entrambi i rami. */
#if defined(OSS_SENZA_SETTORI)
  #define OSS_N_TOF_OSS  0
#else
  #define OSS_N_TOF_OSS  OSS_N_TOF
#endif
#define OSS_TOTALE     (OSS_BASE + OSS_N_TOF_OSS + OSS_N_MAPPA)  /* 1084 o 1072 */
#define OSS_MAX_WP     8
#define OSS_N_AZIONI   4

typedef struct {
    float step_dt;
    float integral_alpha;            /* exp(-dt / tau) */
    float integral_clamp;
    float integral_gate_dist;        /* anti-windup: <= 0 significa sempre aperto */
    float integral_obs_scale;
    float integral_keep_on_advance;  /* al cambio di waypoint */
    float tof_obs_scale;
    float tof_max_range;
    float map_logodds_clamp;
    float map_prior;                 /* diverso fra i bracci! */
    float map_decay;                 /* exp(-dt / tau) */
    float preview_attenuazione_dist; /* 0 = anteprima piena ovunque */
    int   n_waypoints;
    int   wp_preview_horizon;
    int   tof_sectori_v, tof_settori_h;
} osservazione_config_t;

typedef struct {
    osservazione_config_t cfg;
    mappa_t mappa;
    float err_integrale[4];
    int   wp_idx_prec;
    /* LA POSA DEL PRIMO PASSO DOPO UN RESET NON E' AFFIDABILE.
     * In simulazione _reset_idx teletrasporta il drone, ma i buffer che
     * _get_observations legge non sono ancora aggiornati: l'osservazione
     * riporta una posa che il simulatore stesso non usa per i propri
     * aggiornamenti (misurato: 0.39 m contro 1.96 m dal bersaglio, cioe'
     * un metro e mezzo di scarto). Ancorare la mappa a quella posa
     * trascinerebbe tutta la griglia, e alimentare l'integrale aprirebbe
     * un cancello che nel simulatore resta chiuso.
     *
     * Si rimanda quindi di un passo: al primo passo dopo il reset non si
     * ancora e non si integra, al secondo ci si ancora alla posa vera. Sul
     * drone reale il caso non esiste — nessuno lo teletrasporta — quindi
     * questo non e' un compromesso sul comportamento di bordo. */
    int   attesa_ancora;
    int   avviato;
} osservazione_t;

/* Ingressi di un passo. Tutto cio' che la catena non sa da se'. */
typedef struct {
    float pos_env[3];                /* posizione rispetto all'origine stanza */
    /* DUE yaw, non uno per svista. Il simulatore usa convenzioni diverse:
     * l'osservazione passa da Eulero XYZ, la mappa estrae lo yaw diretto dal
     * quaternione per non trascinarsi le singolarita' di Eulero in
     * beccheggio. Con il drone quasi in piano coincidono quasi, ma "quasi"
     * non e' una specifica: sul drone vero il beccheggio non e' nullo. */
    float yaw;                       /* Eulero XYZ: osservazione, anteprima, integrale */
    float yaw_mappa;                 /* diretto dal quaternione: solo la mappa */
    float lin_vel_b[3], ang_vel_b[3], grav_b[3];
    float wp_pos[OSS_MAX_WP][3];
    float wp_yaw[OSS_MAX_WP];
    int   wp_idx;
    float room_min[3], room_max[3];
    float tof_obs[OSS_N_TOF];        /* lettura RITARDATA, va in osservazione.
                                      * Con OSS_SENZA_SETTORI non si legge:
                                      * su quel ramo il campo c'e' ma e'
                                      * ignorato, e chi lo riempie puo'
                                      * risparmiarselo. */
    float tof_cur[OSS_N_TOF];        /* lettura CORRENTE, va nella mappa */
    float prev_azioni[OSS_N_AZIONI];
} osservazione_ingresso_t;

void osservazione_reset(osservazione_t *o, const osservazione_config_t *cfg,
                        const osservazione_ingresso_t *in);

/* Costruisce OSS_TOTALE numeri e fa avanzare lo stato interno.
 *
 * primo_passo va messo a 1 al primo passo di ogni episodio (dopo un reset,
 * o al decollo sul drone vero). In quel caso la mappa vale il PRIOR puro e
 * l'integrale zero, SENZA nessun inserimento: nel simulatore _get_dones()
 * non gira dopo un reset, e inserire qui le prime letture renderebbe il C
 * avanti di un aggiornamento rispetto a cio' che la rete ha visto in
 * addestramento. */
void osservazione_passo(osservazione_t *o, const osservazione_ingresso_t *in,
                        int primo_passo, float *uscita);

#endif
