/* Il protocollo fra il PC e la scheda, lato scheda.
 *
 * COSA VIAGGIA. Il PC manda tutto cio' che la catena di bordo non sa da
 * se': posa, velocita', coda di waypoint, limiti della stanza, letture del
 * ToF. La scheda risponde con le quattro azioni. Mappa e integrale
 * restano a bordo, perche' sono stato interno: tenerne una copia sul PC
 * significherebbe poterla desincronizzare senza accorgersene.
 *
 * QUANTO VIAGGIA. 64 float piu' due interi in andata, 4 float al ritorno.
 * A 25 Hz sono circa 7 kB/s: qualunque trasporto li regge, e il fattore
 * limitante sara' la latenza, non la banda.
 *
 * PERCHE' C'E' UN CRC. Su una seriale i byte si perdono. Senza un modo di
 * accorgersene, un pacchetto corrotto diventa una posa sbagliata, il drone
 * fa una manovra assurda e si passa il pomeriggio a cercare il difetto
 * nella policy invece che nel cavo.
 *
 * PERCHE' C'E' UN NUMERO DI SEQUENZA. Se una risposta arriva in ritardo e
 * il PC nel frattempo ha mandato il passo dopo, senza sequenza si
 * userebbero azioni vecchie credendole nuove. Con la sequenza si vede.
 */
#ifndef PROTOCOLLO_H
#define PROTOCOLLO_H

#include <stdint.h>

#define PROT_MAGIA_RICH   0x4E36494Du   /* "MI6N": richiesta PC -> scheda */
#define PROT_MAGIA_RISP   0x4E364F4Du   /* "MO6N": risposta scheda -> PC */
#define PROT_MAGIA_LOG    0x4E364C4Du   /* "ML6N": messaggio di testo scheda -> PC */
#define PROT_VERSIONE     1u

#define PROT_N_INGRESSO   64            /* float, come cervello_passo */
#define PROT_N_AZIONI     4

/* Le strutture viaggiano cosi' come sono: campi a 4 byte, nessun
 * riempimento possibile, little-endian da entrambe le parti (Cortex-M55 e
 * x86). Se un domani si parlasse con qualcosa di big-endian andrebbe
 * aggiunta una conversione, e questo commento e' il posto dove cercarla. */
typedef struct {
    uint32_t magia;                     /* PROT_MAGIA_RICH */
    uint32_t versione;
    uint32_t sequenza;                  /* cresce di uno a ogni passo */
    int32_t  wp_idx;
    int32_t  primo;                     /* 1 al primo passo dell'episodio */
    float    ingresso[PROT_N_INGRESSO];
    uint32_t crc;                       /* su tutto cio' che precede */
} prot_richiesta_t;

typedef struct {
    uint32_t magia;                     /* PROT_MAGIA_RISP */
    uint32_t sequenza;                  /* copiata dalla richiesta */
    int32_t  stato;                     /* 0 = tutto bene */
    uint32_t cicli;                     /* quanto ci ha messo la scheda */
    float    azioni[PROT_N_AZIONI];
    uint32_t crc;
} prot_risposta_t;

/* PERCHE' IL TESTO VIAGGIA INCAPSULATO. Sulla scheda c'e' una sola UART
 * raggiungibile dal PC, quella dello ST-LINK, e serve ai pacchetti. Se i
 * printf scrivessero sullo stesso filo il loro testo finirebbe in mezzo ai
 * byte binari: il PC leggerebbe un CRC sbagliato e darebbe la colpa al
 * cavo, mentre il guasto e' che due mittenti parlano insieme. Mettendo
 * anche il testo in un pacchetto i due non si pestano, e come effetto
 * secondario i messaggi della scheda si vedono dentro Isaac Lab.
 *
 * Il testo e' a lunghezza fissa e non terminato da zero: una riga piu'
 * lunga viene troncata. Costa qualche byte in piu' sul cavo ma evita di
 * dover leggere una lunghezza di cui non si sa ancora se fidarsi. */
#define PROT_LOG_TESTO    56

typedef struct {
    uint32_t magia;                     /* PROT_MAGIA_LOG */
    uint32_t sequenza;                  /* l'ultima richiesta vista: colloca il log nel tempo */
    uint32_t lunghezza;                 /* byte utili in testo, al massimo PROT_LOG_TESTO */
    char     testo[PROT_LOG_TESTO];
    uint32_t crc;
} prot_log_t;

/* ===================================================================
 * LA RICHIESTA DEL RAMO VAE
 * ===================================================================
 * Il ramo min-pooling manda 64 float e basta: i 12 settori li ha gia'
 * ridotti il PC, e la scheda ci costruisce sopra mappa e osservazione.
 *
 * Il VAE non puo' fare cosi'. Il suo encoder guarda l'IMMAGINE di
 * profondita' intera, 42x54, non i dodici settori: quel passaggio deve
 * attraversare il cavo.
 *
 * COSA VIAGGIA, E PERCHE' COSI'
 *
 *   - profondita' in METRI, un float per zona, 0 = nessun bersaglio.
 *     Non i due canali che vuole la rete: il secondo (validita') si
 *     ricava dal primo, quindi mandarlo sarebbe raddoppiare il cavo per
 *     un'informazione che c'e' gia'.
 *
 *   - in FLOAT, non nel byte da 36 mm del sensore vero. Il sensore vero
 *     quantizza, il simulatore no: quantizzare qui introdurrebbe una
 *     differenza che in simulazione non esiste, e l'equivalenza bit a bit
 *     col simulatore — che e' cio' che l'HIL deve dimostrare — non
 *     sarebbe piu' verificabile.
 *
 *   - i 64 float di v6 restano, IDENTICI e nello stesso ordine. Mappa,
 *     integrale e i 1084 numeri li produce sempre osservazione.c, che non
 *     cambia di una riga: cambia solo cio' che si aggiunge in coda.
 *
 * QUANTO COSTA. 9352 byte contro 280. A 921600 8N1 sono 102 ms per passo
 * invece di 3.4. Non e' un problema: in HIL il simulatore non gira in
 * tempo reale, e il budget dei 40 ms riguarda il calcolo A BORDO, che il
 * contatore DWT misura per conto suo. Sui numeri di v6 — 31.100 passi per
 * 100 missioni in circa 40 minuti — il cavo aggiunge circa un'ora.
 *
 * LA RISPOSTA NON CAMBIA. Sempre prot_risposta_t: quattro azioni, stessa
 * magia, stesso CRC. Cosi' il lato PC che legge le risposte resta comune
 * ai due rami. */
#define PROT_MAGIA_RICH_VAE  0x4E36564Du   /* "MV6N": richiesta VAE */

#define PROT_TOF_ZONE_V      42
#define PROT_TOF_ZONE_H      54
#define PROT_N_PROFONDITA    (PROT_TOF_ZONE_V * PROT_TOF_ZONE_H)  /* 2268 */

typedef struct {
    uint32_t magia;                          /* PROT_MAGIA_RICH_VAE */
    uint32_t versione;
    uint32_t sequenza;
    int32_t  wp_idx;
    int32_t  primo;
    float    ingresso[PROT_N_INGRESSO];      /* 64, come v6 */
    float    profondita[PROT_N_PROFONDITA];  /* metri, 0 = nessun bersaglio */
    uint32_t crc;
} prot_richiesta_vae_t;                      /* 9352 byte */

/* ===================================================================
 * LA RICHIESTA DEL RAMO MIN POOLING
 * ===================================================================
 * Fino al 10/2026 il ramo v6 mandava 64 float e basta, e i dodici
 * settori glieli riduceva IL PC. Funzionava, ma falsava il confronto con
 * il VAE: di quel ramo la scheda riceve l'immagine intera e si costruisce
 * tutto da se', mentre a v6 arrivava un pezzo di catena gia' fatto.
 * Misurare due rami in cui la divisione del lavoro e' diversa non misura
 * le due reti.
 *
 * Adesso attraversa il cavo la GRIGLIA, e il minimo per settore lo fa la
 * scheda con settori.c. Le due catene diventano simmetriche:
 *
 *   v6     18x14 -> settori.c -> 12 settori -> mappa -> rete
 *   VAE    54x42 -> settori.c -> 12 settori -> mappa -> rete
 *                \-> encoder -> 64 latenti --/
 *
 * COSA VIAGGIA, E PERCHE' COSI'
 *
 *   - profondita' in METRI, un float per zona, 0 = zona non valida.
 *     Stessa convenzione del ramo VAE: la validita' non si manda perche'
 *     si ricava dallo zero, e una distanza nulla non esiste (il
 *     simulatore satura a tof_min_range = 0.05 m).
 *
 *   - UNA griglia sola, quella CORRENTE. Il ritardo di un update che
 *     l'osservazione vuole — tof_latency_steps = 1 — lo tiene la scheda,
 *     come lo terrebbe il drone vero: mandare anche la griglia vecchia
 *     significherebbe far ricordare al PC qualcosa che e' memoria di
 *     bordo, ed e' esattamente l'errore che questo pacchetto corregge.
 *
 *   - i 64 float di v6 restano IDENTICI, ventiquattro settori compresi.
 *     Non servono piu' al calcolo: la scheda li usa come CONTROLLO, e
 *     confronta i propri dodici minimi con quelli del PC nei primi passi.
 *     Se le righe e le colonne fossero scambiate — l'errore facile, visto
 *     che tof_binning e' (larghezza, altezza) e il tensore e' (altezza,
 *     larghezza) — lo scarto salta fuori al primo secondo invece che in
 *     una statistica su cinquanta missioni.
 *
 * QUANTO COSTA. 1288 byte contro 280. A 921600 8N1 sono 14 ms per passo
 * invece di 3.4, contro i 102 del ramo VAE. Il budget dei 40 ms riguarda
 * il calcolo A BORDO, che il contatore DWT misura per conto suo.
 *
 * LA RISPOSTA NON CAMBIA. Sempre prot_risposta_t, cosi' il lato PC che
 * legge le risposte resta comune a tutti e tre i formati. */
#define PROT_MAGIA_RICH_POOL  0x4E36504Du   /* "MP6N": richiesta min pooling */

/* La griglia di v6: tof_binning = (18, 14) e' (LARGHEZZA, ALTEZZA), e il
 * tensore della profondita' e' (altezza, larghezza). Scritto qui perche'
 * invertirli non da' nessun errore: da' dodici numeri plausibili e
 * sbagliati. */
#define PROT_POOL_ZONE_V     14
#define PROT_POOL_ZONE_H     18
#define PROT_N_POOL          (PROT_POOL_ZONE_V * PROT_POOL_ZONE_H)   /* 252 */

typedef struct {
    uint32_t magia;                          /* PROT_MAGIA_RICH_POOL */
    uint32_t versione;
    uint32_t sequenza;
    int32_t  wp_idx;
    int32_t  primo;
    float    ingresso[PROT_N_INGRESSO];      /* 64, come v6; i 24 settori sono diagnostica */
    float    profondita[PROT_N_POOL];        /* metri, 0 = zona non valida */
    uint32_t crc;
} prot_richiesta_pool_t;                     /* 1288 byte */

/* CRC-32, polinomio standard riflesso 0xEDB88320.
 * Implementato a bit invece che con tabella: 64 word sono niente, e una
 * tabella da 1 kB per questo non vale lo spazio in RAM. */
uint32_t prot_crc32(const void *dati, uint32_t n_byte);

/* Verificano magia, versione e CRC. Tornano 0 se il pacchetto e' buono. */
int prot_richiesta_valida(const prot_richiesta_t *r);
int prot_richiesta_vae_valida(const prot_richiesta_vae_t *r);
int prot_richiesta_pool_valida(const prot_richiesta_pool_t *r);
int prot_risposta_valida(const prot_risposta_t *r);
int prot_log_valido(const prot_log_t *r);

/* Riempiono il CRC prima di spedire. */
void prot_firma_richiesta(prot_richiesta_t *r);
void prot_firma_richiesta_vae(prot_richiesta_vae_t *r);
void prot_firma_richiesta_pool(prot_richiesta_pool_t *r);
void prot_firma_risposta(prot_risposta_t *r);

/* Riempie testo, lunghezza e CRC da una stringa C, troncando se serve. */
void prot_firma_log(prot_log_t *r, uint32_t sequenza, const char *testo);

#endif
