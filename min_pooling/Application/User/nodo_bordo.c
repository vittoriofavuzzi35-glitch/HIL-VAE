/* Il nodo di bordo: dal protocollo alle azioni.
 *
 * E' il pezzo che sulla scheda sta fra la comunicazione e il cervello.
 * Riceve una richiesta gia' validata, ricostruisce l'osservazione,
 * normalizza, esegue la rete, risponde.
 *
 * NON si occupa del trasporto. Che i byte arrivino da micro-ROS, da una
 * seriale o da un file non lo riguarda: riceve una struttura e ne
 * restituisce un'altra. E' anche il motivo per cui si puo' provare a
 * terra, come fa scheda_finta.py sul PC.
 *
 * LO STATO STA QUI. Mappa e integrale vivono dentro questo modulo e non
 * viaggiano mai sul cavo: sono la memoria della catena, e tenerne una
 * copia dall'altra parte significherebbe poterla desincronizzare senza
 * accorgersene. Il PC manda solo cio' che il drone non sa da se'.
 *
 * COSA E' CAMBIATO COL RAMO 1.2M. Dal 10/2026 anche il MIN POOLING sta
 * qui dentro: dal cavo arriva la griglia 18x14, e i dodici settori li
 * riduce settori.c. Prima li riduceva il PC, e il ramo VAE — che riceve
 * l'immagine intera — faceva a bordo un pezzo di catena in piu'. Adesso
 * i due rami fanno lo stesso lavoro, e il confronto misura le reti.
 *
 * E IL RITARDO ANCHE. L'osservazione vuole la lettura vecchia di un
 * update (tof_latency_steps = 1), la mappa quella corrente. Quel ritardo
 * e' memoria di bordo come la mappa, quindi lo tiene questo modulo in
 * settori_prec: al primo passo vale max_range, come il simulatore che
 * azzera _tof_hist a ogni reset.
 */
#include <string.h>

#include "protocollo.h"
#include "nodo_bordo.h"
#include "osservazione.h"
#include "settori.h"
#include "config_bordo.h"
#include "preprocessore.h"
#include "stai.h"

#if defined(__has_include)
  #if __has_include("stai_rete.h")
    #include "stai_rete.h"
  #else
    #include "rete.h"
  #endif
#else
  #include "rete.h"
#endif

#if defined(__ARM_ARCH)
  #include "main.h"
  #include "mcu_cache.h"
#else
  /* Sul banco di prova del PC non c'e' nessuna cache da mantenere e
   * nessun secondo padrone sulla memoria: le due chiamate diventano
   * niente. Non si tolgono con un #if attorno al codice, perche' allora
   * il PC compilerebbe un file diverso da quello che va sulla scheda, ed
   * e' proprio quello che questi banchi devono evitare. */
  #define mcu_cache_clean_range(a, b)       ((void)0)
  #define mcu_cache_invalidate_range(a, b)  ((void)0)
#endif

/* ===================================================================
 * LA RETE E' INT8, E I DUE INGRESSI SONO SEPARATI
 * ===================================================================
 * Fino a settembre la rete girava sul Cortex-M55, generata con
 * --output-data-type float32: gli ingressi erano float e il grafo si
 * quantizzava da se'. Sul Neural-ART si fa come sul ramo VAE — la
 * quantizzazione la fa il C e alla rete arrivano int8 — per due motivi:
 *
 *   - quelle QuantizeLinear sarebbero epoch SOFTWARE, cioe' lavoro che
 *     torna comunque sul M55 passando pero' per il runtime;
 *   - i due rami devono fare a bordo lo stesso lavoro, altrimenti il
 *     confronto fra loro misura due livelli di ottimizzazione invece di
 *     due reti.
 *
 * E GLI INGRESSI SONO DUE, non uno da 1084. La versione a un ingresso
 * affettava dentro il grafo, e quello Slice sul Neural-ART diventa una
 * epoch IBRIDA che copia UN elemento invece di 60 e 1024 (§8.5 di
 * SULLA_SCHEDA.md): la rete riceve zeri e produce una costante, senza
 * nessun errore. Vedi prove/esporta_policy_2in.py.
 *
 * LE COSTANTI vengono da modelli/rete_di_bordo.onnx, lette il 16/10 con
 * prove/quantizza_2in.py. Si rileggono di li' a ogni riesportazione: una
 * scala sbagliata non da' errore, da' azioni fuori fattore.
 *
 *   scalari  scala 0.039196539670229   offset -1
 *   mappa    scala 0.039196539670229   offset -1   (la stessa: entrambi
 *   azione   scala 0.0586366690695286  offset -5    saturano a +-5)
 *
 * Confermate il 16/10 contro lo stai_rete.h generato, che le espone in
 * STAI_RETE_IN_n_SCALES / _OFFSETS e STAI_RETE_OUT_1_SCALES / _OFFSETS.
 * Non si leggono da quelle macro perche' sono liste fra graffe, non
 * valori: estrarle a compile-time costerebbe piu' codice di quanto ne
 * risparmi, e il controllo e' comunque a occhio sul report.
 */
#define V6_OSS_SCALA      0.039196539670229f
#define V6_OSS_OFFSET     (-1)
#define V6_AZIONE_SCALA   0.0586366690695286f
#define V6_AZIONE_OFFSET  (-5)
#define V6_N_SCALARI      (OSS_BASE + OSS_N_TOF)   /* 60 */

#if (V6_N_SCALARI + OSS_N_MAPPA != OSS_TOTALE)
  #error "scalari e mappa non coprono tutta l'osservazione"
#endif
#if (OSS_N != OSS_TOTALE)
  #error "preprocessore.h e osservazione.h non sono d'accordo sulla lunghezza"
#endif
#if (STAI_RETE_OUT_1_SIZE_BYTES != PROT_N_AZIONI)
  #error "la rete non produce quattro azioni int8: protocollo e rete divergono"
#endif

/* L'ARENA DELLE ATTIVAZIONI, dal report del generatore del 16/10:
 *
 *     Used memory ranges
 *       npuRAM4   0x34270000 - 0x34270C80      attivazioni, 3,12 kB
 *       npuRAM5   0x342E0000 - 0x3431E6A0      pesi, 249,64 kB
 *
 * Azzerarla a ogni passo non serve alla correttezza: serve a far
 * diventare "resta a zero" la prova che una epoch non ha scritto, che e'
 * il metodo con cui la §8.6 ha sciolto in mezz'ora tre giorni di
 * ipotesi. Costa un memset di 3 kB, fuori dal cronometro.
 *
 * E GLI INGRESSI STANNO QUI DENTRO: il report lo dice esplicitamente,
 * "input/output buffers are allocated in the activations buffer". E'
 * esattamente la situazione in cui sul ramo VAE le invalidazioni
 * mancanti davano due risultati alternati A B A B — e il motivo per cui
 * piu' sotto si invalidano anche gli ingressi, non solo l'uscita.
 *
 * I numeri cambiano a ogni rigenerazione: si rileggono nel report. */
#ifndef V6_ARENA_BASE
  #define V6_ARENA_BASE  0x34270000u
  #define V6_ARENA_DIM   0xC80u
#endif

STAI_NETWORK_CONTEXT_DECLARE(bordo_ctx, STAI_RETE_CONTEXT_SIZE)

/* Su questa build STAI_RETE_FLAGS vale
 * (STAI_FLAG_INPUTS|STAI_FLAG_OUTPUTS|STAI_FLAG_ACTIVATIONS): il
 * generatore ha allocato lui ingressi, uscite e arena — il report lo
 * conferma, "input/output buffers are allocated in the activations
 * buffer". Quindi si prende questo ramo e il nodo non alloca niente.
 * L'altro resta per una generazione futura che non preallochi. */
#if (STAI_RETE_FLAGS & STAI_FLAG_ACTIVATIONS)
  #define BORDO_ALLOCA_DA_SE 1
#else
  STAI_ALIGNED(8) static uint8_t bordo_attivazioni[STAI_RETE_ACTIVATIONS_SIZE_BYTES];
#endif

static osservazione_t stato;

/* IL POOLING E IL SUO RITARDO.
 *
 * sett           le finestre adattive, calcolate una volta in init
 * valido         la maschera, ricavata dallo zero della profondita'
 * settori_cur    i dodici minimi di QUESTO passo -> mappa
 * settori_prec   quelli del passo prima         -> osservazione
 *
 * settori_prec non e' un'ottimizzazione: e' lo stesso ritardo che in
 * simulazione fa _tof_hist. Toglierlo darebbe alla policy informazione
 * che sul drone vero, fra esposizione, trasferimento I3C e inferenza,
 * non e' ancora disponibile. */
static settori_t sett;
static unsigned char valido[PROT_N_POOL];
static float settori_cur[OSS_N_TOF];
static float settori_prec[OSS_N_TOF];
/* Copia di cio' che il POOLING DI BORDO ha prodotto, tenuta a parte per il
 * controllo: con NODO_POOL_DAL_PC in.tof_* viene riscritto coi settori del
 * PC, e il confronto diventerebbe un numero che torna sempre. */
static float settori_obs_bordo[OSS_N_TOF];

/* Quanti passi confrontare i nostri settori con quelli del PC. Serve a
 * far cadere subito uno scambio fra righe e colonne, che altrimenti non
 * darebbe nessun errore. Dopo questi passi il controllo si spegne: non
 * costa quasi niente, ma dentro la finestra misurata da DWT non ci va
 * roba che a bordo non servirebbe. */
#define NODO_PASSI_CONTROLLO  20
static int passi_controllo;

static float grezza[OSS_TOTALE];
static float normalizzata[OSS_N];
/* I 1084 quantizzati, CONTIGUI. Servono a due cose: riempire i due buffer
 * della rete, e prendere l'impronta sull'osservazione intera, che resta
 * cosi' confrontabile con quella del PC e con le misure precedenti — i due
 * buffer separati non sono adiacenti in memoria. */
static int8_t oss_q[OSS_TOTALE];
static int8_t *ingresso_scalari;
static int8_t *ingresso_mappa;
static const int8_t *uscita_azioni;
static int pronto = 0;

/* Il nodo non sa come si parla al PC, quindi non stampa: passa la stringa
 * a chi gliel'ha indicata. Finche' nessuno la indica, tace. */
static void (*scrivi_log)(const char *) = 0;

void nodo_bordo_imposta_log(void (*scrivi)(const char *testo))
{
    scrivi_log = scrivi;
}

static void di(const char *testo)
{
    if (scrivi_log) scrivi_log(testo);
}

/* Contatore di cicli, per riportare al PC quanto e' costato il passo.
 * E' l'unico modo che il PC ha di sapere se la scheda sta nei tempi:
 * dall'altra parte si vedrebbe solo la latenza totale, che include il
 * cavo e l'agente. */
#if defined(__ARM_ARCH)
  static uint32_t cicli_ora(void) { return DWT->CYCCNT; }
  static void cicli_avvia(void)
  {
#if defined(DCB)
      DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
#else
      CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#endif
      DWT->CYCCNT = 0;
      DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
#else
  static uint32_t cicli_ora(void) { return 0; }
  static void cicli_avvia(void) { }
#endif

/* Da valore reale a int8, con la scala e lo zero della rete. La
 * saturazione e' ridondante — oss_normalizza limita gia' a +-5 — ma costa
 * due confronti e copre il caso in cui un domani il clip cambiasse solo
 * da una parte. */
static int8_t quantizza(float v, float scala, int offset)
{
    float q = v / scala + (float)offset;
    /* +0.5 e -0.5 per arrotondare al piu' vicino: il troncamento di (int)
     * sposterebbe sistematicamente verso lo zero. */
    int i = (int)(q >= 0.0f ? q + 0.5f : q - 0.5f);
    if (i < -128) i = -128;
    if (i > 127) i = 127;
    return (int8_t)i;
}

int nodo_bordo_init(void)
{
    stai_return_code r;
    stai_ptr buf[2];
    stai_size n;

    if (stai_runtime_init() != STAI_SUCCESS) { di("runtime non parte"); return -1; }
    if (stai_rete_init(bordo_ctx) != STAI_SUCCESS) { di("rete non si apre"); return -2; }

#if !defined(BORDO_ALLOCA_DA_SE)
    stai_ptr att = bordo_attivazioni;
    if (stai_rete_set_activations(bordo_ctx, &att, 1) != STAI_SUCCESS) { di("attivazioni rifiutate"); return -3; }
#endif

    /* Si prendono per DIMENSIONE, non per posizione. L'ordine in cui il
     * compilatore li elenca e' una sua scelta, non una garanzia, e
     * scambiarli non darebbe nessun errore: darebbe una rete che legge i
     * 60 scalari come se fossero l'angolo in alto a sinistra della mappa.
     * Le due taglie sono diverse — 60 e 1024 — quindi il riconoscimento e'
     * univoco. E' la stessa disciplina di nodo_bordo_vae.c. */
    r = stai_rete_get_inputs(bordo_ctx, buf, &n);
    if (r != STAI_SUCCESS || n != 2) { di("la rete non ha due ingressi"); return -4; }
    {
        const stai_size dim[2] = { STAI_RETE_IN_1_SIZE_BYTES,
                                   STAI_RETE_IN_2_SIZE_BYTES };
        ingresso_scalari = 0; ingresso_mappa = 0;
        for (int i = 0; i < 2; ++i) {
            if (dim[i] == (stai_size)V6_N_SCALARI)
                ingresso_scalari = (int8_t *)buf[i];
            else if (dim[i] == (stai_size)OSS_N_MAPPA)
                ingresso_mappa = (int8_t *)buf[i];
        }
        if (!ingresso_scalari || !ingresso_mappa) {
            di("ingressi di taglia inattesa"); return -4;
        }
    }

    r = stai_rete_get_outputs(bordo_ctx, buf, &n);
    if (r != STAI_SUCCESS || n != 1) { di("uscita della rete inattesa"); return -6; }
    uscita_azioni = (const int8_t *)buf[0];

    stato.cfg = config_bordo();
    stato.avviato = 0;

    /* Le finestre si CALCOLANO, non si scrivono a mano: con 18 colonne
     * verso 4 settori e 14 righe verso 3 la divisione non e' intera e le
     * finestre si sovrappongono. Un reshape a blocchi disgiunti darebbe
     * dodici numeri plausibili e diversi da quelli su cui la rete e'
     * stata addestrata. Vedi il cappello di settori.h. */
    if (settori_init(&sett, PROT_POOL_ZONE_V, PROT_POOL_ZONE_H,
                     stato.cfg.tof_sectori_v, stato.cfg.tof_settori_h,
                     stato.cfg.tof_max_range) != 0) {
        di("finestre del pooling rifiutate");
        return -7;
    }
    for (int j = 0; j < OSS_N_TOF; ++j) settori_prec[j] = stato.cfg.tof_max_range;
    passi_controllo = NODO_PASSI_CONTROLLO;

    cicli_avvia();
    pronto = 1;
    di("nodo pronto");
#if (V6_ARENA_DIM == 0u)
    di("arena ignota: niente azzeramento, vedi V6_ARENA_BASE");
#endif
    return 0;
}

int nodo_bordo_passo(const prot_richiesta_pool_t *rich, prot_risposta_t *risp)
{
    memset(risp, 0, sizeof *risp);
    risp->sequenza = rich->sequenza;

    if (!pronto) { risp->stato = -1; prot_firma_risposta(risp); return -1; }

    /* Dai 64 float alla struttura. L'ordine e' quello di cervello.c, ed e'
     * lo stesso che impacchetta protocollo.py: se cambia, cambia in tre
     * posti, e prova_anello.py se ne accorge. */
    const float *p = rich->ingresso;
    osservazione_ingresso_t in;
    int k = 0;
    for (int j = 0; j < 3; ++j) in.pos_env[j] = p[k++];
    in.yaw = p[k++];
    in.yaw_mappa = p[k++];
    for (int j = 0; j < 3; ++j) in.lin_vel_b[j] = p[k++];
    for (int j = 0; j < 3; ++j) in.ang_vel_b[j] = p[k++];
    for (int j = 0; j < 3; ++j) in.grav_b[j] = p[k++];
    for (int w = 0; w < stato.cfg.n_waypoints; ++w)
        for (int j = 0; j < 3; ++j) in.wp_pos[w][j] = p[k++];
    for (int w = 0; w < stato.cfg.n_waypoints; ++w) in.wp_yaw[w] = p[k++];
    for (int j = 0; j < 3; ++j) in.room_min[j] = p[k++];
    for (int j = 0; j < 3; ++j) in.room_max[j] = p[k++];
    /* I ventiquattro settori del PC si SALTANO: adesso li calcola la
     * scheda. Restano nel pacchetto solo come controllo, e si leggono
     * piu' sotto, fuori dalla finestra cronometrata. */
    const float *settori_pc_obs = &p[k];  k += OSS_N_TOF;
    const float *settori_pc_cur = &p[k];  k += OSS_N_TOF;
    for (int j = 0; j < OSS_N_AZIONI; ++j) in.prev_azioni[j] = p[k++];
    in.wp_idx = (int)rich->wp_idx;

    /* L'ARENA AZZERATA, FUORI dal cronometro: non e' lavoro che il drone
     * vero farebbe, e' uno strumento di misura. Un buffer che resta a
     * zero dopo la corsa e' la prova che nessuna epoch l'ha scritto;
     * senza, si legge il residuo del passo precedente e non si distingue
     * "mai scritto" da "scritto uguale". Vedi §8.6. */
#if (V6_ARENA_DIM != 0u)
    memset((void *)(uintptr_t)V6_ARENA_BASE, 0, (size_t)V6_ARENA_DIM);
    mcu_cache_clean_range(V6_ARENA_BASE, V6_ARENA_BASE + V6_ARENA_DIM);
#endif

    uint32_t t0 = cicli_ora();

    /* IL MIN POOLING, a bordo.
     *
     * La validita' si ricava dallo zero: il PC manda 0 dove la zona non
     * ha bersaglio. Non e' un valore possibile per una misura — il
     * simulatore satura a tof_min_range = 0.05 m — quindi la convenzione
     * non ha ambiguita'. Una zona non valida ESCE dal minimo: non vale
     * zero, che vincerebbe sempre, e non vale max_range, che diluirebbe
     * un ostacolo vero. */
    for (int j = 0; j < PROT_N_POOL; ++j)
        valido[j] = (rich->profondita[j] > 0.0f) ? 1u : 0u;
    settori_calcola(&sett, rich->profondita, valido, settori_cur);

    /* Il ritardo di un update. L'osservazione vede il passo prima, la
     * mappa il corrente: e' la divisione che fa _update_tof. Al primo
     * passo di un episodio settori_prec vale max_range, come _tof_hist
     * azzerata dal reset. */
    if (rich->primo)
        for (int j = 0; j < OSS_N_TOF; ++j) settori_prec[j] = stato.cfg.tof_max_range;
    for (int j = 0; j < OSS_N_TOF; ++j) {
        in.tof_obs[j] = settori_prec[j];
        in.tof_cur[j] = settori_cur[j];
        settori_obs_bordo[j] = settori_prec[j];
    }
    for (int j = 0; j < OSS_N_TOF; ++j) settori_prec[j] = settori_cur[j];

#if defined(NODO_POOL_DAL_PC)
    /* Via di fuga per un confronto A/B col comportamento di settembre:
     * si riprendono i settori del PC e il pooling di bordo diventa solo
     * diagnostica. Non e' la configurazione della tesi. */
    for (int j = 0; j < OSS_N_TOF; ++j) {
        in.tof_obs[j] = settori_pc_obs[j];
        in.tof_cur[j] = settori_pc_cur[j];
    }
#endif

    osservazione_passo(&stato, &in, (int)rich->primo, grezza);
    oss_normalizza(grezza, normalizzata);
    for (int j = 0; j < OSS_TOTALE; ++j)
        oss_q[j] = quantizza(normalizzata[j], V6_OSS_SCALA, V6_OSS_OFFSET);

    /* Dai 1084 contigui ai due buffer della rete. La mappa va copiata
     * cosi' com'e': con UN canale solo, NCHW e NHWC sono la stessa
     * memoria, quindi il verso dei canali — che sul ToF del VAE cambio'
     * fra due generazioni e costo' un pomeriggio (§8.4) — qui non puo'
     * mordere, e non serve nessuna guardia sui flag. */
    memcpy(ingresso_scalari, oss_q, (size_t)V6_N_SCALARI);
    memcpy(ingresso_mappa, oss_q + V6_N_SCALARI, (size_t)OSS_N_MAPPA);

    /* MANUTENZIONE DELLA CACHE — senza, la rete legge spazzatura.
     *
     * Il Cortex-M55 ha la cache dati accesa e i buffer d'ingresso stanno
     * in memoria che legge anche l'acceleratore. Quello che il C ha
     * appena scritto e' ancora nella cache del processore: la NPU, che va
     * per conto suo sul bus, troverebbe in RAM quello che c'era prima.
     *
     * Per la build CPU di settembre non serviva — un padrone solo sulla
     * memoria — ed e' per questo che queste righe non c'erano. Sul ramo
     * VAE la loro mancanza dava azioni DIVERSE a ogni corsa con lo stesso
     * ingresso, perche' lo sfratto delle linee dipende da cosa e'
     * successo prima, non dai dati. */
    mcu_cache_clean_range((uint32_t)(uintptr_t)ingresso_scalari,
                          (uint32_t)(uintptr_t)ingresso_scalari + V6_N_SCALARI);
    mcu_cache_clean_range((uint32_t)(uintptr_t)ingresso_mappa,
                          (uint32_t)(uintptr_t)ingresso_mappa + OSS_N_MAPPA);

    if (stai_rete_run(bordo_ctx, STAI_MODE_SYNC) != STAI_SUCCESS) {
        risp->stato = -5;
        prot_firma_risposta(risp);
        return -5;
    }

    /* E il verso opposto, sull'uscita E sugli ingressi.
     *
     * L'uscita e' ovvia: le azioni le ha scritte la NPU in RAM, e il
     * processore rileggerebbe dalla propria cache il risultato del passo
     * prima.
     *
     * Gli INGRESSI meno, ed e' la correzione che sul ramo VAE e' servita
     * DOPO la prima: i buffer d'ingresso stanno dentro l'area delle
     * attivazioni, che la NPU riscrive mentre calcola. Finita la corsa le
     * linee di cache che li coprono sono vecchie, al passo dopo il C ci
     * scrive sopra e il clean rimanda in RAM un miscuglio di byte nuovi e
     * stantii. Il sintomo era due risultati che si alternavano, A B A B:
     * non caso, due stati. */
    mcu_cache_invalidate_range((uint32_t)(uintptr_t)uscita_azioni,
                               (uint32_t)(uintptr_t)uscita_azioni
                               + STAI_RETE_OUT_1_SIZE_BYTES);
    mcu_cache_invalidate_range((uint32_t)(uintptr_t)ingresso_scalari,
                               (uint32_t)(uintptr_t)ingresso_scalari + V6_N_SCALARI);
    mcu_cache_invalidate_range((uint32_t)(uintptr_t)ingresso_mappa,
                               (uint32_t)(uintptr_t)ingresso_mappa + OSS_N_MAPPA);

    risp->cicli = cicli_ora() - t0;
    for (int j = 0; j < PROT_N_AZIONI; ++j)
        risp->azioni[j] = ((float)uscita_azioni[j] - (float)V6_AZIONE_OFFSET)
                          * V6_AZIONE_SCALA;
    risp->stato = 0;
    prot_firma_risposta(risp);

    /* IL CONTROLLO, fuori dalla finestra cronometrata.
     *
     * Il PC manda ancora i suoi dodici settori, calcolati dal simulatore
     * sulla stessa griglia. Se il nostro pooling e' giusto i due insiemi
     * coincidono a meno dell'arrotondamento float. Se righe e colonne
     * fossero scambiate — l'errore facile, visto che tof_binning e'
     * (larghezza, altezza) mentre il tensore e' (altezza, larghezza) —
     * lo scarto e' dell'ordine dei metri e si vede al primo secondo.
     *
     * Si guardano TUTTI E DUE gli insiemi, corrente e ritardato. Il
     * corrente prende in castagna il pooling; il ritardato prende in
     * castagna il ritardo, che e' l'altra meta' di cio' che e' sceso a
     * bordo. Uno sfasamento di un passo lascerebbe il corrente perfetto
     * e il ritardato sbagliato, e senza questo confronto si vedrebbe
     * solo come una policy un po' peggiore. */
    if (passi_controllo > 0) {
        float peggio = 0.0f;
        for (int j = 0; j < OSS_N_TOF; ++j) {
            float d = settori_cur[j] - settori_pc_cur[j];
            if (d < 0.0f) d = -d;
            if (d > peggio) peggio = d;
            d = settori_obs_bordo[j] - settori_pc_obs[j];
            if (d < 0.0f) d = -d;
            if (d > peggio) peggio = d;
        }
        --passi_controllo;
        if (peggio > 1e-3f) {
            char riga[PROT_LOG_TESTO + 1];
            int mm = (int)(peggio * 1000.0f);
            /* Niente printf: su questa scheda il filo e' dei pacchetti.
             * Tre cifre di millimetri bastano a distinguere "arrotondamento"
             * da "assi scambiati". */
            riga[0] = 's'; riga[1] = 'e'; riga[2] = 't'; riga[3] = 't';
            riga[4] = 'o'; riga[5] = 'r'; riga[6] = 'i'; riga[7] = ' ';
            riga[8] = 's'; riga[9] = 'c'; riga[10] = 'a'; riga[11] = 'r';
            riga[12] = 't'; riga[13] = 'o'; riga[14] = ' ';
            int n = 15;
            if (mm > 99999) mm = 99999;
            int div = 10000, visto = 0;
            while (div > 0) {
                int c = (mm / div) % 10;
                if (c != 0 || visto || div == 1) { riga[n++] = (char)('0' + c); visto = 1; }
                div /= 10;
            }
            riga[n++] = ' '; riga[n++] = 'm'; riga[n++] = 'm';
            riga[n] = '\0';
            di(riga);
            passi_controllo = 0;      /* detto una volta, non a ogni passo */
        } else if (passi_controllo == 0) {
            di("settori di bordo d'accordo col PC");
        }
    }
    return 0;
}
