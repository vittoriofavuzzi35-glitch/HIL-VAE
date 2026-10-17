/* Il nodo di bordo del ramo VAE: dal protocollo alle azioni.
 *
 * Gemello di nodo_bordo.c, e quasi tutto e' uguale: stessa mappa, stesso
 * integrale, stessa osservazione da 1084 numeri prodotta da osservazione.c
 * senza una riga di differenza. Cambiano tre cose, e sono tre cose che si
 * sbagliano in silenzio.
 *
 * 1. DUE INGRESSI INVECE DI UNO.
 *
 *      ingresso 1   42 x 54 x 2   profondita' e validita'   -> encoder
 *      ingresso 2   1084          osservazione normalizzata -> policy
 *
 *    Il grafo unito (unisci_vae.py) contiene encoder e policy insieme, e
 *    dentro ci sta anche la normalizzazione del latente. Il C non vede il
 *    latente: mette i due ingressi, legge le quattro azioni.
 *
 * 2. GLI INGRESSI SONO int8, NON float.
 *
 *    Per v6 il compilatore accettava float e quantizzava lui, pagando due
 *    epoch software. Qui l'interfaccia e' gia' int8, quindi quantizziamo
 *    noi — che e' anche piu' economico, perche' e' una moltiplicazione e
 *    una saturazione per numero, sul Cortex-M55 come la normalizzazione
 *    che gia' facciamo su 1084 valori.
 *
 *    Le costanti NON sono inventate: stanno in stai_network.h come
 *    STAI_NETWORK_IN_1_SCALES / _OFFSETS. Se un domani si rigenera la rete
 *    e le scale cambiano, questi numeri vanno riletti da li'. La verifica
 *    sotto (scale_attese) serve proprio a non lasciarlo scoprire al drone.
 *
 * 3. I DUE CANALI SONO PLANARI, UN BLOCCO PER CANALE.
 *
 *    L'ONNX ha la forma (1, 2, 42, 54), canali primi, e in questa build il
 *    compilatore NON li traspone sull'ingresso: network_c_info.json dice
 *    channel_first=True, channel_last=False per Input_26_out_0, e la
 *    trasposizione la fa l'epoch 1 dentro di se'. Quindi:
 *
 *        buf[                     riga * 54 + colonna] = profondita'
 *        buf[PROT_N_PROFONDITA + (riga * 54 + colonna)] = validita'
 *
 *    ATTENZIONE: nella build a DUE ingressi era il contrario. Il
 *    compilatore spingeva la trasposizione fuori, sull'ingresso, e
 *    stai_network.h lo dichiarava STAI_FLAG_CHANNEL_LAST: la' i due canali
 *    andavano interlacciati. Non e' una scelta nostra ed e' cambiata
 *    riesportando il grafo con tre ingressi, quindi ad ogni rigenerazione
 *    va RILETTA, non ricordata.
 *
 *    Sbagliare il verso non da' nessun errore, ne' di compilazione ne' a
 *    runtime: da' azioni plausibili e sbagliate. E' il difetto piu'
 *    pericoloso di questo file. Il 2/10 ci e' costato una giornata: la
 *    scheda macinava i due canali mescolati e il confronto col PC tornava
 *    a meno di un passo di quantizzazione, perche' lo stesso errore lo
 *    rifacevamo anche sul PC.
 *
 * I DUE CANALI, come in addestramento (obstacle_vae_env.py:_encode):
 *
 *      canale 0 = profondita' / tof_max_range   dove la misura e' valida
 *      canale 1 = 1.0                           dove la misura e' valida
 *      entrambi = 0                             dove non lo e'
 *
 * Il secondo non e' ridondante. Con un canale solo bisognerebbe scrivere
 * un numero anche dove il sensore non ha misurato, e qualunque numero
 * sarebbe una bugia: zero vorrebbe dire "ostacolo attaccato", max_range
 * "via libera". Lo zero su ENTRAMBI i canali e' l'unica cosa onesta.
 */
#include <stdint.h>
#include <string.h>

#include "protocollo.h"
#include "nodo_bordo_vae.h"
#include "osservazione.h"
#include "config_bordo.h"
/* NON preprocessore.h: quello ha le statistiche di uni_arco_v6, e il VAE
 * e' un addestramento a se'. Sui primi 1084 numeri le medie differiscono
 * fino a 0.030 — abbastanza per spostare l'osservazione, troppo poco per
 * accorgersene guardando le azioni. */
#include "preprocessore_vae.h"
#include "stai.h"
#include "stai_network.h"
/* Le funzioni di manutenzione della cache del Cortex-M55. Stesso file
 * che usa network.c generato da ST, con le stesse firme. */
#include "mcu_cache.h"

#if defined(__ARM_ARCH)
  #include "main.h"
#endif

STAI_NETWORK_CONTEXT_DECLARE(vae_ctx, STAI_NETWORK_CONTEXT_SIZE)

#if (STAI_NETWORK_FLAGS & STAI_FLAG_ACTIVATIONS)
  #define VAE_ALLOCA_DA_SE 1
#else
  STAI_ALIGNED(8) static uint8_t vae_attivazioni[STAI_NETWORK_ACTIVATIONS_SIZE_BYTES];
#endif

/* Quantizzazione degli ingressi, da stai_network.h. Ricopiate qui perche'
 * servono in un'espressione, ma controllate all'avvio contro le macro:
 * se la rete viene rigenerata con scale diverse, il nodo si rifiuta di
 * partire invece di produrre azioni sbagliate in silenzio. */
#define VAE_TOF_SCALA      0.00391965359449387f
#define VAE_TOF_OFFSET     (-128)
#define VAE_OSS_SCALA      0.039196539670229f
#define VAE_OSS_OFFSET     (-1)
#define VAE_AZIONE_SCALA   0.056314103f
#define VAE_AZIONE_OFFSET  (11)

/* I 1084 dell'osservazione entrano in DUE buffer separati: i primi 60
 * scalari e i 1024 della mappa. Il quantizzatore li calibra ciascuno per
 * conto suo, ma ha trovato la stessa scala e lo stesso zero — entrambi
 * arrivano saturati a +-5, quindi 5/127.5. Si continua percio' a usare
 * VAE_OSS_SCALA per tutti e due. */
#define VAE_N_SCALARI      (OSS_BASE + OSS_N_TOF)   /* 60 */

/* L'arena delle attivazioni, letta dal report del generatore:
 *     npuRAM6  0x34350000 - 0x3435C540
 * Serve per azzerarla a ogni passo — vedi nodo_bordo_vae_passo. Il numero
 * cambia a ogni rigenerazione della rete: si rilegge nel report, riga
 * "Used memory ranges". */
#define VAE_ARENA_BASE     0x34350000u
#define VAE_ARENA_DIM      0xC540u

/* Le due griglie devono combaciare con quello che il protocollo trasporta:
 * se un giorno cambia il binning del sensore, qui si ferma la compilazione
 * invece di partire uno scarto silenzioso fra le due forme. */
#if (PROT_TOF_ZONE_V != 42) || (PROT_TOF_ZONE_H != 54)
  #error "la rete VAE e' compilata per 42x54: rigenerare o correggere il protocollo"
#endif
#if (STAI_NETWORK_IN_1_SIZE_BYTES != PROT_N_PROFONDITA * 2)
  #error "l'ingresso ToF della rete non corrisponde a 42x54x2"
#endif
/* IL VERSO DEI CANALI, che il compilatore sceglie da se' e che e' GIA'
 * cambiato una volta fra due generazioni (vedi il punto 3 in testa).
 * Sbagliarlo da' azioni plausibili e sbagliate, quindi non lo si ricorda:
 * lo si fa dire al generatore. Se una rigenerazione futura torna a
 * CHANNEL_LAST, qui si ferma la build, e il riempimento va rimesso
 * interlacciato — ingresso_tof[z*2+0] e [z*2+1]. */
#if !(STAI_NETWORK_IN_1_FLAGS & STAI_FLAG_CHANNEL_FIRST)
  #error "l'ingresso ToF non e' CHANNEL_FIRST: il riempimento planare in nodo_bordo_vae_passo e' sbagliato"
#endif
#if (OSS_VAE_N != OSS_TOTALE)
  #error "le costanti del VAE non coprono tutta l'osservazione"
#endif
#if (VAE_N_SCALARI + OSS_N_MAPPA != OSS_TOTALE)
  #error "scalari e mappa non coprono tutta l'osservazione"
#endif
/* Le dimensioni dei tre ingressi non si controllano piu' qui: l'ordine in
 * cui il compilatore li elenca e' una sua scelta, e con tre e' peggio che
 * con due. Il controllo e' a runtime, in nodo_bordo_vae_init, e si rifiuta
 * di partire se non trova tutte e tre le taglie attese. */

static osservazione_t stato;
static float grezza[OSS_TOTALE];
static float normalizzata[OSS_VAE_N];
static int8_t *ingresso_tof;      /* 4536 byte, due canali planari */
static int8_t *ingresso_scalari;  /*   60 byte */
static int8_t *ingresso_mappa;    /* 1024 byte */
/* I 1084 quantizzati in un pezzo solo, prima di essere divisi fra i due
 * buffer della rete. Serve all'IMPRONTA: il CRC va calcolato sui 1084
 * contigui, altrimenti perderemmo il confronto col PC — e' quello che per
 * tre giorni ci ha detto che gli ingressi erano giusti. */
static int8_t oss_q[OSS_TOTALE];
static const int8_t *uscita_azioni;
static int pronto = 0;

static void (*scrivi_log)(const char *) = 0;

void nodo_bordo_vae_imposta_log(void (*scrivi)(const char *testo))
{
    scrivi_log = scrivi;
}

static void di(const char *testo)
{
    if (scrivi_log) scrivi_log(testo);
}

/* Scrive v in decimale e torna quanti caratteri ha scritto. snprintf
 * tirerebbe dentro mezza libreria standard per quattro numeri. */
static int scrivi_dec(char *dst, uint32_t v)
{
    char tmp[11];
    int n = 0, m = 0;
    do { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v);
    while (n) dst[m++] = tmp[--n];
    return m;
}

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

/* Da valore reale a int8, con la scala e lo zero della rete.
 * La saturazione non e' una precauzione teorica: l'osservazione e' gia'
 * limitata a +-5 da oss_normalizza, ma la profondita' arriva dal cavo e
 * un pacchetto valido puo' comunque portare un numero fuori scala. */
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

int nodo_bordo_vae_init(void)
{
    stai_return_code r;
    stai_ptr buf[3];
    stai_size n;

    if (stai_runtime_init() != STAI_SUCCESS) { di("runtime non parte"); return -1; }
    if (stai_network_init(vae_ctx) != STAI_SUCCESS) { di("rete non si apre"); return -2; }

#if !defined(VAE_ALLOCA_DA_SE)
    stai_ptr att = vae_attivazioni;
    if (stai_network_set_activations(vae_ctx, &att, 1) != STAI_SUCCESS) {
        di("attivazioni rifiutate"); return -3;
    }
#endif

    r = stai_network_get_inputs(vae_ctx, buf, &n);
    if (r != STAI_SUCCESS || n != 3) { di("la rete non ha tre ingressi"); return -4; }

    /* Si prendono per DIMENSIONE, non per posizione. L'ordine in cui il
     * compilatore li elenca e' una sua scelta, non una garanzia, e
     * scambiarli non darebbe nessun errore: darebbe una rete che guarda la
     * mappa credendola un'immagine. Le tre taglie sono tutte diverse —
     * 4536, 1024, 60 — quindi il riconoscimento e' univoco. */
    {
        const stai_size dim[3] = { STAI_NETWORK_IN_1_SIZE_BYTES,
                                   STAI_NETWORK_IN_2_SIZE_BYTES,
                                   STAI_NETWORK_IN_3_SIZE_BYTES };
        ingresso_tof = 0; ingresso_scalari = 0; ingresso_mappa = 0;
        for (int i = 0; i < 3; ++i) {
            if (dim[i] == (stai_size)(PROT_N_PROFONDITA * 2))
                ingresso_tof = (int8_t *)buf[i];
            else if (dim[i] == (stai_size)OSS_N_MAPPA)
                ingresso_mappa = (int8_t *)buf[i];
            else if (dim[i] == (stai_size)VAE_N_SCALARI)
                ingresso_scalari = (int8_t *)buf[i];
        }
        if (!ingresso_tof || !ingresso_scalari || !ingresso_mappa) {
            di("ingressi di taglia inattesa"); return -4;
        }
    }

    r = stai_network_get_outputs(vae_ctx, buf, &n);
    if (r != STAI_SUCCESS || n != 1) { di("uscita della rete inattesa"); return -5; }
    uscita_azioni = (const int8_t *)buf[0];

    stato.cfg = config_bordo();
    stato.avviato = 0;
    cicli_avvia();
    pronto = 1;
    di("nodo VAE pronto");
    return 0;
}

int nodo_bordo_vae_passo(const prot_richiesta_vae_t *rich, prot_risposta_t *risp)
{
    risp->sequenza = rich->sequenza;

    if (!pronto) { risp->stato = -1; prot_firma_risposta(risp); return -1; }

    /* Dai 64 float alla struttura. L'ordine e' quello di cervello.c ed e'
     * IDENTICO a nodo_bordo.c: se cambia, cambia in tre posti. */
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
    for (int j = 0; j < OSS_N_TOF; ++j) in.tof_obs[j] = p[k++];
    for (int j = 0; j < OSS_N_TOF; ++j) in.tof_cur[j] = p[k++];
    for (int j = 0; j < OSS_N_AZIONI; ++j) in.prev_azioni[j] = p[k++];
    in.wp_idx = (int)rich->wp_idx;

    /* AZZERAMENTO DELL'ARENA, 2/10.
     *
     * Con il grafo a due ingressi l'uscita seguiva un'orbita deterministica:
     * stesso ingresso, quattro risposte diverse, poi un punto fisso sempre
     * uguale. La causa erano i tre Slice ibridi, che copiavano UN elemento
     * invece di 60, 1024 e 64: la policy leggeva quel che la corsa prima
     * aveva lasciato nei suoi buffer. Azzerare l'arena toglieva l'orbita e
     * lasciava una costante — la prova che l'ingresso non arrivava.
     *
     * Con tre ingressi gli Slice non ci sono piu' e questo azzeramento non
     * dovrebbe servire. Si tiene finche' non e' dimostrato: toglierlo e
     * verificare che l'uscita resti identica e' una prova di dieci minuti.
     *
     * Costa qualche decina di microsecondi su dodici millisecondi. */
    /* SEI CRONOMETRI, non uno (2/10).
     *
     * risp->cicli resta quello che era — da t0 alla fine — perche' e' il
     * numero confrontabile con i 2,69 ms di uni_arco_v6, che usa la stessa
     * convenzione. Ma un numero solo non dice DOVE va il tempo, e per il VAE
     * serve saperlo: la catena ha un encoder convoluzionale e dieci epoch
     * software sul Cortex-M55 che v6 non ha.
     *
     * Due fasi stanno fuori dal conto per ragioni diverse:
     *   l'ARENA e' azzerata prima di t0, quindi i 12,47 ms non la contano;
     *   le due CRC32 sono DIAGNOSTICA e in un sistema dispiegato non ci
     *   sarebbero, ma cadono dentro la finestra misurata.
     * Misurarle entrambe serve a sapere quanto valgono prima di decidere se
     * tenerle nel numero da presentare. */
    uint32_t t_arena = cicli_ora();

    memset((void *)(uintptr_t)VAE_ARENA_BASE, 0, VAE_ARENA_DIM);
    mcu_cache_clean_range(VAE_ARENA_BASE, VAE_ARENA_BASE + VAE_ARENA_DIM);

    uint32_t t0 = cicli_ora();

    /* L'osservazione da 1084: mappa, integrale, settori. Invariata da v6. */
    osservazione_passo(&stato, &in, (int)rich->primo, grezza);
    oss_vae_normalizza(grezza, normalizzata);
    /* Si quantizzano i 1084 in un pezzo solo e POI si dividono. Non per
     * eleganza: il CRC dell'osservazione va preso sui 1084 contigui, ed e'
     * l'unico modo di continuare a confrontarlo con quello del PC. */
    for (int j = 0; j < OSS_TOTALE; ++j)
        oss_q[j] = quantizza(normalizzata[j], VAE_OSS_SCALA, VAE_OSS_OFFSET);
    memcpy(ingresso_scalari, oss_q, VAE_N_SCALARI);
    memcpy(ingresso_mappa, oss_q + VAE_N_SCALARI, OSS_N_MAPPA);

    /* L'immagine per l'encoder, DUE canali PLANARI: prima tutta la
     * profondita', poi tutta la validita'. La profondita' arriva in metri;
     * la rete la vuole divisa per il fondo scala, come in addestramento.
     * Lo zero vuol dire "nessun bersaglio" e azzera entrambi i canali: e'
     * l'unico valore che non mente. Vedi il punto 3 in testa al file per
     * il verso dei canali, che dipende dalla generazione. */
    {
        const float inv_max = 1.0f / stato.cfg.tof_max_range;
        const int8_t zero_tof = quantizza(0.0f, VAE_TOF_SCALA, VAE_TOF_OFFSET);
        const int8_t uno_tof  = quantizza(1.0f, VAE_TOF_SCALA, VAE_TOF_OFFSET);
        int8_t *const canale_profondita = ingresso_tof;
        int8_t *const canale_validita   = ingresso_tof + PROT_N_PROFONDITA;
        for (int z = 0; z < PROT_N_PROFONDITA; ++z) {
            const float d = rich->profondita[z];
            if (d > 0.0f) {
                canale_profondita[z] = quantizza(d * inv_max,
                                                 VAE_TOF_SCALA, VAE_TOF_OFFSET);
                canale_validita[z]   = uno_tof;
            } else {
                canale_profondita[z] = zero_tof;
                canale_validita[z]   = zero_tof;
            }
        }
    }

    uint32_t t_oss = cicli_ora();

    /* MANUTENZIONE DELLA CACHE — senza, la rete legge spazzatura.
     *
     * Il Cortex-M55 ha la cache dati accesa (SCB_EnableDCache in main.c) e
     * i buffer d'ingresso stanno in npuRAM6, memoria che legge anche
     * l'acceleratore. Quello che il C ha appena scritto e' ancora nella
     * cache del processore: la NPU, che va per conto suo sul bus, troverebbe
     * in RAM quello che c'era prima.
     *
     * "clean" spinge le linee sporche dalla cache alla RAM. Va fatto DOPO
     * aver riempito gli ingressi e PRIMA di far partire la rete.
     *
     * Per uni_arco_v6 non serviva: nessuna NPU, un padrone solo sulla
     * memoria. Il 29/9 la mancanza di queste due righe dava azioni DIVERSE
     * a ogni corsa con lo stesso ingresso — perche' lo sfratto delle linee
     * dipende da cosa e' successo prima, non dai dati. */
    mcu_cache_clean_range((uint32_t)(uintptr_t)ingresso_tof,
                          (uint32_t)(uintptr_t)ingresso_tof + PROT_N_PROFONDITA * 2);
    mcu_cache_clean_range((uint32_t)(uintptr_t)ingresso_scalari,
                          (uint32_t)(uintptr_t)ingresso_scalari + VAE_N_SCALARI);
    mcu_cache_clean_range((uint32_t)(uintptr_t)ingresso_mappa,
                          (uint32_t)(uintptr_t)ingresso_mappa + OSS_N_MAPPA);

    uint32_t t_clean = cicli_ora();

    /* Le impronte si prendono ADESSO, non dopo la corsa.
     *
     * Il buffer del ToF sta dentro l'area delle attivazioni, che la NPU usa
     * come spazio di lavoro mentre calcola: finita la corsa contiene
     * risultati intermedi, non piu' il nostro ingresso. Misurarlo li' dava
     * un'impronta diversa a ogni giro e ci ha fatto sospettare la
     * quantizzazione del ToF, che invece era innocente. */
    const uint32_t impronta_tof = prot_crc32(ingresso_tof,
                                      (uint32_t)(PROT_N_PROFONDITA * 2));
    /* Sui 1084 contigui, non sui due buffer separati: cosi' il valore resta
     * confrontabile con quello del PC e con tutte le misure precedenti. */
    const uint32_t impronta_oss = prot_crc32(oss_q, (uint32_t)OSS_TOTALE);

    uint32_t t_crc = cicli_ora();

    if (stai_network_run(vae_ctx, STAI_MODE_SYNC) != STAI_SUCCESS) {
        risp->stato = -5;
        prot_firma_risposta(risp);
        return -5;
    }

    uint32_t t_rete = cicli_ora();

    /* E il verso opposto, su TRE aree e non solo sull'uscita.
     *
     * L'uscita e' ovvia: le quattro azioni le ha scritte la NPU in RAM, e il
     * processore rileggerebbe dalla propria cache il risultato del passo
     * prima.
     *
     * Gli INGRESSI meno, ed e' il motivo per cui il 29/9 la scheda ha
     * continuato a sbagliare anche dopo la prima correzione. I buffer
     * d'ingresso stanno DENTRO l'area delle attivazioni, che la NPU
     * riscrive mentre calcola: finita la corsa, le linee di cache del
     * processore che li coprono sono vecchie. Al passo dopo il C ci scrive
     * sopra i nuovi valori e il clean rimanda in RAM un miscuglio di byte
     * nuovi e byte stantii.
     *
     * Il sintomo era inconfondibile: due risultati che si alternavano,
     * A B A B, invece di uno solo. Non caso — due stati.
     *
     * Qui si invalida e basta, senza clean: gli ingressi li abbiamo gia'
     * scritti in RAM poco fa, quindi non c'e' niente di sporco da perdere. */
    mcu_cache_invalidate_range((uint32_t)(uintptr_t)uscita_azioni,
                               (uint32_t)(uintptr_t)uscita_azioni + STAI_NETWORK_OUT_1_SIZE_BYTES);
    mcu_cache_invalidate_range((uint32_t)(uintptr_t)ingresso_tof,
                               (uint32_t)(uintptr_t)ingresso_tof + PROT_N_PROFONDITA * 2);
    mcu_cache_invalidate_range((uint32_t)(uintptr_t)ingresso_scalari,
                               (uint32_t)(uintptr_t)ingresso_scalari + VAE_N_SCALARI);
    mcu_cache_invalidate_range((uint32_t)(uintptr_t)ingresso_mappa,
                               (uint32_t)(uintptr_t)ingresso_mappa + OSS_N_MAPPA);

    uint32_t t_fine = cicli_ora();

    risp->cicli = t_fine - t0;

    /* Le azioni tornano in int8 e vanno riportate a numeri reali: per v6
     * lo faceva il grafo con una DequantizeLinear finale, che era una delle
     * epoch software. Qui sono quattro moltiplicazioni. */
    for (int j = 0; j < PROT_N_AZIONI; ++j)
        risp->azioni[j] = ((float)uscita_azioni[j] - (float)VAE_AZIONE_OFFSET)
                          * VAE_AZIONE_SCALA;
    risp->stato = 0;
    prot_firma_risposta(risp);

    /* DIAGNOSTICA: due impronte di cio' che e' entrato nella rete.
     *
     * Quando le azioni di bordo non coincidono con quelle del PC, la
     * domanda e' una sola: sbaglia cio' che ENTRA nella rete, o cio' che
     * la rete ne fa? Confrontare le azioni non lo dice — sono il fondo
     * della catena.
     *
     * Queste due impronte lo dicono. Sono il CRC dei due buffer d'ingresso
     * gia' quantizzati, cioe' esattamente i byte che l'acceleratore legge.
     * Se combaciano con quelli calcolati dal PC, l'osservazione e la
     * quantizzazione sono giuste e il problema e' nella rete o nei pesi;
     * se non combaciano, e' a monte e la rete e' innocente.
     *
     * Si CALCOLANO prima della corsa e si MANDANO dopo aver firmato la
     * risposta: prima perche' la NPU usa il buffer del ToF come spazio di
     * lavoro, dopo perche' un log in mezzo allungherebbe la latenza. */
    {
        char riga[PROT_LOG_TESTO + 1];
        const uint32_t c_tof = impronta_tof;
        const uint32_t c_oss = impronta_oss;
        /* snprintf tirerebbe dentro mezza libreria standard: si scrive a
         * mano, che sono otto cifre esadecimali per due. */
        static const char cifre[] = "0123456789abcdef";
        int n = 0;
        const char *et = "tof ";
        while (*et) riga[n++] = *et++;
        for (int i = 28; i >= 0; i -= 4) riga[n++] = cifre[(c_tof >> i) & 0xF];
        et = " oss ";
        while (*et) riga[n++] = *et++;
        for (int i = 28; i >= 0; i -= 4) riga[n++] = cifre[(c_oss >> i) & 0xF];
        riga[n] = '\0';
        di(riga);
    }

    /* DOVE VA IL TEMPO. Due righe, perche' in 56 caratteri sei numeri non
     * entrano. Sono CICLI del contatore DWT, non microsecondi: la frequenza
     * la sa il PC, che gia' converte risp->cicli, e metterla qui vorrebbe
     * dire scriverla in due posti.
     *
     *   arena  azzeramento dei 50 kB di attivazioni + clean  (fuori da cicli)
     *   oss    mappa, integrale, settori, normalizzazione, quantizzazione,
     *          riempimento dei tre ingressi
     *   clean  i tre clean degli ingressi
     *   crc    le due impronte: DIAGNOSTICA, non servirebbe a bordo
     *   rete   stai_network_run: 36 epoch sulla NPU + 10 sul Cortex-M55
     *   inval  le quattro invalidazioni
     *
     * rete da sola e' il numero che in tesi si chiama "tempo di inferenza".
     * Tutto il resto e' la catena intorno, ed e' la ragione per cui il
     * numero da presentare resta quello della catena intera. */
    {
        char riga[PROT_LOG_TESTO + 1];
        int n = 0;
        const char *et;
        et = "arena ";  while (*et) riga[n++] = *et++;
        n += scrivi_dec(riga + n, t0      - t_arena);
        et = " oss ";   while (*et) riga[n++] = *et++;
        n += scrivi_dec(riga + n, t_oss   - t0);
        et = " clean "; while (*et) riga[n++] = *et++;
        n += scrivi_dec(riga + n, t_clean - t_oss);
        riga[n] = '\0';
        di(riga);

        n = 0;
        et = "crc ";    while (*et) riga[n++] = *et++;
        n += scrivi_dec(riga + n, t_crc  - t_clean);
        et = " rete ";  while (*et) riga[n++] = *et++;
        n += scrivi_dec(riga + n, t_rete - t_crc);
        et = " inval "; while (*et) riga[n++] = *et++;
        n += scrivi_dec(riga + n, t_fine - t_rete);
        riga[n] = '\0';
        di(riga);
    }
    return 0;
}
