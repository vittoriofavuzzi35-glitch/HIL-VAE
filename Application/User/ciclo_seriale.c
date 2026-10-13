#include <string.h>

#include "ciclo_seriale.h"
#include "protocollo.h"

/* QUALE RAMO GIRA, si sceglie a compilazione.
 *
 * Definendo CICLO_SERIALE_VAE il ciclo parla il formato "MV6N", che porta
 * anche i 2268 valori di profondita', e chiama nodo_bordo_vae. Senza,
 * resta esattamente il ciclo di v6, byte per byte.
 *
 * Perche' a compilazione e non a runtime: i due nodi tengono ciascuno una
 * mappa e un integrale, e due copie dello stesso stato che avanzano
 * insieme sarebbero due droni diversi che si contendono la risposta. Ne
 * entra UNO nel progetto. In cambio non si paga il buffer piu' grande
 * quando si gira v6: 9352 byte contro 280 sono RAM che sulla scheda non
 * avanza. */
#if defined(CICLO_SERIALE_VAE)
  #include "nodo_bordo_vae.h"
  #define RICHIESTA_T       prot_richiesta_vae_t
  #define RICHIESTA_MAGIA   PROT_MAGIA_RICH_VAE
  #define RICHIESTA_VALIDA  prot_richiesta_vae_valida
  #define NODO_INIT         nodo_bordo_vae_init
  #define NODO_PASSO        nodo_bordo_vae_passo
  #define NODO_IMPOSTA_LOG  nodo_bordo_vae_imposta_log
  #define RAMO_DETTO        "in ascolto, 921600 8N1, ramo VAE"
#else
  #include "nodo_bordo.h"
  #define RICHIESTA_T       prot_richiesta_t
  #define RICHIESTA_MAGIA   PROT_MAGIA_RICH
  #define RICHIESTA_VALIDA  prot_richiesta_valida
  #define NODO_INIT         nodo_bordo_init
  #define NODO_PASSO        nodo_bordo_passo
  #define NODO_IMPOSTA_LOG  nodo_bordo_imposta_log
  #define RAMO_DETTO        "in ascolto, 921600 8N1"
#endif

/* Quanto si aspetta il resto di un pacchetto di cui si e' gia' vista la
 * magia. Serve solo a non restare bloccati per sempre se il cavo viene
 * staccato a meta' di un pacchetto — ma DEVE essere piu' lungo del tempo
 * che quel pacchetto impiega davvero ad arrivare:
 *
 *     v6     276 byte che restano  ->    3 ms a 921600 8N1
 *     VAE   9348 byte che restano  ->  101 ms
 *
 * I 50 ms bastavano per v6 e sono TROPPO POCHI per il VAE. Con quel valore
 * la scheda vede la magia, prova a leggere il corpo, scade a meta', conta un
 * pacchetto guasto e ricomincia a cercare la magia — senza rispondere. Dal
 * PC si vede come silenzio assoluto, che e' lo stesso sintomo di una scheda
 * spenta: e' costato mezz'ora il 29/9. */
#if defined(CICLO_SERIALE_VAE)
  #define ATTESA_CORPO_MS  500u
#else
  #define ATTESA_CORPO_MS   50u
#endif

/* La scrittura invece deve riuscire: se il PC non legge, restare qui non
 * peggiora niente, mentre rinunciare lascerebbe un pacchetto a metà sul
 * filo e manderebbe l'altro capo fuori sincrono. */
#define ATTESA_SCRITTURA_MS 200u

static UART_HandleTypeDef *porta;
static RICHIESTA_T richiesta;
static prot_risposta_t risposta;
static uint32_t ultima_sequenza;

/* Contatori di cio' che e' andato storto. Non si stampano a ogni passo —
 * sarebbero migliaia di righe — ma vanno nel log quando cambiano: un
 * numero che cresce di continuo e' il sintomo di un baud diverso fra le
 * due parti, e senza contarlo si finisce a sospettare la rete. */
static uint32_t byte_scartati;
static uint32_t pacchetti_guasti;

void ciclo_seriale_log(const char *testo)
{
    prot_log_t pacchetto;
    if (!porta) return;
    prot_firma_log(&pacchetto, ultima_sequenza, testo);
    HAL_UART_Transmit(porta, (uint8_t *)&pacchetto, sizeof pacchetto,
                      ATTESA_SCRITTURA_MS);
}

int ciclo_seriale_init(UART_HandleTypeDef *uart)
{
    int esito;

    porta = uart;
    ultima_sequenza = 0;
    byte_scartati = 0;
    pacchetti_guasti = 0;

    /* Il log si collega PRIMA di aprire il nodo, cosi' se l'apertura
     * fallisce il motivo arriva al PC invece di perdersi. */
    NODO_IMPOSTA_LOG(ciclo_seriale_log);

    esito = NODO_INIT();
    if (esito != 0) {
        char riga[PROT_LOG_TESTO + 1];
        /* Senza printf: la scheda non ha una libreria di formattazione
         * collegata, e tirarla dentro per un numero solo costa kilobyte. */
        strcpy(riga, "nodo non pronto, esito -");
        riga[24] = (char)('0' + (-esito % 10));
        riga[25] = '\0';
        ciclo_seriale_log(riga);
    }
    return esito;
}

/* Un byte, aspettando quanto serve. Torna 0 se non e' arrivato. */
static int prendi_byte(uint8_t *b, uint32_t attesa_ms)
{
    return HAL_UART_Receive(porta, b, 1, attesa_ms) == HAL_OK;
}

/* Cerca l'inizio di una richiesta e la legge intera.
 *
 * SI CERCA LA MAGIA, NON SI CONTANO I BYTE. Se un byte si perde, leggere a
 * blocchi di 280 lascerebbe sfasati per sempre: ogni pacchetto seguente
 * sarebbe letto a cavallo di due, e tutti i CRC sarebbero sbagliati. Con
 * una finestra di quattro byte che scorre di uno ci si riallinea al
 * pacchetto dopo, perdendone uno solo.
 *
 * Torna 1 se in 'richiesta' c'e' un pacchetto intero — non ancora
 * validato: della magia si e' sicuri, del resto no.
 */
static int prendi_richiesta(void)
{
    uint8_t finestra[4];
    uint8_t b;
    uint32_t magia;
    int riempiti = 0;

    for (;;) {
        /* Senza scadenza: fra un episodio e l'altro il PC puo' tacere a
         * lungo, e una scadenza qui vorrebbe solo dire tornare indietro per
         * rientrare subito. */
        if (!prendi_byte(&b, HAL_MAX_DELAY)) continue;

        if (riempiti < 4) {
            finestra[riempiti++] = b;
        } else {
            finestra[0] = finestra[1];
            finestra[1] = finestra[2];
            finestra[2] = finestra[3];
            finestra[3] = b;
            ++byte_scartati;
        }
        if (riempiti < 4) continue;

        memcpy(&magia, finestra, 4);
        if (magia != RICHIESTA_MAGIA) continue;

        memcpy(&richiesta, finestra, 4);
        if (HAL_UART_Receive(porta,
                             (uint8_t *)&richiesta + 4,
                             (uint16_t)(sizeof richiesta - 4),
                             ATTESA_CORPO_MS) != HAL_OK) {
            /* Pacchetto troncato: si ricomincia a cercare la magia. La
             * finestra si svuota, altrimenti i suoi quattro byte vecchi
             * verrebbero riesaminati come se fossero nuovi. */
            ++pacchetti_guasti;
            riempiti = 0;
            continue;
        }
        return 1;
    }
}

void ciclo_seriale_gira(void)
{
    uint32_t guasti_detti = 0;
    uint32_t scartati_detti = 0;

    ciclo_seriale_log(RAMO_DETTO);

    for (;;) {
        if (!prendi_richiesta()) continue;

        if (RICHIESTA_VALIDA(&richiesta) != 0) {
            /* Si risponde comunque, con lo stato d'errore e la sequenza
             * letta cosi' com'e'. Il PC ha bisogno di distinguere "la
             * scheda ha rifiutato il pacchetto" da "il pacchetto non e'
             * mai arrivato": nel primo caso il guasto e' sul cavo, nel
             * secondo puo' essere la scheda bloccata.
             *
             * La sequenza viene da un pacchetto che non ha passato il CRC,
             * quindi potrebbe essere falsa: e' proprio per questo che il PC
             * la confronta con quella che aspettava. */
            ++pacchetti_guasti;
            memset(&risposta, 0, sizeof risposta);
            risposta.sequenza = richiesta.sequenza;
            risposta.stato = -2;
            prot_firma_risposta(&risposta);
            HAL_UART_Transmit(porta, (uint8_t *)&risposta, sizeof risposta,
                              ATTESA_SCRITTURA_MS);
            continue;
        }

        ultima_sequenza = richiesta.sequenza;
        NODO_PASSO(&richiesta, &risposta);
        HAL_UART_Transmit(porta, (uint8_t *)&risposta, sizeof risposta,
                          ATTESA_SCRITTURA_MS);

        /* I guai si raccontano dopo aver risposto, mai prima: un log in
         * mezzo allungherebbe la latenza del passo proprio quando il cavo
         * sta gia' dando problemi. */
        if (pacchetti_guasti != guasti_detti) {
            guasti_detti = pacchetti_guasti;
            ciclo_seriale_log("attenzione: pacchetti rifiutati sul cavo");
        }
        if (byte_scartati > scartati_detti + 64u) {
            scartati_detti = byte_scartati;
            ciclo_seriale_log("attenzione: molti byte scartati, baud diverso?");
        }
    }
}
