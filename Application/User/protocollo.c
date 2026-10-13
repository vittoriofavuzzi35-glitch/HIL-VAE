#include <string.h>
#include "protocollo.h"

uint32_t prot_crc32(const void *dati, uint32_t n_byte)
{
    const uint8_t *p = (const uint8_t *)dati;
    uint32_t c = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n_byte; ++i) {
        c ^= p[i];
        for (int k = 0; k < 8; ++k)
            c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1u)));
    }
    return c ^ 0xFFFFFFFFu;
}

/* Il CRC copre tutto tranne se stesso, che sta in fondo. */
#define SENZA_CRC(T) ((uint32_t)(sizeof(T) - sizeof(uint32_t)))

void prot_firma_richiesta(prot_richiesta_t *r)
{
    r->magia = PROT_MAGIA_RICH;
    r->versione = PROT_VERSIONE;
    r->crc = prot_crc32(r, SENZA_CRC(prot_richiesta_t));
}

void prot_firma_richiesta_vae(prot_richiesta_vae_t *r)
{
    r->magia = PROT_MAGIA_RICH_VAE;
    r->versione = PROT_VERSIONE;
    r->crc = prot_crc32(r, SENZA_CRC(prot_richiesta_vae_t));
}

void prot_firma_risposta(prot_risposta_t *r)
{
    r->magia = PROT_MAGIA_RISP;
    r->crc = prot_crc32(r, SENZA_CRC(prot_risposta_t));
}

void prot_firma_log(prot_log_t *r, uint32_t sequenza, const char *testo)
{
    uint32_t n = 0;
    while (testo[n] != '\0' && n < (uint32_t)PROT_LOG_TESTO) ++n;
    memset(r, 0, sizeof *r);
    r->magia = PROT_MAGIA_LOG;
    r->sequenza = sequenza;
    r->lunghezza = n;
    memcpy(r->testo, testo, n);
    r->crc = prot_crc32(r, SENZA_CRC(prot_log_t));
}

int prot_richiesta_valida(const prot_richiesta_t *r)
{
    if (r->magia != PROT_MAGIA_RICH) return -1;
    /* La versione si controlla PRIMA del CRC: se un giorno il formato
     * cambia, il messaggio utile e' "versione sbagliata", non "pacchetto
     * corrotto", che manderebbe a cercare un problema sul cavo. */
    if (r->versione != PROT_VERSIONE) return -2;
    if (r->crc != prot_crc32(r, SENZA_CRC(prot_richiesta_t))) return -3;
    return 0;
}

int prot_richiesta_vae_valida(const prot_richiesta_vae_t *r)
{
    /* Stessi controlli e stesso ordine della richiesta di v6: la magia
     * diversa e' cio' che impedisce di scambiare i due formati, che hanno
     * dimensioni cosi' lontane (280 contro 9352) che un errore qui si
     * manifesterebbe come un cavo che perde byte. */
    if (r->magia != PROT_MAGIA_RICH_VAE) return -1;
    if (r->versione != PROT_VERSIONE) return -2;
    if (r->crc != prot_crc32(r, SENZA_CRC(prot_richiesta_vae_t))) return -3;
    return 0;
}

int prot_risposta_valida(const prot_risposta_t *r)
{
    if (r->magia != PROT_MAGIA_RISP) return -1;
    if (r->crc != prot_crc32(r, SENZA_CRC(prot_risposta_t))) return -3;
    return 0;
}

int prot_log_valido(const prot_log_t *r)
{
    if (r->magia != PROT_MAGIA_LOG) return -1;
    /* Una lunghezza fuori scala si scarta prima di usarla come indice:
     * il pacchetto arriva da un cavo, non da un amico. */
    if (r->lunghezza > (uint32_t)PROT_LOG_TESTO) return -2;
    if (r->crc != prot_crc32(r, SENZA_CRC(prot_log_t))) return -3;
    return 0;
}
