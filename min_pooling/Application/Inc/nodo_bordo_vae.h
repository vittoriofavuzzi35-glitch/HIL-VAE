/* Il nodo di bordo del ramo VAE.
 *
 * Stesso confine di nodo_bordo.h — una richiesta gia' letta dal cavo, una
 * risposta gia' firmata — ma con un ingresso in piu': l'immagine di
 * profondita', che l'encoder guarda per intero.
 *
 * I DUE NODI NON CONVIVONO. Tengono entrambi una mappa e un integrale, e
 * due copie dello stesso stato che avanzano insieme sarebbero due droni
 * diversi che si contendono la stessa risposta. Nel progetto CubeIDE ne
 * entra UNO: o nodo_bordo.c, o questo.
 */
#ifndef NODO_BORDO_VAE_H
#define NODO_BORDO_VAE_H

#include "protocollo.h"

/* Apre il runtime, alloca la rete, azzera lo stato. Torna 0 se e' andata;
 * un negativo dice a che punto si e' fermata. Vale l'avvertenza di v6: una
 * rete che non si apre da' azioni plausibili ma false, quindi l'errore va
 * riportato al PC, non ingoiato. */
int nodo_bordo_vae_init(void);

/* Un passo: 64 float piu' 2268 profondita' in, 4 azioni out. */
int nodo_bordo_vae_passo(const prot_richiesta_vae_t *rich, prot_risposta_t *risp);

/* Dove mandare i messaggi di testo. Se nessuno la chiama, il nodo tace. */
void nodo_bordo_vae_imposta_log(void (*scrivi)(const char *testo));

#endif
