/* Il nodo di bordo: dal protocollo alle azioni.
 *
 * Due funzioni sole, e nessuna che sappia da dove arrivano i byte: e' il
 * confine fra il trasporto e il calcolo. Chi lo usa passa una richiesta
 * gia' letta dal cavo e riceve una risposta gia' firmata, pronta da
 * spedire. Lo stesso file gira sulla scheda e sul PC, ed e' il motivo per
 * cui l'anello si e' potuto verificare prima che l'hardware ci fosse.
 */
#ifndef NODO_BORDO_H
#define NODO_BORDO_H

#include "protocollo.h"

/* Apre il runtime, alloca la rete, azzera lo stato. Torna 0 se e' andata;
 * un negativo dice a che punto si e' fermata, e va riportato al PC invece
 * che ignorato: una rete che non si apre da' azioni plausibili ma false. */
int nodo_bordo_init(void);

/* Un passo: 64 float piu' la griglia 18x14 in, 4 azioni out. La risposta
 * e' sempre firmata, anche quando lo stato e' d'errore, perche' il PC deve
 * poter distinguere "la scheda ha un problema" da "il pacchetto si e'
 * perso".
 *
 * Il min pooling e il suo ritardo stanno DENTRO questa funzione, e finiscono
 * dentro i cicli riportati: il numero che torna al PC e' il costo della
 * catena intera, come lo era per v6 prima che il pooling scendesse a bordo.
 * Confrontarlo coi 2,69 ms di settembre va fatto sapendolo. */
int nodo_bordo_passo(const prot_richiesta_pool_t *rich, prot_risposta_t *risp);

/* Dove mandare i messaggi di testo del nodo. Se nessuno la chiama, il nodo
 * tace: cosi' su un banco di prova senza cavo non serve fornire nulla. */
void nodo_bordo_imposta_log(void (*scrivi)(const char *testo));

#endif
