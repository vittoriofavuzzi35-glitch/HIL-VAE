/* Il ciclo seriale: l'unico pezzo che tocca il cavo.
 *
 * Tutto il resto dell'HIL e' indifferente al trasporto — nodo_bordo.c
 * riceve strutture e ne restituisce, e per questo si e' potuto verificare
 * sul PC prima che la scheda ci fosse. Qui invece si legge e si scrive
 * davvero, e le cose che possono andare male sono quelle del mondo fisico:
 * byte persi, disallineamenti, un baud sbagliato da una parte sola.
 *
 * COME SI USA. Nel main, dopo MX_USART1_UART_Init:
 *
 *     ciclo_seriale_init(&huart1);
 *     ciclo_seriale_gira();          // non torna mai
 *
 * LA UART DEVE ESSERE 921600 8N1. L'esempio UART_Printf di ST la mette a
 * 7 bit con parita' dispari, e con sette bit i byte oltre il 127 non
 * passano: un float su due arriverebbe mutilato. Sul baud non e' una
 * questione di eleganza: fra andata e ritorno sono 316 byte per passo, che
 * a 115200 sono 27 ms su un budget di 40, mentre a 921600 sono 3,4.
 */
#ifndef CICLO_SERIALE_H
#define CICLO_SERIALE_H

#include "main.h"

/* Prende la UART gia' inizializzata dal codice generato, apre il nodo e
 * manda al PC il primo messaggio di stato. Torna 0 se il nodo e' pronto.
 * Se non lo e' gira comunque: il PC deve poter leggere il motivo, e una
 * scheda muta non dice niente a nessuno. */
int ciclo_seriale_init(UART_HandleTypeDef *uart);

/* Legge richieste e risponde, per sempre. Non torna. */
void ciclo_seriale_gira(void);

/* Manda una riga di testo al PC, incapsulata nel protocollo. Si usa al
 * posto di printf: sullo stesso filo passano i pacchetti, e due mittenti
 * che scrivono insieme si corrompono a vicenda. */
void ciclo_seriale_log(const char *testo);

#endif
