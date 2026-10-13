# I pesi della rete, e come finiscono sulla scheda

Tre file binari, 939 kB in tutto. Sono i pesi del ramo VAE quantizzato,
prodotti da ST Edge AI 4.0 dal modello a TRE ingressi
(`vae_unito_3in_int8.onnx`) il 2/10/2026.

    network_atonbuf.AXISRAM3.raw      21.841 byte  ->  0x34200000
    network_atonbuf.AXISRAM4.raw     458.744 byte  ->  0x34270000
    network_atonbuf.AXISRAM5.raw     458.744 byte  ->  0x342E0000

## Perche' non stanno dentro il programma

L'STM32N6 **non ha flash interna**: nella sigla N657X0 lo zero finale vuol
dire zero kB. Al suo posto ci sono 4.2 MB di SRAM contigua e l'acceleratore
Neural-ART. Tutto cio' che deve girare va quindi messo in RAM a ogni
accensione, da qualcuno.

Il ramo v6 non ha questo problema: i suoi 249 kB di pesi stanno nei 384 kB di
memoria programma, compilati dentro il firmware come array C in
`rete_data.c`. Il VAE ne ha 909 e non ci entra. Non e' un'ottimizzazione: e'
l'unico modo in cui il ramo VAE sta sulla scheda.

La rete e' stata compilata col profilo **n6-noextmem**, che tiene i pesi nei
banchi npuRAM3/4/5 invece di mandarli nella flash esterna. Quella strada
funzionerebbe (la flash da 64 MB sulla NUCLEO c'e') ma vorrebbe
l'ExtMemLoader di STM32CubeProgrammer e una programmazione a parte. Con
noextmem bastano tre comandi del debugger, e per l'HIL — dove il cavo e'
attaccato comunque — e' la via piu' corta. Il prezzo e' che a scheda spenta
non resta niente, ne' i pesi ne' il programma.

## Come si caricano

In CubeIDE: Run -> Debug Configurations -> Startup -> Initialization
Commands, aggiungere le tre righe

    restore <percorso>/pesi/network_atonbuf.AXISRAM3.raw binary 0x34200000
    restore <percorso>/pesi/network_atonbuf.AXISRAM4.raw binary 0x34270000
    restore <percorso>/pesi/network_atonbuf.AXISRAM5.raw binary 0x342E0000

**Barre in avanti**, anche su Windows: gdb vuole quelle.

Sono tre file e non uno perche' sono tre banchi fisici distinti, da 448 kB
l'uno, che la NPU legge in parallelo per avere piu' banda. L'abbinamento fra
file e indirizzo quindi conta: scambiarli non da' errore, da' una rete che
calcola numeri sbagliati.

## Il guasto che questi file evitano

Senza, il progetto compila, la scheda parte, il protocollo risponde e la
rete gira su memoria vuota: azioni plausibili e prive di significato,
nessun messaggio d'errore. E' il modo peggiore in cui una cosa puo'
rompersi, ed e' la prima ipotesi da verificare quando l'anello si chiude ma
i numeri sono assurdi.

## Se vanno rigenerati

Il comando sta in `deploy/hil/SULLA_SCHEDA.md` del repository della tesi,
sezione sulla generazione della rete. Due vincoli da non perdere: il modello
e' quello a tre ingressi, e **niente** `--enable-epoch-controller`, che e'
incompatibile con `LL_ATON_RT_POLLING` e fa fermare il runtime.

## Cosa NON c'e' qui

Il `.raw` del ramo v6 (`rete_atonbuf.AXISRAM5.raw`, 247.905 byte, in
`Prove/generato_npu/` sulla macchina di sviluppo). Esiste ma non e' mai
stato messo sulla scheda: il v6 che ha prodotto i numeri della tesi gira
interamente sul Cortex-M55 coi pesi dentro `rete_data.c`. Tenerlo accanto a
questi inviterebbe a confondere i due oggetti — ed e' esattamente l'origine
della confusione fra i 2.281 ms misurati e i 0.3439 ms stimati.
