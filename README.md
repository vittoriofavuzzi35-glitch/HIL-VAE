# HIL-STM32N6 — i due progetti CubeIDE della scheda

I controllori della tesi che girano sulla **NUCLEO-N657X0-Q**: ricevono
dall'UART cio' che il drone non sa da se', costruiscono l'osservazione,
fanno girare la rete sull'acceleratore Neural-ART e rispondono con le
quattro azioni. Dall'altra parte del cavo c'e' Isaac Lab, nel repository
della tesi, che aggancia questo come submodule in `deploy/scheda/`.

    min_pooling/   il ramo v6: dodici settori di min pooling
    vae/           il ramo definitivo: 64 latenti al posto dei settori

Fino al 16/10 qui c'era un progetto solo, il VAE, e stava nella radice.
I due rami adesso fanno **a bordo lo stesso lavoro** — stessa mappa,
stesso pooling, stessa quantizzazione, stesso epoch controller — ed e'
questo che rende confrontabili i loro tempi.

## I numeri, 16/10/2026

|  | min_pooling | vae |
|---|---|---|
| osservazione | 1084 | 1136 |
| catena a bordo | **2.635 ms** | **11.934 ms** |
| senza le impronte CRC | 2.635 (non le calcola) | **10.017 ms** |
| % del budget di 40 ms | 6.6% | 30.4% |
| di cui inferenza | non misurata | 8.052 ms |
| MAC | 375.394 | 4.220.462 |
| epoch | 12: 9 HW in 2 blob EC, 3 SW | 46: 36 HW in 4 blob EC, 10 SW |
| HIL, 100 missioni | **99/100** | **97/100** |
| simulazione, 8000 missioni | 93.15% | 97.56% |
| contatti, code scadute, pacchetti persi | 0 | 0 |

**ATTENZIONE A CONFRONTARE I TEMPI.** `nodo_bordo_vae.c` calcola due
impronte CRC32 dentro la finestra cronometrata, `nodo_bordo.c` no: sono
1,92 ms di sola diagnostica che pesano su un ramo e non sull'altro. Il
rapporto onesto e' **10,017 contro 2,635, cioe' 3,8x**.

## Cosa cambia fra i due progetti

| | min_pooling | vae |
|---|---|---|
| simboli | — | `CICLO_SERIALE_VAE`, `OSS_SENZA_SETTORI` |
| nome della rete | `--name rete` -> `stai_rete_*` | predefinito -> `stai_network_*` |
| ingressi | 2: scalari(60), mappa(1024) | 3: tof(4536), scalari(48), mappa(1024) |
| file dei pesi | un `.raw` | tre `.raw` |
| nodo | `nodo_bordo.c` | `nodo_bordo_vae.c` |
| ritardo del ToF | a bordo, un passo | non serve: i settori ritardati non entrano |

Comuni a entrambi: `LL_ATON_PLATFORM=LL_ATON_PLAT_STM32N6`,
`LL_ATON_OSAL=LL_ATON_OSAL_BARE_METAL`, `LL_ATON_SW_FALLBACK=1`,
`USE_HAL_DRIVER`, `STM32N657xx`.

**Niente `LL_ATON_RT_MODE`.** ASYNC e' il default del runtime ed e' cio'
che l'epoch controller pretende. Non serve configurare interrupt in
CubeMX: `ll_aton_runtime.c` definisce `NPU0_IRQHandler`, lo startup lo
dichiara `.weak`, e l'OSAL bare-metal abilita il NVIC da se'. Abilitarlo
anche in CubeMX genera un handler vuoto con lo stesso nome e il linker
trova due simboli forti.

## Caricare i pesi

I `.raw` **non** stanno nell'ELF: vanno scritti in npuRAM a mano, e solo
**dopo** che `npu_accendi()` ha dato clock all'acceleratore e configurato
i RISAF. Breakpoint sulla riga dopo quella chiamata, poi dalla console
gdb:

    min_pooling
      restore <percorso>/min_pooling/pesi/rete_atonbuf.AXISRAM5.raw binary 0x342E0000

    vae
      restore <percorso>/vae/pesi/network_atonbuf.AXISRAM3.raw binary 0x34200000
      restore <percorso>/vae/pesi/network_atonbuf.AXISRAM4.raw binary 0x34270000
      restore <percorso>/vae/pesi/network_atonbuf.AXISRAM5.raw binary 0x342E0000

Verificare **sempre** con `x/8xb <indirizzo>`: otto `0x00` o otto `0xff`
vogliono dire che il restore non e' passato, e da li' in poi la rete
lavora su pesi a zero **senza dare nessun errore**.

I pesi stanno in RAM: ogni reset della scheda li cancella. Fra un
`restore` e la fine di una corsa la sessione di debug non si tocca.

## Le configurazioni di debug

`min_pooling/HIL_POOL.launch` e' quella usata davvero il 16/10.
`vae/HIL_VAE_2.launch` e' **derivata da quella**, cambiando nome del
progetto ed ELF: nel progetto VAE la configurazione usata viveva nei
metadati del workspace e non in un file, e quella che c'era nella
cartella puntava a `Release/UART_Printf_FSBL.elf`, cioe' a un terzo
progetto. Controllare sempre, in *Main*, che **Project** e **C/C++
Application** siano quelli giusti: flashare l'ELF sbagliato non da'
errore, da' una scheda che risponde male.

## Cosa NON e' qui

Il codice sorgente condiviso — protocollo, osservazione, mappa, settori,
nodi — vive anche in `deploy/` del repository della tesi, che e' la
copia da cui si parte. Qui dentro c'e' quella **compilata**, insieme al
codice generato da ST Edge AI (`rete.c`, `network.c`, `stai_*`), che si
rigenera dall'ONNX e non si scrive a mano.

Le build (`Debug/`, `Release/`) non sono versionate.
