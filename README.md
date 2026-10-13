# HIL-VAE — il progetto CubeIDE della scheda

Il controllore del VAE che gira sulla **NUCLEO-N657X0-Q**: riceve
dall'UART i 64 numeri che il drone non sa da se', costruisce
l'osservazione, fa girare la rete sull'acceleratore Neural-ART e risponde
con le quattro azioni. Dall'altra parte del cavo c'e' Isaac Lab, nel
repository della tesi, che aggancia questo progetto come submodule in
`deploy/scheda/`.

Il ramo min-pooling/v6 — quello che ha prodotto i numeri buoni, 2.69 ms per
passo sul Cortex-M55 — **non e' qui**: il suo C sta in `deploy/c/` e
`deploy/hil/` del repository della tesi. Un progetto CubeIDE pero' ce l'ha:
e' `UART_Printf_FSBL`, dentro l'albero del firmware package
(`.../Examples/UART/UART_Printf/STM32CubeIDE/FSBL/`), ed e' quello da cui
questo progetto e' stato copiato. Li' convivono i due rami — `nodo_bordo.c`
accanto a `nodo_bordo_vae.c` — piu' i file che qui non servono: `rete.c` e
`rete_data.c`, cioe' la rete del v6 **coi pesi dentro**, e i banchi di prova
`prova_rete.c` e `prova_scheda.c`. Quel progetto non e' versionato da
nessuna parte: chi riprende il v6 parta da li'.

## Cosa c'e'

    Application/Inc/      gli header: protocollo, osservazione, mappa,
                          config_bordo, piu' quelli generati (network.h,
                          stai_network.h) e la configurazione HAL
    Application/User/      il codice: main.c, ciclo_seriale.c,
                          nodo_bordo_vae.c, osservazione.c, mappa.c,
                          il runtime Neural-ART (ll_aton*.c, ll_sw_*.c)
                          e la rete generata (network.c, stai_network.c)
    Application/Startup/   lo startup FSBL
    STM32N657X0HXQ_AXISRAM2_fsbl.ld   il linker script
    UART_Printf_FSBL.launch           la configurazione di debug
    pesi/                  i 939 kB di pesi della rete, da caricare in RAM
                           a ogni avvio: vedi pesi/LEGGIMI.md
    UART_Printf.ioc        la configurazione CubeMX. E' quella dell'esempio
                           ST da cui questo progetto e' stato copiato, non
                           un .ioc di HIL_VAE_2: aprendola si vede la
                           configurazione vera della scheda (pinout, clock,
                           UART), non il progetto del VAE

`mappa.c` e `osservazione.c` sono gli stessi file che sul PC vengono
compilati in `libcervello.so`: questo repository e' la loro unica copia, e
il repository della tesi li compila da qui.

## Tre cose da sapere prima di premere Build

Il progetto **non e' autosufficiente**, e non lo e' per scelta: la
toolchain ST non si versiona. Chi lo apre si aspetti tre passaggi.

**1. Il firmware package va al posto giusto.** `.project` aggancia HAL,
BSP, CMSIS e `system_stm32n6xx_fsbl.c` come *linked resources* relative a
`${PARENT-3-PROJECT_LOC}/OneDrive/Desktop/STM/STM32Cube_FW_N6_V1.4.0/Projects/NUCLEO-N657X0-Q/Examples/UART/UART_Printf`.
Quei link si risolvono solo nella disposizione di cartelle della macchina
su cui il progetto e' nato. Su un'altra macchina vanno rifatti puntando al
proprio **STM32Cube_FW_N6 v1.4.0**, oppure si sposta il progetto dentro
l'albero degli esempi, da cui proviene (l'esempio `UART/UART_Printf`).

**2. Gli header del Neural-ART non sono nel repository.** `ll_aton.c`,
`network.c` e `stai_network.c` includono `ll_aton.h`, `ll_sw.h`,
`ll_aton_NN_interface.h` e compagnia, che arrivano da
`C:/ST/STEdgeAI/4.0/scripts/epoch_controller/include`. Attenzione a tre che
sfuggono, perche' il loro `.c` E' qui mentre il loro `.h` no: `mcu_cache.h`
e `npu_cache.h` stanno in `Middlewares/ST/AI/Npu/Devices/STM32N6xx/`,
`ecloader.h` in `Middlewares/ST/AI/Npu/ll_aton/`. Senza **ST Edge AI
Core 4.0** installato non compila nemmeno coi Drivers a posto.

**3. I pesi vanno caricati dal debugger, e questo e' il punto che inganna.**
`network.c` legge i pesi da `0x34200000`, che **non e' la flash esterna**:
e' npuRAM3, memoria volatile. La rete e' compilata col profilo
`n6-noextmem` apposta per tenerli li' invece che nella flash da programmare
a parte, e questo significa che a ogni avvio vanno riscritti — tre comandi
`restore` nella configurazione di debug, spiegati in `pesi/LEGGIMI.md`
insieme ai tre indirizzi.
I tre `.raw` adesso sono nel repository (`pesi/`, 939 kB), quindi chi clona
ha tutto. Se pero' la configurazione di debug non li carica, il progetto
compila, la scheda parte e la rete gira su memoria vuota: azioni plausibili
e prive di significato, senza un messaggio d'errore. Se l'anello si chiude e
i numeri sono assurdi, e' quasi sempre questo.

## Dettagli che confondono

- Il nome dentro `.project` e' **`HIL_VAE_2`**, non `HIL-VAE`: all'import
  in CubeIDE il progetto compare con quel nome.
- `UART_Printf_FSBL.launch` porta ancora il nome dell'esempio ST da cui si
  e' partiti. La configurazione di debug e' giusta, il nome e' storico.
- La UART e' a **921600 8N1**, non a 115200. A 115200 non si vede niente e
  sembra che la scheda sia morta.
- Sul filo passano i pacchetti, non il testo: al posto di `printf` c'e'
  `ciclo_seriale_log()`, che manda i messaggi dentro un terzo tipo di
  pacchetto. Il perche' sta in `deploy/hil/LEGGIMI.md`.
