# MIL nativo — anello chiuso control core + plant (host, no MATLAB)

Harness di simulazione **host-only** che chiude l'anello tra:

- il **control core** portabile (`lib/float_core/`, C puro, tick-based) e
- un **plant** del float (`plant.c`): attuatore motore rate-limited (~2 mm/s),
  dinamica di profondità del 2° ordine, modelli sensore TOF e pressione.

È il primo anello della catena MBSD **prima** di aprire Simulink: dà feedback
immediato sul comportamento del core ed è la "specifica eseguibile" che il
modello Stateflow/Simulink dovrà riprodurre (equivalenza MIL).

## Build & run

Serve un compilatore C (`gcc`/`cc`) e, opzionalmente, `make`.

```bash
# con make
make -C tools/mil run

# oppure a mano
cd tools/mil
gcc -std=c11 -O2 -I../../lib/float_core/include -I. \
    mil_main.c plant.c ../../lib/float_core/src/float_core.c -lm -o build/float_mil
./build/float_mil > build/mission.csv
```

Esegue: `boot → homing → IDLE → GO → 2 profili (discesa 2.5 m / hold /
risalita / hold) → sosta finale → IDLE`, stampa il CSV della missione e un
riepilogo con la verifica dei requisiti MATE.

## Cosa NON è

Non usa Embedded Coder né MATLAB: il core qui è l'implementazione di
**riferimento** scritta a mano (`float_core.c`). Quando il modello Simulink
sarà pronto, il codice generato rimpiazzerà `float_core.c` mantenendo la stessa
firma `float_core_step()`, e questo stesso harness servirà a confrontare i due
(back-to-back, verso il SIL).

## Colonne del CSV

`t_s, mode, phase, depth_m, target_m, u, motor_pos, tof_mm, stable_s`
