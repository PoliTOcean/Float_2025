# model/ — Modello Simulink/Stateflow del control core (Fase 1: MIL)

Questo è il mondo Model-Based del float. Riproduce in Simulink/Stateflow il
**control core** già validato in C (`lib/float_core/float_core.c`) e in
simulazione nativa (`tools/mil/`). Il core in C resta la **specifica eseguibile
di riferimento**: il modello deve dare la stessa traccia (equivalenza MIL), e in
seguito il codice generato da Embedded Coder rimpiazzerà `float_core.c`.

## Struttura del testbench `float_mil.slx`

```
              ┌─────────────────────────────┐
   In (bus)   │  float_core   (Stateflow)   │  Out (bus)
  ┌──────────►│  controllore, Ts = 0.05 s   ├──────────┐
  │           │  INIT/HOMING/IDLE/MISSION/  │          │
  │           │  SAFE_STOP + PID            │          │
  │           └─────────────────────────────┘          │
  │                                                     ▼
  │           ┌─────────────────────────────┐
  └───────────┤  float_plant (MATLAB Fcn)   │◄─────────┘
      In      │  attuatore + 2° ordine +    │   Out.motor_*
   (sensori)  │  sensori TOF/pressione      │
              └─────────────────────────────┘
```

- **`float_core`** — chart Stateflow. Ingresso: bus `In` (= `CoreInputs` senza
  `cfg`). Uscita: bus `Out` (= `CoreOutputs`). `cfg` è un **parametro** (struct
  `P`), non un segnale. Spec completa in [STATEFLOW_CHART.md](STATEFLOW_CHART.md).
- **`float_plant`** — blocco MATLAB Function. Codice in
  [plant/float_plant_fcn.m](plant/float_plant_fcn.m): incolla il contenuto nel
  blocco. Riceve `Out` + `P` (plant params `PL`), restituisce il nuovo `In`.
- **Trigger** — una sorgente che, dopo l'homing (quando `Out.phase`==IDLE e il
  plant è pronto), emette un impulso `event = GO` una volta. Vedi run_mil.m.

## Ordine di costruzione (in MATLAB)

1. **Parametri e bus**: esegui [`params/core_params.m`](params/core_params.m).
   Crea nel base workspace: `P` (config del core), `PL` (config del plant), e i
   bus `CoreInputsBus` / `CoreOutputsBus`.
2. **Nuovo modello** `float_mil.slx`, sample time discreto fisso `P.pid_period_s`.
3. Aggiungi il **chart Stateflow** `float_core` seguendo STATEFLOW_CHART.md.
   - Imposta ingresso `In` (bus `CoreInputsBus`), uscita `Out` (`CoreOutputsBus`),
     parametro `P` (Data, scope Parameter).
4. Aggiungi il blocco **MATLAB Function** `float_plant`, incolla
   `plant/float_plant_fcn.m`. Parametro `PL`.
5. **Chiudi l'anello**: `float_core.Out → float_plant`, `float_plant → In →
   float_core`. Usa un blocco **Unit Delay** (o Memory) sul ramo di retroazione
   per rompere l'algebraic loop (In dipende da Out dello stesso tick).
6. Logga almeno `In.depth_m`, `Out.phase`, `In.motor_pos`, `Out.completed_profiles`
   (Signal Logging o To Workspace: vedi run_mil.m per i nomi attesi).
7. **Esegui** [`tests/run_mil.m`](tests/run_mil.m): lancia la sim e verifica i
   requisiti MATE (2.5 m, 0.88 m, sosta, no rottura superficie).

## Roadmap dopo il MIL

- **SIL**: Embedded Coder genera C/C++ dal chart `float_core`; test back-to-back
  contro il modello E contro `lib/float_core/float_core.c` (deve coincidere).
- **PIL**: stesso codice cross-compilato sull'ESP32, plant in Simulink via seriale.
- **HIL**: core generato sull'ESP32 con driver reali (l'HAL di `src/espA_core/`).

> Nota: gli `.m` sono scritti da riferimento ma NON eseguiti in questo ambiente.
> Al primo `run` potrebbero servire micro-aggiustamenti (nomi segnali, versione
> MATLAB). La logica è quella, validata in `tools/mil`.
