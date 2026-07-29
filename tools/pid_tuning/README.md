# Tuning PID del Float — guida + tool

Strumento **plug-and-play** per chi deve tarare il controllo di profondità del Float **senza essere esperto di controlli automatici**.

Due cose:
1. Un **notebook** (gira su Google Colab, niente da installare) che ti **dà i valori da mettere nella GUI** e **analizza i log** dei tuoi test dicendoti cosa correggere.
2. Questa guida, che spiega **ogni parametro** e **cosa succede se lo cambi**.

---

## 🚀 Apri il tool su Google Colab

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/PoliTOcean/Float/blob/master/tools/pid_tuning/pid_tuning.ipynb)

1. Clicca il badge qui sopra (o apri il link).
2. In alto: **Runtime → Esegui tutto** (`Runtime → Run all`).
3. Nella sezione 4 dai i tuoi dati in **uno** di questi modi:
   - **Incolla** nella variabile `DATI` la **tabella** copiata dalla vista *"Raw chart"* della GUI (o un JSON), **oppure**
   - lascia `DATI` vuoto e **carica un CSV** (export GUI o log flash `DUMP_LOG`).
   - Se non metti nulla, usa un log di esempio così vedi subito come funziona.
4. Il notebook **riconosce da solo le fasi** del log: solo discesa, solo salita o profilo completo (discesa → hold → salita), con metriche e diagnosi **per fase**. Imposti due target come nella GUI (`target_discesa_m` riferito al FONDO, `target_salita_m` riferito al TOP, convertito da solo); con `fase = "discesa"` o `"salita"` puoi forzare l'analisi su una sola fase. L'eventuale riemersione finale in superficie viene esclusa dall'analisi della salita.

> Uso locale (senza Colab): `pip install -r requirements.txt` e poi `jupyter notebook pid_tuning.ipynb`.

---

## ⚠️ Premessa importante: dove sta davvero il float

Il **barometro è in cima al float** (lungo ~0,51 m). Il controllo ragiona sul **fondo del float**:

- **Target di DISCESA** = profondità che deve raggiungere il **FONDO** del float. Quindi quando sei "a target", il barometro in cima legge **target − 0,51 m**.
- **Target di RISALITA** = riferito al **TOP** del float (barometro).

**Conseguenza pratica:** in una vasca bassa (< ~1 m) il barometro resta a pochi centimetri dal pelo dell'acqua → letture rumorose, il sensore può uscire dall'acqua → **il float oscilla e questo NON è colpa del PID**. Per tarare sul serio servono **almeno ~1,5–2 m** di acqua. (Es.: target discesa 0,55 in vasca da 0,8 m mette il sensore a soli ~4 cm dal pelo: condizione impossibile da stabilizzare.)

---

## 📖 I tre concetti minimi

- **`u` (apertura siringa)** = quanto la siringa è "piena", da **0** a **1**. `u = 0` → siringa vuota → **galleggia**; `u = 1` → siringa piena → **affonda**. È l'**uscita** che il controllo calcola da solo; nei log è una colonna. **Non si imposta a mano.**
- **PID** = il "cervello" che decide `u` in base all'errore di profondità (quanto sei lontano dal target). Ha tre manopole: **P** (reazione all'errore ora), **I** (recupero dell'errore che persiste), **D** (freno/anticipo sulle variazioni).
- **`u_neutral`** = l'apertura siringa di **assetto neutro** (quella a cui il float né sale né scende). È un **parametro** da impostare; il notebook lo **stima dai dati**. Diverso da `u`!

---

## 🎛️ Parametri PID (campo per campo)

Valori di **default** e **range validi** presi dal firmware (`include/config.h`, `lib/runtime_config/src/runtime_config.cpp`).

| Parametro | Cosa fa | Default | Range valido | Se lo **aumenti** | Se lo **diminuisci** |
|---|---|---|---|---|---|
| **kp** | Forza della reazione all'errore di profondità | `1.7` | > 0 | Reagisce più pronto, ma **oscilla / overshoot** se troppo | Più lento e dolce, ma può restare un **offset** |
| **ki** | Recupera l'errore che **persiste** nel tempo (offset) | `0.1` | ≥ 0 | Elimina l'offset; troppo → **oscillazione lenta / overshoot** | Resta un errore stazionario (float un po' troppo alto/basso) |
| **kd** | **Smorza**: frena in base alla velocità di avvicinamento | `0.3` | ≥ 0 | Riduce **oscillazioni e overshoot**; troppo → amplifica il **rumore** | Più overshoot e oscillazione |
| **period_ms** | Ogni quanto ricalcola il comando | `50` | `[20, 500]` | Reagisce **meno spesso** (più lento) | Più reattivo, ma più rumore/carico |
| **alpha_d** | Filtro sulla derivata (0 = liscio, 1 = grezzo) | `0.25` | `[0.05, 1.0]` | Derivata più "viva" ma più **rumorosa** | Derivata più liscia ma più in **ritardo** |
| **integral_limit** | Tetto anti-windup dell'integrale | `5.0` | > 0 | L'integrale può accumulare di più (recupero forte, rischio overshoot) | Limita il recupero dell'offset |
| **min_retarget_frac** | Zona morta: di quanto deve cambiare il comando prima di muovere il motore | `0.001` | ≥ 0 | Meno micro-movimenti (motore più fermo), meno preciso | Insegue ogni minima variazione: più preciso, più usura |
| **u_neutral** | Apertura siringa di **base** (assetto neutro / spinta iniziale) | `0.011` | ≥ 0 (utile ≤ 0.92) | Parte più "**affondante**" | Parte più "**galleggiante**" |

> Nota: l'uscita `u` è sempre limitata dal firmware a **[0 ; 0,92]** (margine di sicurezza dai finecorsa/TOF). Se nei log vedi `u` incollata a 0 o 0,92, il problema è di **assetto/zavorra**, non dei guadagni.

---

## 🌊 Parametri del profilo (PROFILE_SET)

| Parametro | Cosa fa | Default | Range valido | Note |
|---|---|---|---|---|
| **descent_target** | Profondità del **FONDO** del float in discesa | `2.5` m | `[0, 5]` | Vedi premessa geometria (+0,51 m) |
| **ascent_target** | Profondità del **TOP** del float in risalita | `0.40` m | `[0, 5]` | Riferito al barometro |
| **depth_tolerance** | Semi-banda ± attorno al target per dirsi "a target" | `0.33` m | `[0.005, 1.0]` | In vasca **abbassala** (es. `0.05`) o "a target" non significa niente |
| **hold_time** | Quanto resta fermo al target | `30` s | `[1, 600]` | Il check scatta ogni 5 s |
| **descent_timeout** | Tempo massimo della fase di discesa | `180` s | `[5, 900]` | Hold incluso |
| **ascent_timeout** | Tempo massimo della fase di risalita | `120` s | `[5, 900]` | Hold incluso |
| **surface_offset** | Quanto il top del float resta sotto il pelo a riposo | `0.10` m | `[0, 5]` | — |

> **Vincolo del firmware:** `ascent_target + 0,51 < descent_target` (la risalita deve restare più in alto della discesa), altrimenti la GUI rifiuta la configurazione.

---

## 🛠️ Ricetta di tuning (passo per passo)

1. **Prima l'assetto (`u_neutral`).** Trova l'apertura siringa a cui il float **né sale né scende** alla quota di prova (osserva dove la colonna `u` si stabilizza in un mantenimento, oppure fai uno sweep manuale). Imposta `u_neutral` ≈ quel valore. **Senza un buon assetto nessun guadagno funziona.**
2. **Parti dai default** della tabella PID.
3. **Regola una manopola alla volta**, guardando il comportamento:
   - **Oscilla** con ampiezza che non si spegne → **riduci kp** (×0,7) oppure **aumenta kd** (×1,5).
   - **Lento** e resta un **offset** (galleggia troppo alto/basso) → **aumenta ki** (×1,5–2).
   - **Overshoot** grande e poi si assesta → **aumenta kd**, eventualmente riduci un po' kp.
   - `u` sempre a **0 o 0,92** → **non è il PID**: è l'assetto/zavorra o `u_neutral`.
4. **Fai il test**, poi **esporta il log** dalla GUI.
5. **Carica il log nel notebook**: leggi la diagnosi, applica i **valori suggeriti**, e ripeti dal punto 3.

---

## 📤 Quali dati dare al notebook

Hai due sorgenti possibili.

### A) Incollare la tabella dalla GUI — modo più semplice
Nel pannello *Profile Data Log* attiva lo switch **"Raw chart"**: compare una **tabella**. Selezionala, copiala e incollala nel notebook (variabile `DATI`). Le righe sono tipo:

```
Time (s)   Depth     Pressure    Syringe
12.40      0.51 m    101.30 kPa  0.30 u
```

Il notebook è robusto: capisce **2, 3 o 4 colonne**, con o senza unità (`m`, `kPa`), timestamp in secondi o **millisecondi**, separatori spazi/virgole/tab, decimali con la virgola, righe `N/A`, e `u` anche in percentuale (`30` → `0.30`). Accetta pure il `raw` in formato **JSON**.

> La tabella "Raw chart" include la colonna **Syringe (`u`)**: incollandola ottieni il tuning completo (`kp/ki/kd`, `u_neutral` e controllo saturazione). Se per qualche motivo la colonna `u` manca, il notebook analizza comunque la profondità e ti avvisa di usare il log flash per il resto.

### B) Caricare il log flash del Float (CSV, consigliato per il tuning completo)
Il Float salva su memoria flash, **dopo ogni profilo**, un CSV a **8 colonne** (`lib/flash_storage/`):

```
company_number, profile_id, time_s, pressure_kpa, depth_m, phase, sensor_depth_m, syringe_u
```

- **`depth_m`** = profondità del **FONDO** del float — quella su cui ragiona il PID, confrontata col target.
- **`sensor_depth_m`** = profondità del **barometro** (cima del float).
- **`phase`** = stato (`descending`, `hold_2_5m`, `ascending`, `emergency_stop:...`): se compare un emergency stop, il notebook **te lo segnala**.
- **`syringe_u`** = apertura siringa `u` → **serve per `u_neutral` e per la saturazione**.

Il notebook riconosce **per nome di colonna** sia il JSON `raw` della GUI, sia il CSV a 8 colonne, sia un export ridotto (`timestamp, profondità, pressione, syringe/u`), con separatore `,` o `;`.

> La tabella della GUI (modo A) basta per il tuning completo, colonna `Syringe` inclusa. Il log flash (modo B) resta utile come sorgente alternativa: ha anche `phase` (segnala gli emergency stop) e `sensor_depth_m`.

---

## 🔒 Sicurezza e limiti

- L'uscita `u` è limitata a **0,92** e c'è una **guardia TOF** che ferma il pistone ai finecorsa: non forzare oltre.
- **Vasca troppo bassa = test inaffidabile** (vedi premessa geometria).
- I valori che il tool propone restano **dentro i range accettati dal firmware**; se inserisci a mano valori fuori range, la GUI/firmware li rifiuta.

---

*Manutentori: Team PoliTOcean. I default e i limiti sono allineati al firmware in `include/config.h`, `lib/runtime_config/`, `lib/profile/`.*
