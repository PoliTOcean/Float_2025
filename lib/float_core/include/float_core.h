#ifndef FLOAT_CORE_H
#define FLOAT_CORE_H

/*
 *******************************************************************************
 * float_core.h
 * API del control core tick-based. Due sole funzioni pure:
 *   - float_core_init(): azzera lo stato del modello.
 *   - float_core_step(): un passo del modello. Chiamata dall'HAL a passo fisso
 *     (pid_period_s). NON blocca, NON fa I/O: legge CoreInputs, aggiorna
 *     CoreState, produce CoreOutputs.
 *
 * Questa e' esattamente la firma che Embedded Coder generera' dal modello
 * Simulink/Stateflow: quando il modello sara' pronto, la libreria generata
 * rimpiazza float_core.c senza toccare l'HAL.
 *
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

#include "float_core_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Inizializza lo stato del core (mode = INIT). Chiamare una volta al boot. */
void float_core_init(CoreState* st);

/* Un passo del modello. in->cfg deve essere valido e non-NULL. */
void float_core_step(const CoreInputs* in, CoreState* st, CoreOutputs* out);

/* Helper di mappatura logica <-> passi, esposti per HAL e test.
 * u in [0,1]: u=0 -> home (galleggia), u=1 -> siringa piena (affonda).
 * Convenzione: u cresce verso posizioni motore NEGATIVE. */
long  float_core_u_to_motor_pos(const CoreConfig* cfg, float u);
float float_core_motor_pos_to_u(const CoreConfig* cfg, long position);

#ifdef __cplusplus
}
#endif

#endif /* FLOAT_CORE_H */
