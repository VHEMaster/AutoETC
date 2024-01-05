/*
 * outputs.h
 *
 *  Created on: Mar 6, 2022
 *      Author: VHEMaster
 */

#ifndef INC_OUTPUTS_H_
#define INC_OUTPUTS_H_

#include "main.h"
#include "structs.h"

typedef enum {
  OutCruizeG = 0,
  OutCruizeR,
  OutRsvd3,
  OutRsvd4,
  OutputCount
}eOutput;

void out_set_state(eOutput channel, GPIO_PinState state);
GPIO_PinState out_get_state(eOutput channel, uint32_t *time);

HAL_StatusTypeDef outputs_get_diagnostic(sEtcOutputDiagnostic *diagnostic);

void outputs_init(void);
HAL_StatusTypeDef outputs_register(eOutput output, GPIO_TypeDef *port, uint16_t pin, uint8_t inverted, GPIO_PinState initial);
void outputs_loop(void);

#endif /* INC_OUTPUTS_H_ */
