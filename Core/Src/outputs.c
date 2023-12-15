/*
 * outputs.h
 *
 *  Created on: Mar 6, 2022
 *      Author: VHEMaster
 */

#include "outputs.h"
#include "misc.h"
#include "delay.h"
#include <string.h>
#include <limits.h>

typedef struct {
  GPIO_TypeDef *port;
  volatile GPIO_PinState state;
  uint32_t time_switched;
  uint16_t pin;
  uint8_t inverted;
}sOutput;

static sOutput Outputs[OutputCount] = {{0}};
static sEtcOutputDiagnostic OutputDiagnostic;

inline void out_set_state(eOutput channel, GPIO_PinState state)
{
  if(channel < OutputCount && Outputs[channel].state != state) {
    Outputs[channel].time_switched = Delay_Tick;
    Outputs[channel].state = state;
  }
}

inline GPIO_PinState out_get_state(eOutput channel, uint32_t *time)
{
  if(channel < OutputCount) {
    if(time)
      *time = DelayDiff(Delay_Tick, Outputs[channel].time_switched);
    return Outputs[channel].state;
  } else {
    return GPIO_PIN_RESET;
  }
}

void outputs_init(void)
{
  memset(&OutputDiagnostic, 0xFF, sizeof(OutputDiagnostic));
}

HAL_StatusTypeDef outputs_register(eOutput output, GPIO_TypeDef *port, uint16_t pin, uint8_t inverted, GPIO_PinState initial)
{
  if(output < OutputCount && port && pin) {
    Outputs[output].port = port;
    Outputs[output].pin = pin;
    Outputs[output].inverted = inverted;
    Outputs[output].state = initial;
    Outputs[output].time_switched = Delay_Tick;

    port->BSRR = pin << ((initial == inverted) * 16);

    return HAL_OK;
  }
  return HAL_ERROR;

}

inline void outputs_loop(void)
{
  for(int i = 0; i < OutputCount; i++) {
    if(Outputs[i].port && Outputs[i].pin) {
      if(Outputs[i].state == Outputs[i].inverted) {
        if((Outputs[i].port->ODR & Outputs[i].pin) != GPIO_PIN_RESET)
          Outputs[i].port->BSRR = Outputs[i].pin << 16;
      } else {
        if((Outputs[i].port->ODR & Outputs[i].pin) == GPIO_PIN_RESET)
          Outputs[i].port->BSRR = Outputs[i].pin;
      }
    }
  }

  OutputDiagnostic.Outs.Availability =
      Misc_Outs_GetDiagnostic(MiscOutsDiagCh1,
          &OutputDiagnostic.Outs.Diagnostic.Byte);

  OutputDiagnostic.Motor.Availability =
      Misc_Motor_GetDiagnostic(MiscMotorDiagCh1,
          &OutputDiagnostic.Motor.Diagnostic.Byte);
}

HAL_StatusTypeDef outputs_get_diagnostic(sEtcOutputDiagnostic *diagnostic)
{
  *diagnostic = OutputDiagnostic;
  return HAL_OK;
}


