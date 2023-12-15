/*
 * sensors.c
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#include "sensors.h"
#include "misc.h"
#include "adc.h"
#include "defines.h"
#include "delay.h"

typedef struct {
  GPIO_TypeDef *port;
  uint32_t time_switched;
  volatile GPIO_PinState state;
  uint16_t pin;
  uint8_t inverted;
}sDigitalSensor;

static sDigitalSensor Sensors[SensorDigitalCount] = {{0}};

void sensors_init(void)
{

}

HAL_StatusTypeDef sensors_register_digital(eDigitalSensor sensor, GPIO_TypeDef *port, uint16_t pin, uint8_t inverted)
{
  if(sensor < SensorDigitalCount && port && pin) {
    Sensors[sensor].port = port;
    Sensors[sensor].pin = pin;
    Sensors[sensor].inverted = inverted;

    Sensors[sensor].state = ((port->IDR & pin) != GPIO_PIN_RESET) ? !inverted : inverted;
    return HAL_OK;
  }
  return HAL_ERROR;

}

inline void sensors_loop(void)
{
  GPIO_PinState pin_state;
  for(int i = 0; i < SensorDigitalCount; i++) {
    if(Sensors[i].port && Sensors[i].pin) {
      pin_state = ((Sensors[i].port->IDR & Sensors[i].pin) != GPIO_PIN_RESET) ? (!Sensors[i].inverted) : (Sensors[i].inverted);
      if(Sensors[i].state != pin_state) {
        Sensors[i].time_switched = Delay_Tick;
        Sensors[i].state = pin_state;
      }
    }
  }
}

inline GPIO_PinState sens_get_digital(eDigitalSensor sensor, uint32_t *time)
{
  if(sensor < SensorDigitalCount) {
    if(time)
      *time = DelayDiff(Delay_Tick, Sensors[sensor].time_switched);
    return Sensors[sensor].state;
  }

  return GPIO_PIN_RESET;
}

