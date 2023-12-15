/*
 * sensors.h
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "main.h"
#include "misc.h"

typedef enum {
  SensorOilPressure = 0,
  SensorDigitalCount
}eDigitalSensor;

GPIO_PinState sens_get_digital(eDigitalSensor sensor, uint32_t *time);

void sensors_init(void);
HAL_StatusTypeDef sensors_register_digital(eDigitalSensor sensor, GPIO_TypeDef *port, uint16_t pin, uint8_t inverted);
void sensors_loop(void);

#endif /* INC_SENSORS_H_ */
