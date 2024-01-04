/*
 * adc.h
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"

typedef enum {
  AdcChTps1 = 0,
  AdcChTps2,
  AdcChPedal1,
  AdcChPedal2,
  AdcChRsvd5,
  AdcChRsvd6,
  AdcChPowerVoltage,
  AdcChRefVoltage,
}eAdcChannel;

#define ADC_RANGE_0P2500 0x05
#define ADC_RANGE_0P1250 0x06

#define ADC_FILTER_DISABLE 0
#define ADC_FILTER_ENABLE  1

typedef int8_t (*AdcChannelEvent)(eAdcChannel channel);

void ADC_ErrorCallback(SPI_HandleTypeDef * _hspi);
void ADC_TxCpltCallback(SPI_HandleTypeDef * _hspi);
void ADC_RxCpltCallback(SPI_HandleTypeDef * _hspi);
void ADC_TxRxCpltCallback(SPI_HandleTypeDef * _hspi);
HAL_StatusTypeDef adc_init(SPI_HandleTypeDef * _hspi);
HAL_StatusTypeDef adc_register(eAdcChannel channel, uint8_t range, uint16_t divider, uint8_t filter);
HAL_StatusTypeDef adc_fast_loop(void);
HAL_StatusTypeDef adc_slow_loop(void);
uint16_t adc_get_voltage(eAdcChannel channel);
HAL_StatusTypeDef adc_get_status(void);
void adc_set_events(AdcChannelEvent startEvent, AdcChannelEvent doneEvent);

#endif /* INC_ADC_H_ */
