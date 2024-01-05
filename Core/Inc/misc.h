/*
 * mis.h
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#ifndef INC_MISC_H_
#define INC_MISC_H_

#include "adc.h"
#include "main.h"
#include "structs.h"

typedef enum {
  MiscMotorDiagCh1 = 0,
  MiscMotorDiagChCount
}eMiscMotorDiagChannels;

typedef enum {
  MiscOutsDiagCh1 = 0,
  MiscOutsDiagChCount
}eMiscOutsDiagChannels;

void Misc_ErrorCallback(SPI_HandleTypeDef * _hspi);
void Misc_TxCpltCallback(SPI_HandleTypeDef * _hspi);
void Misc_RxCpltCallback(SPI_HandleTypeDef * _hspi);
void Misc_TxRxCpltCallback(SPI_HandleTypeDef * _hspi);
HAL_StatusTypeDef Misc_Init(SPI_HandleTypeDef * _hspi);
HAL_StatusTypeDef Misc_Outs_GetDiagnostic(eMiscOutsDiagChannels channel, uint8_t *byte);
HAL_StatusTypeDef Misc_Motor_GetDiagnostic(eMiscMotorDiagChannels channel, uint8_t *byte);
HAL_StatusTypeDef Misc_Motor_SetConfig(eMiscOutsDiagChannels channel, uint8_t byte);
HAL_StatusTypeDef Misc_Motor_GetConfig(eMiscMotorDiagChannels channel, uint8_t *byte);
HAL_StatusTypeDef Misc_Motor_StatusReset(eMiscMotorDiagChannels channel);

HAL_StatusTypeDef Misc_Motor_SetEnable(uint8_t enabled);
HAL_StatusTypeDef Misc_Motor_SetDir(uint8_t direction);
HAL_StatusTypeDef Misc_Motor_SetPwm(uint8_t pwm);

void Misc_Fast_Loop(void);
void Misc_Loop(void);


#endif /* INC_MISC_H_ */
