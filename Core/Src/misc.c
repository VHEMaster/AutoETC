/*
 * misc.c
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */
/*
 * Misc.c
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#include "pid.h"
#include "misc.h"
#include "adc.h"
#include "delay.h"
#include "defines.h"
#include <string.h>
#include <stdlib.h>

#define SPI_NSS_OUTS_ON() HAL_GPIO_WritePin(SPI2_NSS_OUT_GPIO_Port, SPI2_NSS_OUT_Pin, GPIO_PIN_RESET)
#define SPI_NSS_OUTS_OFF() HAL_GPIO_WritePin(SPI2_NSS_OUT_GPIO_Port, SPI2_NSS_OUT_Pin, GPIO_PIN_SET)
#define SPI_NSS_MOTOR_ON() HAL_GPIO_WritePin(SPI2_NSS_MOTOR_GPIO_Port, SPI2_NSS_MOTOR_Pin, GPIO_PIN_RESET)
#define SPI_NSS_MOTOR_OFF() HAL_GPIO_WritePin(SPI2_NSS_MOTOR_GPIO_Port, SPI2_NSS_MOTOR_Pin, GPIO_PIN_SET)

static SPI_HandleTypeDef * hspi;

static uint8_t tx[32] ALIGNED(32) BUFFER_DMA;
static uint8_t rx[32] ALIGNED(32) BUFFER_DMA;

static volatile uint8_t semTx = 0;
static volatile uint8_t semRx = 0;

static uint8_t OutputsDiagBytes[MiscOutsDiagChCount] = {0};
static uint8_t OutputsDiagnosticStored[MiscOutsDiagChCount] = {0};
static HAL_StatusTypeDef OutputsAvailability[MiscOutsDiagChCount] = {0};

static uint8_t MotorDiagBytes[MiscMotorDiagChCount] = {0};
static uint8_t MotorDiagnosticStored[MiscMotorDiagChCount] = {0};
static HAL_StatusTypeDef MotorAvailability[MiscMotorDiagChCount] = {0};

volatile uint8_t nss_motor_off = 0;
volatile uint8_t nss_outs_off = 0;

static volatile uint32_t *gEtcPwmPeriod;
static volatile uint32_t *gEtcPwmPulse;

STATIC_INLINE void Misc_CpltNssCheck(void)
{
  if(nss_motor_off) {
    SPI_NSS_MOTOR_OFF();
    nss_motor_off = 0;
  }
  if(nss_outs_off) {
    SPI_NSS_OUTS_OFF();
    nss_outs_off = 0;
  }
}

void Misc_ErrorCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    Misc_CpltNssCheck();
  }
}

void Misc_TxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    Misc_CpltNssCheck();
    semTx = 1;
  }
}

void Misc_RxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    Misc_CpltNssCheck();
    semRx = 1;
  }
}

void Misc_TxRxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    Misc_CpltNssCheck();
    semTx = 1;
    semRx = 1;
  }
}

static uint8_t waitTxRxCplt(void)
{
  if(semRx && semTx) {
    semRx = 0;
    semTx = 0;
    return 1;
  }
  return 0;
}

static void Motor_CriticalLoop(void)
{

}


static int8_t Motor_Loop(void)
{
  static uint8_t state = 0;
  static eMiscMotorDiagChannels channel = 0;
  GPIO_PinState pin;

  do {
    switch(state) {
      case 0:
        SPI_NSS_MOTOR_OFF();

        switch(channel) {
          case MiscMotorDiagCh1: SPI_NSS_MOTOR_ON(); nss_motor_off = 1; break;
          default: channel = 0; continue;
        }

        tx[0] = 0x00;
        HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 1);
        state++;
        break;
      case 1:
        if(waitTxRxCplt()) {
          //SPI_NSS_MOTOR_OFF();

          rx[0] = __RBIT(rx[0]) >> 24;

          if(MotorDiagnosticStored[channel] == 0) {
            MotorDiagnosticStored[channel] = 1;
            MotorDiagBytes[channel] = rx[0];
          } else {
            MotorDiagBytes[channel] |= rx[0];
          }

          if(rx[0] == 0xFF || rx[0] == 0x00)
            MotorAvailability[channel] = HAL_ERROR;
          else MotorAvailability[channel] = HAL_OK;

          state = 0;
          if(++channel >= MiscMotorDiagChCount) {
            channel = 0;
            return 1;
          }
          else continue;
        }
        break;
      default:
        state = 0;
        break;
    }
  } while(0);

  return 0;
}

static int8_t Outs_Loop(void)
{
  static uint8_t state = 0;
  static eMiscOutsDiagChannels channel = 0;
  static uint8_t failure_stored = 0;
  GPIO_PinState pin;

  do {
    switch(state) {
      case 0:
        SPI_NSS_OUTS_OFF();

        switch(channel) {
          case MiscOutsDiagCh1: SPI_NSS_OUTS_ON(); nss_outs_off = 1; break;
          default: channel = 0; continue;
        }
        state++;
        break;
      case 1:
        pin = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14); //SPI2_MISO
        failure_stored = pin != GPIO_PIN_RESET;
        tx[0] = 0xFF;
        HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 1);
        state++;
        break;
      case 2:
        if(waitTxRxCplt()) {
          //SPI_NSS_OUTS_OFF();

          if(OutputsDiagnosticStored[channel] == 0 || failure_stored == 1 || rx[0] != 0xFF) {
            OutputsDiagnosticStored[channel] = 1;
            OutputsDiagBytes[channel] = rx[0];
            if((failure_stored == 1 && rx[0] == 0xFF) || (failure_stored == 0 && rx[0] != 0xFF))
              OutputsAvailability[channel] = HAL_ERROR;
            else OutputsAvailability[channel] = HAL_OK;
          }

          state = 0;
          if(++channel >= MiscOutsDiagChCount) {
            channel = 0;
            return 1;
          }
          else continue;
        }
        break;
      default:
        state = 0;
        break;
    }
  } while(0);

  return 0;
}

void Misc_Fast_Loop(void)
{

}

void Misc_Loop(void)
{
  static uint8_t work_motor = 0;
  static uint8_t work_outs = 0;

  static uint32_t lastMotorExec = 0;
  static uint32_t lastOutsExec = 0;

  uint32_t now = Delay_Tick;

  if(!work_motor && !work_outs) {
    if(DelayDiff(now, lastMotorExec) >= 10000) {
      lastMotorExec = now;
      work_motor = 1;
    }
    if(DelayDiff(now, lastOutsExec) >= 100000) {
      lastOutsExec = now;
      work_outs = 1;
    }
  }

  Motor_CriticalLoop();

  if(work_motor) {
    if(Motor_Loop())
      work_motor = 0;
  }
  else if(work_outs) {
    if(Outs_Loop())
      work_outs = 0;
  }
}

HAL_StatusTypeDef Misc_Init(SPI_HandleTypeDef * _hspi)
{
  HAL_StatusTypeDef result = HAL_OK;

  SPI_NSS_OUTS_OFF();
  SPI_NSS_MOTOR_OFF();

  DelayMs(5);

  memset(tx, 0, sizeof(tx));
  memset(rx, 0, sizeof(tx));

  hspi = _hspi;

  HAL_GPIO_WritePin(SPI2_NRST_GPIO_Port, SPI2_NRST_Pin, GPIO_PIN_SET);

  gEtcPwmPeriod = &TIM3->ARR;
  gEtcPwmPulse = &TIM3->CCR2;

  for(int i = 0; i < MiscOutsDiagChCount; i++)
    OutputsDiagBytes[i] = 0xFF;
  for(int i = 0; i < MiscMotorDiagChCount; i++)
    MotorDiagBytes[i] = 0xFF;

  return result;
}

HAL_StatusTypeDef Misc_Outs_GetDiagnostic(eMiscOutsDiagChannels channel, uint8_t *byte)
{
  HAL_StatusTypeDef result = OutputsAvailability[channel];
  *byte = OutputsDiagBytes[channel] ^ 0xFF;
  OutputsDiagnosticStored[channel] = 0;
  return result;
}

HAL_StatusTypeDef Misc_Motor_GetDiagnostic(eMiscMotorDiagChannels channel, uint8_t *byte)
{
  HAL_StatusTypeDef result = MotorAvailability[channel];
  *byte = MotorDiagBytes[channel];
  MotorDiagnosticStored[channel] = 0;
  return result;
}

HAL_StatusTypeDef Misc_Motor_SetEnable(uint8_t enabled)
{
  HAL_StatusTypeDef result = HAL_OK;

  HAL_GPIO_WritePin(MOTOR_DIS_GPIO_Port, MOTOR_DIS_Pin, enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);

  return result;
}

HAL_StatusTypeDef Misc_Motor_SetDir(uint8_t direction)
{
  HAL_StatusTypeDef result = HAL_OK;

  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);

  return result;
}

HAL_StatusTypeDef Misc_Motor_SetPwm(uint8_t pwm)
{
  HAL_StatusTypeDef result = HAL_OK;

  if (gEtcPwmPeriod && gEtcPwmPulse) {
    *gEtcPwmPulse = pwm;
  }

  return result;
}
