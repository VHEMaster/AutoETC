/*
 * adc.c
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#include "adc.h"
#include "delay.h"
#include "defines.h"
#include <string.h>

#define SPI_NSS_ON() HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET)
#define SPI_NSS_OFF() HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET)

#define ADC_VREF          4096

#define ADC_CHANNELS      8
#define MCU_CHANNELS      0
#define ADC_BUFFER_SIZE   12

#define ADC_INIT_TIME     (15000)
#define ADC_TIMEOUT       (5000)

static uint16_t AdcBuffer[ADC_CHANNELS][ADC_BUFFER_SIZE] = {{0}};
static uint16_t AdcDataLast[ADC_CHANNELS] = {0};
static uint16_t AdcVoltages[ADC_CHANNELS];
static volatile HAL_StatusTypeDef adcInitStatus = HAL_OK;
static volatile HAL_StatusTypeDef adcTimeoutStatus = HAL_OK;
static volatile HAL_StatusTypeDef adcStatus = HAL_OK;
static volatile uint32_t adcInitTime = 0;
static volatile uint8_t adcInited = 0;

static uint16_t ChData[ADC_CHANNELS] = {0};
static uint8_t ChRange[ADC_CHANNELS] = {0};
static uint8_t ChFilter[ADC_CHANNELS] = {0};
static uint8_t ChIgnoreNext[ADC_CHANNELS] = {0};
static uint16_t ChDivider[ADC_CHANNELS] = {0};

#ifdef DEBUG
static volatile uint32_t ChPeriod[ADC_CHANNELS] = {0};
static uint32_t ChLast[ADC_CHANNELS] = {0};
#endif /* DEBUG */

static SPI_HandleTypeDef *hspi = NULL;

static uint8_t tx[32] ALIGNED(32);
static uint8_t rx[32] ALIGNED(32);

static volatile uint8_t semEr = 0;
static volatile uint8_t semTx = 0;
static volatile uint8_t semRx = 0;
static uint32_t AdcLastComm = 0;
static uint32_t AdcWritePos = 0;
static uint8_t AdcChannel = 0;
static uint8_t AdcReset = 0;
static uint8_t AdcFirstTime = 1;

static AdcChannelEvent AdcStartSamplingEvent = NULL;
static AdcChannelEvent AdcSamplingDoneEvent = NULL;

static uint32_t AdcOldSamples[ADC_CHANNELS];
static uint32_t AdcOldTimestamps[ADC_CHANNELS];

void adc_set_events(AdcChannelEvent startEvent, AdcChannelEvent doneEvent)
{
  AdcStartSamplingEvent = startEvent;
  AdcSamplingDoneEvent = doneEvent;
}

void ADC_ErrorCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    semEr = 1;
  }
}

void ADC_TxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    SPI_NSS_OFF();
    semTx = 1;
  }
}

void ADC_RxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    SPI_NSS_OFF();
    semRx = 1;
  }
}

void ADC_TxRxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    SPI_NSS_OFF();
    semTx = 1;
    semRx = 1;
  }
}

static int8_t waitTxRxCplt(void)
{
  int8_t status = 0;

  if(semEr) {
    status = -1;
  } else if(semRx && semTx) {
    status = 1;
  }

  if(status) {
    semEr = 0;
    semRx = 0;
    semTx = 0;
  }

  return status;
}

static HAL_StatusTypeDef SPI_SendCommand(uint16_t cmd)
{
  int8_t status;

  tx[0] = cmd >> 8;
  tx[1] = cmd & 0xFF;
  SPI_NSS_ON();
  HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 2);
  do {
    status = waitTxRxCplt();
  } while(!status);
  //SPI_NSS_OFF();

  //if(rx[0] != 0x00 || rx[1] != 0x00)
  //  return HAL_ERROR;

  return status > 0 ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef SPI_WriteRegister(uint8_t reg, uint8_t data)
{
  int8_t status;

  tx[0] = reg << 1 | 1;
  tx[1] = data;
  tx[2] = 0;
  SPI_NSS_ON();
  HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 3);
  do {
    status = waitTxRxCplt();
  } while(!status);
  //SPI_NSS_OFF();

  if(status > 0) {
    if(rx[0] != 0x00 || rx[1] != 0x00)
      status = -2;
    if(rx[2] != data)
      status = -3;
  }

  return status > 0 ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef SPI_ReadRegister(uint8_t reg, uint8_t *data)
{
  int8_t status;

  *data = 0;

  tx[0] = reg << 1;
  tx[1] = 0;
  tx[2] = 0;
  SPI_NSS_ON();
  HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 3);
  do {
    status = waitTxRxCplt();
  } while(!status);
  //SPI_NSS_OFF();

  if(status > 0) {
    if(rx[0] != 0x00 || rx[1] != 0x00)
      status = -2;
  }

  *data = rx[2];

  return status > 0 ? HAL_OK : HAL_ERROR;
}

static INLINE uint16_t ADC_Convert(uint8_t channel, uint16_t data)
{
  switch(ChRange[channel]) {
    case ADC_RANGE_0P2500 :
      return data * 15259 / 10000 * ADC_VREF / 1000 * 2500 / 100000;
    case ADC_RANGE_0P1250 :
      return data * 15259 / 10000 * ADC_VREF / 1000 * 1250 / 100000;
    default:
      return 0;
  }
}

HAL_StatusTypeDef adc_init(SPI_HandleTypeDef * _hspi)
{
  HAL_StatusTypeDef result = HAL_OK;
  uint8_t data;

  for(int i = 0; i < ADC_CHANNELS; i++) {
    for(int j = 0; j < ADC_BUFFER_SIZE; j++) {
      AdcBuffer[i][j] = 0x8000;
    }
  }

  memset(AdcOldTimestamps, 0, sizeof(AdcOldTimestamps));
  memset(AdcOldSamples, 0, sizeof(AdcOldSamples));

  DelayMs(10);

  HAL_GPIO_WritePin(SPI1_NRST_GPIO_Port, SPI1_NRST_Pin, GPIO_PIN_SET);

  DelayMs(1);

  hspi = _hspi;

  //RST command
  result = SPI_SendCommand(0x8500);
  if(result != HAL_OK)
    goto ret;

  //Enable all channels
  result = SPI_WriteRegister(0x02, 0x00);
  if(result != HAL_OK)
    goto ret;

  //Check all channels enabled
  result = SPI_ReadRegister(0x02, &data);
  if(result != HAL_OK)
    goto ret;

  if(data != 0x00) {
    goto ret;
    result = HAL_ERROR;
  }

  //All channels in sequence mode
  result = SPI_WriteRegister(0x01, 0xFF);
  if(result != HAL_OK)
    goto ret;

  //Check all channels in sequence mode
  result = SPI_ReadRegister(0x01, &data);
  if(result != HAL_OK)
    goto ret;

  if(data != 0xFF) {
    result = HAL_ERROR;
    goto ret;
  }

  //SDO format: 011, dev address: 0
  result = SPI_WriteRegister(0x03, 0x03);
  if(result != HAL_OK)
    goto ret;

  //Check SDO format: 011, dev address: 0
  result = SPI_ReadRegister(0x03, &data);
  if(result != HAL_OK)
    goto ret;

  if(data != 0x03) {
    result = HAL_ERROR;
    goto ret;
  }

  for(int i = 0; i < ADC_CHANNELS; i++) {
    result = SPI_WriteRegister(0x05 + i, ChRange[i]);
    if(result != HAL_OK)
      goto ret;

    //Check channel setup
    result = SPI_ReadRegister(0x05 + i, &data);
    if(result != HAL_OK)
      goto ret;

    if(data != ChRange[i]) {
      result = HAL_ERROR;
      goto ret;
    }
  }

  result = SPI_SendCommand(0xA000); //AUTO_RST command
  if(result != HAL_OK)
    goto ret;

  adcInited = 0;
  adcInitTime = Delay_Tick;
  AdcLastComm = Delay_Tick;

ret:
  adcInitStatus = result;
  adcStatus = result;

  if(AdcStartSamplingEvent)
    if(AdcStartSamplingEvent(ADC_CHANNELS) < 0)
      ChIgnoreNext[AdcChannel] = 1;

  return result;
}

HAL_StatusTypeDef adc_register(eAdcChannel channel, uint8_t range, uint16_t divider, uint8_t filter)
{
  HAL_StatusTypeDef result = HAL_OK;

  if(channel >= ADC_CHANNELS)
    return HAL_ERROR;

  ChRange[channel] = range;
  ChDivider[channel] = divider;
  ChFilter[channel] = filter;

  return result;
}

ITCM_FUNC HAL_StatusTypeDef adc_fast_loop(void)
{
  static HAL_StatusTypeDef result = HAL_OK;
  static uint8_t state = 0;
  HAL_StatusTypeDef hal_status;
  int8_t status;
  uint32_t now = Delay_Tick;
  uint16_t data;

  if(adcInitStatus != HAL_OK)
    return adcInitStatus;

  if(!adcInited && DelayDiff(now, adcInitTime) >= ADC_INIT_TIME) {
    adcInited = 1;
    AdcLastComm = now;
  }

  if(adcInited) {

    switch(state) {
      case 0:
        if(AdcStartSamplingEvent)
          if(AdcStartSamplingEvent(AdcChannel) < 0)
            ChIgnoreNext[AdcChannel] = 1;
        SPI_NSS_ON();
        memset(tx, 0, 4);
        hal_status = HAL_SPI_TransmitReceive_DMA(hspi, tx, rx, 4);
        if(hal_status != HAL_OK) {
          if(IS_DEBUGGER_ATTACHED()) {
            BREAKPOINT(0);
          }
        } else {
          state++;
        }
        break;
      case 1:
        status = waitTxRxCplt();
        if(status) {
          if(status > 0) {
            //SPI_NSS_OFF();
            data = rx[2] << 8 | rx[3];

            if(AdcReset) {
              AdcReset = 0;
              AdcChannel = 0;
            } else {
              if(!ChIgnoreNext[AdcChannel]) {
                if(ChFilter[AdcChannel]) {
                  if(!AdcFirstTime)
                    AdcBuffer[AdcChannel][AdcWritePos] = data;
                  else {
                    AdcWritePos = 0;
                    for(int i = 0; i < ADC_BUFFER_SIZE; i++) {
                      AdcBuffer[AdcChannel][i] = data;
                    }
                  }
                } else {
                  AdcBuffer[AdcChannel][0] = data;
                }
                AdcDataLast[AdcChannel] = data;
#ifdef DEBUG
                ChPeriod[AdcChannel] = DelayDiff(now, ChLast[AdcChannel]);
                ChLast[AdcChannel] = now;
#endif /* DEBUG */
              } else {
                ChIgnoreNext[AdcChannel] = 0;
              }

              if(AdcSamplingDoneEvent)
                if(AdcSamplingDoneEvent(AdcChannel) < 0)
                  ChIgnoreNext[AdcChannel] = 1;

              if(++AdcChannel >= ADC_CHANNELS) {
                if(++AdcWritePos >= ADC_BUFFER_SIZE) {
                  AdcWritePos = 0;
                }
                AdcChannel = 0;
                AdcFirstTime = 0;
              }
            }
            AdcLastComm = now;
          } else {
            AdcReset = 1;
            if(IS_DEBUGGER_ATTACHED()) {
              BREAKPOINT(0);
            }
          }

          SPI_NSS_ON();
          memset(tx, 0, 4);
          if(AdcReset)
            tx[0] = 0xA0;
          hal_status = HAL_SPI_TransmitReceive_DMA(hspi, tx, rx, 4);
          if(hal_status != HAL_OK) {
            state = 0;
            if(IS_DEBUGGER_ATTACHED()) {
              BREAKPOINT(0);
            }
          }
        } else {
          if(DelayDiff(now, AdcLastComm) > ADC_TIMEOUT) {
            adcTimeoutStatus = HAL_ERROR;
            if(IS_DEBUGGER_ATTACHED()) {
              BREAKPOINT(0);
            }
          } else {
            adcTimeoutStatus = HAL_OK;
          }
        }
        break;

      default:
        state = 0;
        break;
    }
  }

  return result;
}

HAL_StatusTypeDef adc_slow_loop(void)
{
  const uint32_t failed_adc_mask = (1 << ADC_CHANNELS) - 1;
  static HAL_StatusTypeDef result = HAL_OK;
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t data;
  uint32_t failed_channels = 0;
  uint32_t now = Delay_Tick;

  uint16_t new_val;


  if(adcInitStatus != HAL_OK)
    return adcInitStatus;

  for(int i = 0; i < ADC_CHANNELS; i++) {
    if(ChFilter[i]) {
      data = 0;
      for(int j = 0; j < ADC_BUFFER_SIZE; j++)
        data += AdcBuffer[i][j];
      ChData[i] = data / ADC_BUFFER_SIZE;
    } else {
      ChData[i] = AdcBuffer[i][0];
    }
    new_val = ADC_Convert(i, ChData[i]) * ChDivider[i] / 1000;
    AdcVoltages[i] = new_val;

  }

  for(int i = 0; i < ADC_CHANNELS; i++) {
    if(ChData[i] == AdcOldSamples[i] || ChData[i] == 0x0000 || ChData[i] == 0xFFFF) {
      if(DelayDiff(now, AdcOldTimestamps[i]) > 1000000) {
        failed_channels |= 1 << i;
      }
    } else {
      AdcOldTimestamps[i] = now;
    }
    AdcOldSamples[i] = ChData[i];
  }

  if((failed_channels & failed_adc_mask) == failed_adc_mask)
    status |= HAL_ERROR;

  adcStatus = status;

  return result;
}

INLINE uint16_t adc_get_voltage(eAdcChannel channel)
{
  if(channel < ADC_CHANNELS)
    return AdcVoltages[channel];
  return 0;
}

HAL_StatusTypeDef adc_get_status(void)
{
  return adcStatus | adcTimeoutStatus;
}

