/*
 * etc.c
 *
 *  Created on: Dec 15, 2023
 *      Author: VHEMaster
 */

#include "main.h"
#include "adc.h"
#include "structs.h"
#include "misc.h"
#include "delay.h"
#include "pid.h"
#include "outputs.h"
#include "sensors.h"
#include "can.h"
#include "can_signals.h"
#include "can_signals_db.h"
#include <string.h>
#include <stdlib.h>

#define MOTOR_STARTUP_TIME (500 * 1000)

typedef struct {
    uint32_t error_time;
    uint32_t error_last;
    uint32_t confirm_time;
    HAL_StatusTypeDef status;
    uint8_t is_error;
}sErrorCtx;

typedef struct {
    sEtcOutputDiagnostic Outputs;
    struct {
      sErrorCtx Tps1;
      sErrorCtx Tps2;
      sErrorCtx Pedal1;
      sErrorCtx Pedal2;
      sErrorCtx TpsMismatch;
      sErrorCtx PedalMismatch;
    }Sensors;
    HAL_StatusTypeDef CanInitStatus : 2;
    HAL_StatusTypeDef CanTestStatus : 2;
    HAL_StatusTypeDef AdcStatus : 2;
}sEtcStatus;

static sEtcConfig gEtcConfig;
static const sEtcConfig gEtcConfigDefault = {
    .Tps1Min = 500,
    .Tps1Max = 4256,
    .Tps1Limit = 4649,
    .Tps2Min = 4481,
    .Tps2Max = 723,
    .Tps2Limit = 324,
    .Pedal1Min = 860,
    .Pedal1Max = 4380,
    .Pedal2Min = 450,
    .Pedal2Max = 2222,

    .PidP = 2400,
    .PidI = 3800,
    .PidD = 30,

    .TimPsc = 18
};

static uint8_t gCanTestStarted = 0;
static sEtcParametersInt gEtcParameters;
static sEtcStatus gEtcStatus;
static sMathPid gThrottlePid;
static uint32_t gCommLast = 0;

static HAL_StatusTypeDef etc_error_ctx_handle(sErrorCtx *ctx, uint8_t is_error)
{
  uint32_t now = Delay_Tick;

  if (is_error != ctx->is_error) {
    if(ctx->error_time < ctx->confirm_time) {
      ctx->error_time += DelayDiff(now, ctx->error_last);
    } else {
      ctx->is_error = is_error;
      ctx->error_time = 0;
    }
  } else {
    ctx->error_time = 0;
  }

  ctx->error_last = now;
  ctx->status = ctx->is_error ? HAL_ERROR : HAL_OK;

  return ctx->status;
}

static void etc_throttle_loop(void)
{
  static uint32_t startup_time = 0;
  static uint32_t last = 0;
  static uint32_t pos_filtered = 0;
  uint32_t now = Delay_Tick;
  int16_t pedal1_v, pedal2_v;
  uint16_t pedal1_v_diff, pedal2_v_diff;

  gEtcParameters.AdcTps1 = adc_get_voltage(AdcChTps1);
  gEtcParameters.AdcTps2 = adc_get_voltage(AdcChTps2);
  gEtcParameters.AdcPedal1 = adc_get_voltage(AdcChPedal1);
  gEtcParameters.AdcPedal2 = adc_get_voltage(AdcChPedal2);
  gEtcParameters.AdcRsvd5 = adc_get_voltage(AdcChRsvd5);
  gEtcParameters.AdcRsvd6 = adc_get_voltage(AdcChRsvd6);
  gEtcParameters.AdcReferenceVoltage = adc_get_voltage(AdcChRefVoltage);
  gEtcParameters.AdcPowerVoltage = adc_get_voltage(AdcChPowerVoltage);

  pedal1_v = CLAMP(gEtcParameters.AdcPedal1, gEtcConfig.Pedal1Min, gEtcConfig.Pedal1Max);
  pedal2_v = CLAMP(gEtcParameters.AdcPedal2, gEtcConfig.Pedal2Min, gEtcConfig.Pedal2Max);
  pedal1_v_diff = gEtcConfig.Pedal1Max - gEtcConfig.Pedal1Min;
  pedal2_v_diff = gEtcConfig.Pedal2Max - gEtcConfig.Pedal2Min;

  gEtcParameters.PedalError = HAL_OK;
  gEtcParameters.PedalError |= etc_error_ctx_handle(&gEtcStatus.Sensors.Pedal1, abs(gEtcParameters.AdcPedal1 - pedal1_v) > 200);
  gEtcParameters.PedalError |= etc_error_ctx_handle(&gEtcStatus.Sensors.Pedal2, abs(gEtcParameters.AdcPedal2 - pedal2_v) > 200);

  pedal1_v -= gEtcConfig.Pedal1Min;
  pedal2_v -= gEtcConfig.Pedal2Min;

  gEtcParameters.Pedal1 = pedal1_v * 8191 / pedal1_v_diff;
  gEtcParameters.Pedal2 = pedal2_v * 8191 / pedal2_v_diff;

  if(gEtcParameters.PedalError == HAL_OK) {
    gEtcParameters.PedalPosition = (gEtcParameters.Pedal1 + gEtcParameters.Pedal2) / 2;
  } else {
    gEtcParameters.PedalPosition = 0;
  }

  gEtcParameters.PedalError |= etc_error_ctx_handle(&gEtcStatus.Sensors.PedalMismatch, abs(gEtcParameters.Pedal1 - gEtcParameters.Pedal2) > 100);


  int16_t tps1_v, tps2_v;
  int16_t tps1_v_diff, tps2_v_diff;

  tps1_v = CLAMP(gEtcParameters.AdcTps1, gEtcConfig.Tps1Min, gEtcConfig.Tps1Limit);
  tps2_v = CLAMP(gEtcParameters.AdcTps2, gEtcConfig.Tps2Limit, gEtcConfig.Tps2Min);
  tps1_v_diff = gEtcConfig.Tps1Max - gEtcConfig.Tps1Min;
  tps2_v_diff = gEtcConfig.Tps2Min - gEtcConfig.Tps2Max;

  gEtcParameters.TpsError = HAL_OK;
  gEtcParameters.TpsError |= etc_error_ctx_handle(&gEtcStatus.Sensors.Tps1, abs(gEtcParameters.AdcTps1 - tps1_v) > 200);
  gEtcParameters.TpsError |= etc_error_ctx_handle(&gEtcStatus.Sensors.Tps2, abs(gEtcParameters.AdcTps2 - tps2_v) > 200);

  tps1_v -= gEtcConfig.Tps1Min;
  tps2_v -= gEtcConfig.Tps2Max;

  gEtcParameters.Tps1 = tps1_v * 8191 / tps1_v_diff;
  gEtcParameters.Tps2 = tps2_v * 8191 / tps2_v_diff;

  gEtcParameters.Tps2 = 8191 - gEtcParameters.Tps2;

  if(gEtcParameters.TpsError == HAL_OK) {
    gEtcParameters.ThrottlePosition = (gEtcParameters.Tps1 + gEtcParameters.Tps2) / 2;
  } else {
    gEtcParameters.ThrottlePosition = 0;
  }

  gEtcParameters.TpsError |= etc_error_ctx_handle(&gEtcStatus.Sensors.TpsMismatch, abs(gEtcParameters.Tps1 - gEtcParameters.Tps2) > 100);

  gEtcParameters.MotorError = outputs_get_diagnostic(&gEtcStatus.Outputs);
  if(gEtcStatus.Outputs.Motor.Availability != HAL_OK) {
    gEtcParameters.MotorError = HAL_ERROR;
  } else if(gEtcStatus.Outputs.Motor.Diagnostic.Data.ErrorFlag) {
    gEtcParameters.MotorError = HAL_ERROR;
  }
  gEtcParameters.MotorDiagByte = gEtcStatus.Outputs.Motor.Diagnostic.Byte;
  gEtcParameters.OutsDiagByte = gEtcStatus.Outputs.Outs.Diagnostic.Byte;

  uint8_t pwm;
  int16_t pid;

  if(last == 0) {
    last = now;
  }

  if(gEtcParameters.DefaultPosition == 0) {
    startup_time += DelayDiff(now, last);
    if (startup_time >= MOTOR_STARTUP_TIME) {
      gEtcParameters.DefaultPosition = gEtcParameters.ThrottlePosition;
      gEtcParameters.TargetPosition = gEtcParameters.DefaultPosition;
      pos_filtered = gEtcParameters.TargetPosition;
    }
  }

  if(gEtcParameters.CommError != HAL_OK) {
    gEtcParameters.StandaloneMode = 1;
    gEtcParameters.MotorActive = 1;
    gEtcParameters.FullCloseRequest = 0;
    gEtcParameters.AdaptationProcess = 0;
  }

  if(gEtcParameters.CommError != HAL_OK || gEtcParameters.StandaloneMode) {
    gEtcParameters.TargetPosition = gEtcParameters.DefaultPosition;
    if(gEtcParameters.DefaultPosition + gEtcParameters.PedalPosition < 8191) {
      gEtcParameters.TargetPosition = gEtcParameters.DefaultPosition + gEtcParameters.PedalPosition;
    }
  }

  gEtcParameters.TargetPosition = CLAMP(gEtcParameters.TargetPosition, 0, 8191);

  pos_filtered = (gEtcParameters.TargetPosition * 1000 + pos_filtered * 9000) / 10000;

  if(gEtcParameters.MotorActive && gEtcParameters.TpsError == HAL_OK && (gEtcParameters.PedalError == HAL_OK || !gEtcParameters.StandaloneMode) && gEtcParameters.DefaultPosition != 0) {
    math_pid_set_koffs(&gThrottlePid, gEtcConfig.PidP * 0.0001f, gEtcConfig.PidI * 0.0001f, gEtcConfig.PidD * 0.0001f);
    math_pid_set_target(&gThrottlePid, pos_filtered);

    if(gEtcParameters.FullCloseRequest) {
      pid = -100;
    } else {
      pid = math_pid_update(&gThrottlePid, gEtcParameters.ThrottlePosition, now);

      if(gEtcParameters.ThrottlePosition <= 0) {
        math_pid_set_clamp(&gThrottlePid, -60.0f, 255.0f);
      } else if(gEtcParameters.ThrottlePosition >= 8191) {
        math_pid_set_clamp(&gThrottlePid, -160.0f, 60.0f);
      } else {
        math_pid_set_clamp(&gThrottlePid, -160.0f, 255.0f);
      }
    }

    if(pid >= 0) {
      Misc_Motor_SetDir(0);
      pwm = pid;
    } else {
      Misc_Motor_SetDir(1);
      pwm = -pid;
    }

    Misc_Motor_SetPwm(pwm);
    Misc_Motor_SetEnable(1);

    TIM3->PSC = gEtcConfig.TimPsc;
  } else {
    Misc_Motor_SetEnable(0);
    Misc_Motor_SetDir(0);
    Misc_Motor_SetPwm(0);
    math_pid_reset(&gThrottlePid, now);
  }
}

void etc_irq_slow_loop(void)
{
  etc_throttle_loop();
}

void etc_irq_fast_loop(void)
{

}

static void ecu_can_init(void)
{
  gEtcStatus.CanInitStatus = can_start(0x080, 0x7F0);
  if(gEtcStatus.CanInitStatus == HAL_OK) {
    gCanTestStarted = 1;
  }

  can_message_register_msg(&g_can_message_id080_ECU_ETC);
  can_message_register_msg(&g_can_message_id088_ECU_ETC);
  can_message_register_msg(&g_can_message_id089_ECU_ETC);
  can_message_register_msg(&g_can_message_id08A_ECU_ETC);
  can_message_register_msg(&g_can_message_id08B_ECU_ETC);
  can_message_register_msg(&g_can_message_id08C_ECU_ETC);
}

static void etc_can_process_message(const sCanRawMessage *message)
{
  sCanMessage * can_msg = can_message_get_msg(message);
  uint32_t now = Delay_Tick;
  uint32_t value = 0;

  if(can_msg != NULL) {
    switch(can_msg->Id) {
      case 0x080:
        gEtcParameters.CommError = HAL_OK;
        gCommLast = now;

        can_signal_get_raw(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_StandaloneMode, &value);
        gEtcParameters.StandaloneMode = value;

        can_signal_get_raw(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_MotorActive, &value);
        gEtcParameters.MotorActive = value;

        can_signal_get_raw(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_TargetPosition, &value);
        gEtcParameters.TargetPosition = value;

        can_signal_get_raw(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_MotorErrorReset, &value);
        if(value) Misc_Motor_StatusReset(MiscMotorDiagCh1);

        can_signal_get_raw(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_MotorFullCloseRequest, &value);
        gEtcParameters.FullCloseRequest = value;

        can_signal_get_raw(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_AdaptationRequest, &value);
        gEtcParameters.AdaptationProcess = value;

        //can_signal_get_raw(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_Rsvd1, &value);

        //can_signal_get_raw(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_Rsvd2, &value);

        can_signal_get_raw(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_OutCruizeR, &value);
        out_set_state(OutCruizeR, value ? GPIO_PIN_SET : GPIO_PIN_RESET);

        can_signal_get_raw(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_OutCruizeG, &value);
        out_set_state(OutCruizeG, value ? GPIO_PIN_SET : GPIO_PIN_RESET);

        can_signal_get_raw(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_OutRsvd3, &value);
        out_set_state(OutRsvd3, value ? GPIO_PIN_SET : GPIO_PIN_RESET);

        can_signal_get_raw(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_OutRsvd4, &value);
        out_set_state(OutRsvd4, value ? GPIO_PIN_SET : GPIO_PIN_RESET);

        break;
      case 0x088:
        can_signal_get_raw(&g_can_message_id088_ECU_ETC, &g_can_signal_id088_ECU_ETC_ConfigIdRequest, &value);
        switch(value) {
          case 0x019:
            can_signal_message_clear(&g_can_message_id019_ETC_ECU);
            can_signal_append_raw(&g_can_message_id019_ETC_ECU, &g_can_signal_id019_ETC_ECU_Tps1Min, gEtcConfig.Tps1Min * 10000 / 14652);
            can_signal_append_raw(&g_can_message_id019_ETC_ECU, &g_can_signal_id019_ETC_ECU_Tps1Max, gEtcConfig.Tps1Max * 10000 / 14652);
            can_signal_append_raw(&g_can_message_id019_ETC_ECU, &g_can_signal_id019_ETC_ECU_Tps1Limit, gEtcConfig.Tps1Limit * 10000 / 14652);
            can_signal_append_raw(&g_can_message_id019_ETC_ECU, &g_can_signal_id019_ETC_ECU_Rsvd, 0);
            can_message_send(&g_can_message_id019_ETC_ECU);
            break;
          case 0x01A:
            can_signal_message_clear(&g_can_message_id01A_ETC_ECU);
            can_signal_append_raw(&g_can_message_id01A_ETC_ECU, &g_can_signal_id01A_ETC_ECU_Tps2Min, gEtcConfig.Tps2Min * 10000 / 14652);
            can_signal_append_raw(&g_can_message_id01A_ETC_ECU, &g_can_signal_id01A_ETC_ECU_Tps2Max, gEtcConfig.Tps2Max * 10000 / 14652);
            can_signal_append_raw(&g_can_message_id01A_ETC_ECU, &g_can_signal_id01A_ETC_ECU_Tps2Limit, gEtcConfig.Tps2Limit * 10000 / 14652);
            can_signal_append_raw(&g_can_message_id01A_ETC_ECU, &g_can_signal_id01A_ETC_ECU_Rsvd, value);
            can_message_send(&g_can_message_id01A_ETC_ECU);
            break;
          case 0x01B:
            can_signal_message_clear(&g_can_message_id01B_ETC_ECU);
            can_signal_append_raw(&g_can_message_id01B_ETC_ECU, &g_can_signal_id01B_ETC_ECU_Pedal1Min, gEtcConfig.Pedal1Min * 10000 / 14652);
            can_signal_append_raw(&g_can_message_id01B_ETC_ECU, &g_can_signal_id01B_ETC_ECU_Pedal1Max, gEtcConfig.Pedal1Max * 10000 / 14652);
            can_signal_append_raw(&g_can_message_id01B_ETC_ECU, &g_can_signal_id01B_ETC_ECU_Pedal2Min, gEtcConfig.Pedal2Min * 10000 / 14652);
            can_signal_append_raw(&g_can_message_id01B_ETC_ECU, &g_can_signal_id01B_ETC_ECU_Pedal2Max, gEtcConfig.Pedal2Max * 10000 / 14652);
            can_message_send(&g_can_message_id01B_ETC_ECU);
            break;
          case 0x01C:
            can_signal_message_clear(&g_can_message_id01C_ETC_ECU);
            can_signal_append_raw(&g_can_message_id01C_ETC_ECU, &g_can_signal_id01C_ETC_ECU_PidP, gEtcConfig.PidP);
            can_signal_append_raw(&g_can_message_id01C_ETC_ECU, &g_can_signal_id01C_ETC_ECU_PidI, gEtcConfig.PidI);
            can_signal_append_raw(&g_can_message_id01C_ETC_ECU, &g_can_signal_id01C_ETC_ECU_PidD, gEtcConfig.PidD);
            can_signal_append_raw(&g_can_message_id01C_ETC_ECU, &g_can_signal_id01C_ETC_ECU_TimPsc, gEtcConfig.TimPsc);
            can_signal_append_raw(&g_can_message_id01C_ETC_ECU, &g_can_signal_id01C_ETC_ECU_Rsvd, 0);
            can_message_send(&g_can_message_id01C_ETC_ECU);
            break;
          default:
            break;
        }
        break;
      case 0x089:
        can_signal_get_raw(&g_can_message_id089_ECU_ETC, &g_can_signal_id089_ECU_ETC_Tps1Min, &value);
        gEtcConfig.Tps1Min = value * 14652 / 10000;

        can_signal_get_raw(&g_can_message_id089_ECU_ETC, &g_can_signal_id089_ECU_ETC_Tps1Max, &value);
        gEtcConfig.Tps1Max = value * 14652 / 10000;

        can_signal_get_raw(&g_can_message_id089_ECU_ETC, &g_can_signal_id089_ECU_ETC_Tps1Limit, &value);
        gEtcConfig.Tps1Limit = value * 14652 / 10000;

        //can_signal_get_raw(&g_can_message_id089_ECU_ETC, &g_can_signal_id089_ECU_ETC_Rsvd, &value);

        can_signal_message_clear(&g_can_message_id018_ETC_ECU);
        can_signal_append_raw(&g_can_message_id018_ETC_ECU, &g_can_signal_id018_ETC_ECU_ConfigApplyIdAck, can_msg->Id);
        can_message_send(&g_can_message_id018_ETC_ECU);
        break;
      case 0x08A:
        can_signal_get_raw(&g_can_message_id08A_ECU_ETC, &g_can_signal_id08A_ECU_ETC_Tps2Min, &value);
        gEtcConfig.Tps2Min = value * 14652 / 10000;

        can_signal_get_raw(&g_can_message_id08A_ECU_ETC, &g_can_signal_id08A_ECU_ETC_Tps2Max, &value);
        gEtcConfig.Tps2Max = value * 14652 / 10000;

        can_signal_get_raw(&g_can_message_id08A_ECU_ETC, &g_can_signal_id08A_ECU_ETC_Tps2Limit, &value);
        gEtcConfig.Tps2Limit = value * 14652 / 10000;

        //can_signal_get_raw(&g_can_message_id08A_ECU_ETC, &g_can_signal_id08A_ECU_ETC_Rsvd, &value);

        can_signal_message_clear(&g_can_message_id018_ETC_ECU);
        can_signal_append_raw(&g_can_message_id018_ETC_ECU, &g_can_signal_id018_ETC_ECU_ConfigApplyIdAck, can_msg->Id);
        can_message_send(&g_can_message_id018_ETC_ECU);
        break;
      case 0x08B:
        can_signal_get_raw(&g_can_message_id08B_ECU_ETC, &g_can_signal_id08B_ECU_ETC_Pedal1Min, &value);
        gEtcConfig.Pedal1Min = value * 14652 / 10000;

        can_signal_get_raw(&g_can_message_id08B_ECU_ETC, &g_can_signal_id08B_ECU_ETC_Pedal1Max, &value);
        gEtcConfig.Pedal1Max = value * 14652 / 10000;

        can_signal_get_raw(&g_can_message_id08B_ECU_ETC, &g_can_signal_id08B_ECU_ETC_Pedal2Min, &value);
        gEtcConfig.Pedal2Min = value * 14652 / 10000;

        can_signal_get_raw(&g_can_message_id08B_ECU_ETC, &g_can_signal_id08B_ECU_ETC_Pedal2Max, &value);
        gEtcConfig.Pedal2Max = value * 14652 / 10000;

        can_signal_message_clear(&g_can_message_id018_ETC_ECU);
        can_signal_append_raw(&g_can_message_id018_ETC_ECU, &g_can_signal_id018_ETC_ECU_ConfigApplyIdAck, can_msg->Id);
        can_message_send(&g_can_message_id018_ETC_ECU);
        break;
      case 0x08C:
        can_signal_get_raw(&g_can_message_id018_ETC_ECU, &g_can_signal_id08C_ECU_ETC_PidP, &value);
        gEtcConfig.PidP = value;

        can_signal_get_raw(&g_can_message_id018_ETC_ECU, &g_can_signal_id08C_ECU_ETC_PidP, &value);
        gEtcConfig.PidI = value;

        can_signal_get_raw(&g_can_message_id018_ETC_ECU, &g_can_signal_id08C_ECU_ETC_PidP, &value);
        gEtcConfig.PidD = value;

        can_signal_get_raw(&g_can_message_id018_ETC_ECU, &g_can_signal_id08C_ECU_ETC_TimPsc, &value);
        gEtcConfig.TimPsc = value;

        //can_signal_get_raw(&g_can_message_id018_ETC_ECU, &g_can_signal_id08C_ECU_ETC_Rsvd, &value);

        can_signal_message_clear(&g_can_message_id018_ETC_ECU);
        can_signal_append_raw(&g_can_message_id018_ETC_ECU, &g_can_signal_id018_ETC_ECU_ConfigApplyIdAck, can_msg->Id);
        can_message_send(&g_can_message_id018_ETC_ECU);
        break;
      default:
        break;
    }
  }
}

static void can_signals_fast_send(const sEtcParametersInt *parameters)
{
  can_signal_message_clear(&g_can_message_id012_ETC);
  can_signal_append_raw(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_ThrottlePosition, parameters->ThrottlePosition * 100000 / 119998);
  can_signal_append_raw(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_TargetPosition, parameters->TargetPosition * 100000 / 119998);
  can_signal_append_raw(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_DefaultPosition, parameters->DefaultPosition * 100000 / 119998);
  can_signal_append_raw(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_PedalPosition, parameters->PedalPosition);
  can_signal_append_raw(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_Tps1ErrorFlag, gEtcStatus.Sensors.Tps1.status != HAL_OK);
  can_signal_append_raw(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_Tps2ErrorFlag, gEtcStatus.Sensors.Tps2.status != HAL_OK);
  can_signal_append_raw(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_Pedal1ErrorFlag, gEtcStatus.Sensors.Pedal1.status != HAL_OK);
  can_signal_append_raw(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_Pedal2ErrorFlag, gEtcStatus.Sensors.Pedal2.status != HAL_OK);
  can_signal_append_raw(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_TpsMismatchFlag, gEtcStatus.Sensors.TpsMismatch.status != HAL_OK);
  can_signal_append_raw(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_PedalMismatchFlag, gEtcStatus.Sensors.PedalMismatch.status != HAL_OK);
  can_signal_append_raw(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_MotorErrorFlag, parameters->MotorError != HAL_OK);
  can_signal_append_raw(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_StandaloneFlag, parameters->StandaloneMode);

  can_message_send(&g_can_message_id012_ETC);
}

static void can_signals_mid_send(const sEtcParametersInt *parameters)
{
  can_signal_message_clear(&g_can_message_id010_ETC);
  can_signal_append_raw(&g_can_message_id010_ETC, &g_can_signal_id010_ETC_Tps1, parameters->AdcTps1 * 109225 / 10000);
  can_signal_append_raw(&g_can_message_id010_ETC, &g_can_signal_id010_ETC_Tps2, parameters->AdcTps2 * 109225 / 10000);
  can_signal_append_raw(&g_can_message_id010_ETC, &g_can_signal_id010_ETC_Pedal1, parameters->AdcPedal1 * 109225 / 10000);
  can_signal_append_raw(&g_can_message_id010_ETC, &g_can_signal_id010_ETC_Pedal2, parameters->AdcPedal2 * 109225 / 10000);

  can_signal_message_clear(&g_can_message_id011_ETC);
  can_signal_append_raw(&g_can_message_id011_ETC, &g_can_signal_id011_ETC_Rsvd5, parameters->AdcRsvd5 * 109225 / 10000);
  can_signal_append_raw(&g_can_message_id011_ETC, &g_can_signal_id011_ETC_Rsvd6, parameters->AdcRsvd6 * 109225 / 10000);
  can_signal_append_raw(&g_can_message_id011_ETC, &g_can_signal_id011_ETC_ReferenceVoltage, parameters->AdcReferenceVoltage * 109225 / 10000);
  can_signal_append_raw(&g_can_message_id011_ETC, &g_can_signal_id011_ETC_PowerVoltage, parameters->AdcPowerVoltage * 32125 / 10000);

  can_message_send(&g_can_message_id010_ETC);
  can_message_send(&g_can_message_id011_ETC);
}

static void can_signals_slow_send(const sEtcParametersInt *parameters)
{
  can_signal_message_clear(&g_can_message_id013_ETC);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_InCruizeStart, sens_get_digital(SensorDigitalCruizeStart, NULL));
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_InCruizeStop, sens_get_digital(SensorDigitalCruizeStop, NULL));
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_InBrake, sens_get_digital(SensorDigitalBrake, NULL));
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_InRsvd4, sens_get_digital(SensorDigitalRsvd4, NULL));
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_InRsvd5, sens_get_digital(SensorDigitalRsvd5, NULL));
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_InRsvd6, sens_get_digital(SensorDigitalRsvd6, NULL));
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutCruizeG, out_get_state(OutCruizeG, NULL));
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutCruizeR, out_get_state(OutCruizeR, NULL));
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutRsvd3, out_get_state(OutRsvd3, NULL));
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutRsvd4, out_get_state(OutRsvd4, NULL));
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutCruizeGState, gEtcStatus.Outputs.Outs.Diagnostic.Data.Out1);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutCruizeRState, gEtcStatus.Outputs.Outs.Diagnostic.Data.Out2);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutRsvd3State, gEtcStatus.Outputs.Outs.Diagnostic.Data.Out3);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutRsvd4State, gEtcStatus.Outputs.Outs.Diagnostic.Data.Out4);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutputsAvailability, gEtcStatus.Outputs.Outs.Availability != HAL_OK);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorErrorFlag, gEtcStatus.Outputs.Motor.Diagnostic.Data.ErrorFlag);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorTemperature, gEtcStatus.Outputs.Motor.Diagnostic.Data.Temperature);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorOpenLoad, gEtcStatus.Outputs.Motor.Diagnostic.Data.OpenLoad);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorShortToGND, gEtcStatus.Outputs.Motor.Diagnostic.Data.ShortToGND);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorShortToSupply, gEtcStatus.Outputs.Motor.Diagnostic.Data.ShortToSupply);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorSupplyFailure, gEtcStatus.Outputs.Motor.Diagnostic.Data.SupplyFailure);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorAlwaysHigh, gEtcStatus.Outputs.Motor.Diagnostic.Data.AlwaysHigh);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorAvailability, gEtcStatus.Outputs.Motor.Availability != HAL_OK);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_AdcErrorFlag, gEtcStatus.AdcStatus != HAL_OK);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_CommErrorFlag, gEtcParameters.CommError != HAL_OK);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_CanInitFlag, gEtcStatus.CanInitStatus != HAL_OK);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_CanTestFlag, gEtcStatus.CanTestStatus != HAL_OK);
  can_signal_append_raw(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_AdaptationCompletedFlag, 0);

  can_message_send(&g_can_message_id013_ETC);
}

static void can_signals_update(const sEtcParametersInt *parameters)
{
  static uint32_t last_fast = 0;
  static uint32_t last_mid = 0;
  static uint32_t last_slow = 0;
  uint32_t now = Delay_Tick;

  if (DelayDiff(now, last_fast) >= 5000) {
    last_fast = now;
    can_signals_fast_send(parameters);
  }

  if (DelayDiff(now, last_mid) >= 20000) {
    last_mid = now;
    can_signals_mid_send(parameters);
  }

  if (DelayDiff(now, last_slow) >= 100000) {
    last_slow = now;
    can_signals_slow_send(parameters);
  }
}

static void etc_can_loop(void)
{
  static sCanRawMessage message = {0};
  uint32_t now = Delay_Tick;
  int8_t status;

  if(gEtcStatus.CanInitStatus == HAL_OK) {
    if(gCanTestStarted) {
      status = can_test();
      if(status != 0) {
        gCanTestStarted = 0;
        gEtcStatus.CanTestStatus = status > 0 ? HAL_OK : HAL_ERROR;
      }
    } else {
      status = can_receive(&message);
      if(status > 0) {
        etc_can_process_message(&message);
      }
    }
  }

  if (gEtcParameters.CommError == HAL_OK) {
    now = Delay_Tick;
    if(DelayDiff(now, gCommLast) > 1000000) {
      gEtcParameters.CommError = HAL_ERROR;
    }
  }
}

static void etc_can_process(void)
{
  if(gEtcStatus.CanInitStatus == HAL_OK) {
    if(!gCanTestStarted) {
      can_signals_update(&gEtcParameters);
      can_loop();
    }
  }
}

void etc_init(void)
{
  uint32_t now = Delay_Tick;

  memcpy(&gEtcConfig, &gEtcConfigDefault, sizeof(sEtcConfig));
  memset(&gEtcParameters, 0, sizeof(sEtcParametersInt));
  memset(&gEtcStatus, 0, sizeof(gEtcStatus));

  Misc_Motor_SetEnable(0);
  Misc_Motor_SetDir(0);
  Misc_Motor_SetPwm(0);

  math_pid_init(&gThrottlePid);
  math_pid_set_koffs(&gThrottlePid, gEtcConfig.PidP * 0.0001f, gEtcConfig.PidI * 0.0001f, gEtcConfig.PidD * 0.0001f);
  math_pid_set_clamp(&gThrottlePid, -200.0f, 255.0f);
  math_pid_set_integral_clamp(&gThrottlePid, -50.0f, 50.0f);

  math_pid_reset(&gThrottlePid, now);

  gEtcStatus.Sensors.Pedal1.confirm_time = 10000;
  gEtcStatus.Sensors.Pedal2.confirm_time = 10000;
  gEtcStatus.Sensors.Tps1.confirm_time = 10000;
  gEtcStatus.Sensors.Tps2.confirm_time = 10000;
  gEtcStatus.Sensors.PedalMismatch.confirm_time = 100000;
  gEtcStatus.Sensors.TpsMismatch.confirm_time = 100000;

  ecu_can_init();
}

void etc_loop(void)
{
  etc_can_loop();
  etc_can_process();

  gEtcStatus.AdcStatus = adc_get_status();
}
