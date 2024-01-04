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
}sEtcStatus;

static sEtcConfig gEtcConfig;
static const sEtcConfig gEtcConfigDefault = {
    .Tps1Min = 500,
    .Tps1Mid = 866,
    .Tps1Max = 4256,
    .Tps1Limit = 4649,
    .Tps2Min = 4481,
    .Tps2Mid = 4110,
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

static sEtcParametersInt gEtcParameters;
static sEtcStatus gEtcStatus;
static sMathPid gThrottlePid;
static uint16_t gEtcTargetPosition;

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

  gEtcParameters.AdcPedal1 = adc_get_voltage(AdcChPedal1);
  gEtcParameters.AdcPedal2 = adc_get_voltage(AdcChPedal2);

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

  gEtcParameters.PedalPosition = (gEtcParameters.Pedal1 + gEtcParameters.Pedal2) / 2;

  gEtcParameters.PedalError |= etc_error_ctx_handle(&gEtcStatus.Sensors.PedalMismatch, abs(gEtcParameters.Pedal1 - gEtcParameters.Pedal2) > 100);



  int16_t tps1_v, tps2_v;
  int16_t tps1_v_diff, tps2_v_diff;

  gEtcParameters.AdcTps1 = adc_get_voltage(AdcChTps1);
  gEtcParameters.AdcTps2 = adc_get_voltage(AdcChTps2);

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

  gEtcParameters.ThrottlePosition = (gEtcParameters.Tps1 + gEtcParameters.Tps2) / 2;

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
      gEtcTargetPosition = gEtcParameters.DefaultPosition;
      pos_filtered = gEtcTargetPosition;
    }
  }

  if(gEtcParameters.CommError != HAL_OK) {
    gEtcTargetPosition = gEtcParameters.DefaultPosition;
    if(gEtcParameters.DefaultPosition + gEtcParameters.PedalPosition < 8191) {
      gEtcParameters.TargetPosition = gEtcParameters.DefaultPosition + gEtcParameters.PedalPosition;
    }
  } else {
    gEtcParameters.TargetPosition = gEtcTargetPosition;
  }

  gEtcParameters.TargetPosition = CLAMP(gEtcParameters.TargetPosition, 0, 8191);

  pos_filtered = (gEtcParameters.TargetPosition * 1000 + pos_filtered * 9000) / 10000;

  if(gEtcParameters.TpsError == HAL_OK && gEtcParameters.PedalError == HAL_OK && gEtcParameters.DefaultPosition != 0) {
    math_pid_set_koffs(&gThrottlePid, gEtcConfig.PidP * 0.0001f, gEtcConfig.PidI * 0.0001f, gEtcConfig.PidD * 0.0001f);
    math_pid_set_target(&gThrottlePid, pos_filtered);
    pid = math_pid_update(&gThrottlePid, gEtcParameters.ThrottlePosition, now);

    if(gEtcParameters.ThrottlePosition <= 0) {
      math_pid_set_clamp(&gThrottlePid, -60.0f, 255.0f);
    } else if(gEtcParameters.ThrottlePosition >= 8191) {
      math_pid_set_clamp(&gThrottlePid, -160.0f, 60.0f);
    } else {
      math_pid_set_clamp(&gThrottlePid, -160.0f, 255.0f);
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

  gEtcParameters.CommError = HAL_ERROR;
}

void etc_loop(void)
{
}
