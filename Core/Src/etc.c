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

static sEtcConfig gEtcParams;
static const sEtcConfig gEtcParamsDefault = {
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

    .PidP = 2800,
    .PidI = 4000,
    .PidD = 43
};

static sEtcStatus gEtcStatus;

volatile uint16_t pedal;
volatile int16_t tps;
volatile uint8_t pwm;

static sMathPid gThrottlePid;

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
  HAL_StatusTypeDef etc_status = HAL_OK;
  uint32_t now = Delay_Tick;
  int16_t pedal1, pedal2;
  int16_t pedal1_v, pedal2_v;
  int16_t pedal1_vv, pedal2_vv;
  uint16_t pedal1_v_diff, pedal2_v_diff;

  pedal1_vv = adc_get_voltage(AdcChPedal1);
  pedal2_vv = adc_get_voltage(AdcChPedal2);

  pedal1_v = CLAMP(pedal1_vv, gEtcParams.Pedal1Min, gEtcParams.Pedal1Max);
  pedal2_v = CLAMP(pedal2_vv, gEtcParams.Pedal2Min, gEtcParams.Pedal2Max);
  pedal1_v_diff = gEtcParams.Pedal1Max - gEtcParams.Pedal1Min;
  pedal2_v_diff = gEtcParams.Pedal2Max - gEtcParams.Pedal2Min;

  etc_status |= etc_error_ctx_handle(&gEtcStatus.Sensors.Pedal1, abs(pedal1_vv - pedal1_v) > 200);
  etc_status |= etc_error_ctx_handle(&gEtcStatus.Sensors.Pedal2, abs(pedal2_vv - pedal2_v) > 200);

  pedal1_v -= gEtcParams.Pedal1Min;
  pedal2_v -= gEtcParams.Pedal2Min;

  pedal1 = pedal1_v * 8192 / pedal1_v_diff;
  pedal2 = pedal2_v * 8192 / pedal2_v_diff;

  pedal = (pedal1 + pedal2) / 2;

  etc_status |= etc_error_ctx_handle(&gEtcStatus.Sensors.PedalMismatch, abs(pedal1 - pedal2) > 100);



  int16_t tps1, tps2;
  int16_t tps1_v, tps2_v;
  int16_t tps1_vv, tps2_vv;
  int16_t tps1_v_diff, tps2_v_diff;

  tps1_vv = adc_get_voltage(AdcChTps1);
  tps2_vv = adc_get_voltage(AdcChTps2);

  tps1_v = CLAMP(tps1_vv, gEtcParams.Tps1Min, gEtcParams.Tps1Limit);
  tps2_v = CLAMP(tps2_vv, gEtcParams.Tps2Limit, gEtcParams.Tps2Min);
  tps1_v_diff = gEtcParams.Tps1Max - gEtcParams.Tps1Min;
  tps2_v_diff = gEtcParams.Tps2Min - gEtcParams.Tps2Max;

  etc_status |= etc_error_ctx_handle(&gEtcStatus.Sensors.Tps1, abs(tps1_vv - tps1_v) > 200);
  etc_status |= etc_error_ctx_handle(&gEtcStatus.Sensors.Tps2, abs(tps2_vv - tps2_v) > 200);

  tps1_v -= gEtcParams.Tps1Min;
  tps2_v -= gEtcParams.Tps2Max;

  tps1 = tps1_v * 8192 / tps1_v_diff;
  tps2 = tps2_v * 8192 / tps2_v_diff;

  tps2 = 8192 - tps2;

  tps = (tps1 + tps2) / 2;

  etc_status |= etc_error_ctx_handle(&gEtcStatus.Sensors.TpsMismatch, abs(tps1 - tps2) > 100);

  int16_t pid;
  if(etc_status == HAL_OK) {
    math_pid_set_koffs(&gThrottlePid, gEtcParams.PidP * 0.0001f, gEtcParams.PidI * 0.0001f, gEtcParams.PidD * 0.0001f);
    math_pid_set_target(&gThrottlePid, pedal);
    pid = math_pid_update(&gThrottlePid, tps, now);

    if(tps <= 0) {
      math_pid_set_clamp(&gThrottlePid, -60.0f, 255.0f);
    } else if(tps >= 8192) {
      math_pid_set_clamp(&gThrottlePid, -200.0f, 60.0f);
    } else {
      math_pid_set_clamp(&gThrottlePid, -200.0f, 255.0f);
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

  memcpy(&gEtcParams, &gEtcParamsDefault, sizeof(sEtcConfig));
  memset(&gEtcStatus, 0, sizeof(gEtcStatus));

  Misc_Motor_SetEnable(0);
  Misc_Motor_SetDir(0);
  Misc_Motor_SetPwm(0);

  math_pid_init(&gThrottlePid);
  math_pid_set_koffs(&gThrottlePid, gEtcParams.PidP * 0.0001f, gEtcParams.PidI * 0.0001f, gEtcParams.PidD * 0.0001f);
  math_pid_set_clamp(&gThrottlePid, -200.0f, 255.0f);
  math_pid_reset(&gThrottlePid, now);

  gEtcStatus.Sensors.Pedal1.confirm_time = 10000;
  gEtcStatus.Sensors.Pedal2.confirm_time = 10000;
  gEtcStatus.Sensors.Tps1.confirm_time = 10000;
  gEtcStatus.Sensors.Tps2.confirm_time = 10000;
  gEtcStatus.Sensors.PedalMismatch.confirm_time = 100000;
  gEtcStatus.Sensors.TpsMismatch.confirm_time = 100000;
}

void etc_loop(void)
{
  outputs_get_diagnostic(&gEtcStatus.Outputs);
}
