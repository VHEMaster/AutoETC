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
#include <string.h>

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
};

volatile uint16_t pedal;
volatile int16_t tps;
volatile uint8_t pwm;

static sMathPid gThrottlePid;

static void etc_throttle_loop(void)
{
  uint32_t now = Delay_Tick;
  uint16_t pedal1, pedal2;
  uint16_t pedal1_v, pedal2_v;
  uint16_t pedal1_v_diff, pedal2_v_diff;

  pedal1_v = adc_get_voltage(AdcChPedal1);
  pedal2_v = adc_get_voltage(AdcChPedal2);

  pedal1_v = CLAMP(pedal1_v, gEtcParams.Pedal1Min, gEtcParams.Pedal1Max);
  pedal2_v = CLAMP(pedal2_v, gEtcParams.Pedal2Min, gEtcParams.Pedal2Max);
  pedal1_v_diff = gEtcParams.Pedal1Max - gEtcParams.Pedal1Min;
  pedal2_v_diff = gEtcParams.Pedal2Max - gEtcParams.Pedal2Min;

  pedal1_v -= gEtcParams.Pedal1Min;
  pedal2_v -= gEtcParams.Pedal2Min;

  pedal1 = pedal1_v * 8192 / pedal1_v_diff;
  pedal2 = pedal2_v * 8192 / pedal2_v_diff;

  pedal = (pedal1 + pedal2) / 2;

  float p = pedal;
  p /= 8192;
  p *= p;
  p *= 8192;
  pedal = p;

  int16_t pid;
  int16_t tps1, tps2;
  int16_t tps1_v, tps2_v;
  int16_t tps1_v_diff, tps2_v_diff;

  tps1_v = adc_get_voltage(AdcChTps1);
  tps2_v = adc_get_voltage(AdcChTps2);

  tps1_v = CLAMP(tps1_v, gEtcParams.Tps1Min, gEtcParams.Tps1Limit);
  tps2_v = CLAMP(tps2_v, gEtcParams.Tps2Limit, gEtcParams.Tps2Min);
  tps1_v_diff = gEtcParams.Tps1Max - gEtcParams.Tps1Min;
  tps2_v_diff = gEtcParams.Tps2Min - gEtcParams.Tps2Max;

  tps1_v -= gEtcParams.Tps1Min;
  tps2_v -= gEtcParams.Tps2Max;

  tps1 = tps1_v * 8192 / tps1_v_diff;
  tps2 = tps2_v * 8192 / tps2_v_diff;

  tps2 = 8192 - tps2;

  tps = (tps1 + tps2) / 2;

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

  Misc_Motor_SetEnable(1);
  Misc_Motor_SetDir(0);
  Misc_Motor_SetPwm(0);


  math_pid_init(&gThrottlePid);
  math_pid_set_koffs(&gThrottlePid, 0.3f, 1.0f, 0.0035f);
  math_pid_set_clamp(&gThrottlePid, -200.0f, 255.0f);
  math_pid_reset(&gThrottlePid, now);
}

void etc_loop(void)
{

}
