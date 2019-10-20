#include <Arduino.h>

#include "config.h"

#include "injection.h"

#define MILLIS_TO_LITERS (INJ_CC_BY_MIN / 60000000.0 * 1.024)

/* Copy the value of a variable in an interrupt-safe manner. */
#define SAFE_COPY(type_,var_) ({ \
  volatile type_ __tmp0, __tmp1; \
  do { \
    __tmp0 = var_; \
    __tmp1 = var_; \
  } while (__tmp0 != __tmp1); \
  __tmp1; \
})

static uint32_t lastInjMillis = 0;
static uint32_t lastSampleTime = 0;
static volatile uint32_t injMillis = 0;
static uint16_t injMicros = 0;
static volatile uint16_t lastInjMicros = 0;
static volatile uint32_t lastOpen;
// Time between two cycles divided by 4, in microseconds
static volatile uint16_t cycleBy4 = 0xFFFF;
static volatile signed char injVoltCorrection = 0;


static byte sampleId = 0;
static uint16_t rpmArr[RPM_MEAN_SAMPLES] = {0};
static uint16_t dutyArr[INJ_MEAN_SAMPLES] = {0};

void injInterrupt(void)
{
  uint32_t now = micros();
  if (digitalRead(INJ_READ_PIN) == LOW) {
    // Injector opening
    // Time elapsed since last opening
    cycleBy4 = (uint16_t)((now - lastOpen) >> 2);
    lastOpen = now;
    // Move micros to millis, keep the carry
    injMillis += injMicros >> 10;  // injMicros / 1024
    injMicros &= 1023U;
  } else {
    // Injector closing
    // Time elapsed since opening
    lastInjMicros = (now - lastOpen) & 0xFFFF;
    // Substract injector offset (plus 120µs per volt under 14V)
    uint16_t correction = INJ_OFFSET_MICROS + injVoltCorrection;
    int16_t diff = lastInjMicros - correction;
    if (diff > 0) {
      injMicros += diff;
    }
  }
}

void injTakeSample(uint16_t voltage10)
{
  uint32_t curInjMillis;
  uint32_t curTime;

  int16_t correction = (140 - voltage10) * 12;
  injVoltCorrection = constrain(correction, 0, 255);

  curTime = millis();
  curInjMillis = SAFE_COPY(uint32_t, injMillis);

  uint32_t injGap = curInjMillis - lastInjMillis;
  uint32_t timeGap = curTime - lastSampleTime;

  if (timeGap < 200) {
    return;
  }

  lastInjMillis = curInjMillis;
  lastSampleTime = curTime;

  if (timeGap && timeGap < 5000uL && injGap > 0uL) {
    dutyArr[sampleId % INJ_MEAN_SAMPLES] = (1024 * injGap) / timeGap;
    // (60 * 1000000 / 4) / (cycle / 4)
    rpmArr[sampleId % RPM_MEAN_SAMPLES] = min(15000000UL / cycleBy4, 9999);
  } else {
    dutyArr[sampleId % INJ_MEAN_SAMPLES] = 0;
    rpmArr[sampleId % RPM_MEAN_SAMPLES] = 0;
    lastInjMicros = 0;
  }
  sampleId++;
}

void injCompute(uint16_t *dutyCycle10, uint16_t *consLiterPerHour10, byte samples)
{
  uint32_t dutySum = 0;
  byte i;
  byte id;

  for (i = 0; i < samples; i++) {
    id = (sampleId + INJ_MEAN_SAMPLES - i) % INJ_MEAN_SAMPLES;
    dutySum += dutyArr[id];
  }
  *dutyCycle10 = dutySum / samples;
  *dutyCycle10 = min(1000, *dutyCycle10);

  // cc/min * 0.06 = L/h
  *consLiterPerHour10 = (uint16_t)((INJ_CC_BY_MIN * *dutyCycle10) / 1666L);
}

void injGetSmoothRpm(uint16_t *rpm)
{
  byte i;
  uint32_t rpmSum = 0;
  for (i = 0; i < RPM_MEAN_SAMPLES; i++) {
    rpmSum += rpmArr[i];
  }
  *rpm = (uint16_t)(rpmSum / RPM_MEAN_SAMPLES);
}

void injGetRpm(uint16_t *rpm)
{
  uint32_t cycle = 4UL * SAFE_COPY(uint16_t, cycleBy4);
  *rpm = (uint16_t)(60 * 1000000 / cycle);
}

void injGetTotalLiters(float *totalLiters)
{
  uint32_t totalTime = SAFE_COPY(uint32_t, injMillis);
  *totalLiters = ((float)totalTime) * MILLIS_TO_LITERS;
}

void injSetTotalLiters(float totalLiters) {
  cli();
  injMillis = (uint32_t)(totalLiters / MILLIS_TO_LITERS);
  lastInjMillis = injMillis;
  sei();
}

