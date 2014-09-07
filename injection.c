#include <Arduino.h>

#include "config.h"

#include "injection.h"

#define MILLIS_TO_LITERS (INJ_CC_BY_MIN / 60000000.0 * 1.024)

static unsigned long lastInjMillis = 0;
static unsigned long lastSampleTime = 0;
static volatile uint32_t injMillis = 0, injMicros = 0;
static volatile uint32_t lastOpen;
static volatile uint16_t cycle4 = 0xFFFF;

static byte sampleId = 0;
static int rpmArr[INJ_MEAN_SAMPLES] = {0};
static float dutyArr[INJ_MEAN_SAMPLES] = {0.0};

void injInterrupt(void)
{
  unsigned long m = micros();
  if (digitalRead(INJ_READ_PIN) == LOW) {
    // Injector opening
    cycle4 = (uint16_t)((m - lastOpen) >> 2);
    lastOpen = m;
    if (injMicros > 2047UL) {
      injMillis += injMicros >> 10;
      injMicros &= 2047UL;
    }
  } else {
    // Injector closing
    long diff = m - lastOpen - INJ_OFFSET_MICROS;
    if (diff > 0 && diff < 100000L) {
      injMicros += diff;
    }
  }
}

void injTakeSample(void)
{
  unsigned long curInjMillis;
  unsigned long curTime;

  cli();
  curTime = millis();
  curInjMillis = injMillis;
  sei();

  unsigned long injGap = curInjMillis - lastInjMillis;
  unsigned long timeGap = curTime - lastSampleTime;
  
  if (timeGap < 200) {
    return;
  }

  lastInjMillis = curInjMillis;
  lastSampleTime = curTime;

  if (timeGap && timeGap < 5000 && injGap > 0) {
    dutyArr[sampleId] = 1.024 * ((float)injGap) / ((float)timeGap);
    rpmArr[sampleId] = min(15000000UL / cycle4, 9999);
  } else {
    dutyArr[sampleId] = 0.0;
    rpmArr[sampleId] = 0;
  }
  sampleId = (sampleId + 1) % INJ_MEAN_SAMPLES;
}

void injCompute(float *dutyCycle, float *consLiterPerHour, int *rpm)
{
  float dutySum = 0;
  long rpmSum = 0;
  int i;

  for (i = 0; i < INJ_MEAN_SAMPLES; i++) {
    dutySum += dutyArr[i];
    rpmSum += rpmArr[i];
  }
  *dutyCycle = dutySum / ((float)INJ_MEAN_SAMPLES);
  *dutyCycle = min(100.0, *dutyCycle);
  *rpm = rpmSum / INJ_MEAN_SAMPLES;
  // cc/min * 0.06 = L/h
  *consLiterPerHour = INJ_CC_BY_MIN * 0.06 * *dutyCycle;
}

void injGetTotalLiters(float *totalLiters)
{
  unsigned long totalTime;
  cli();
  totalTime = injMillis;
  sei();
  *totalLiters = ((float)totalTime) * MILLIS_TO_LITERS;
}

void injSetTotalLiters(float totalLiters) {
  cli();
  injMillis = (unsigned long)(totalLiters / MILLIS_TO_LITERS);
  lastInjMillis = injMillis;
  sei();
}

