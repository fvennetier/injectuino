#include <Arduino.h>

#include "config.h"

#include "injection.h"

#define MILLIS_TO_LITERS (INJ_CC_BY_MIN / 60000000.0 * 1.024)

static unsigned long lastInjMillis = 0;
static unsigned long lastSampleTime = 0;
static volatile uint32_t injMillis = 0;
static uint16_t injMicros = 0;
static volatile uint16_t lastInjMicros = 0;
static volatile uint32_t lastOpen;
// Time between two cycles divided by 4, in microseconds
static volatile uint16_t cycleBy4 = 0xFFFF;
static volatile signed char voltageDiff = 0;


static byte sampleId = 0;
static short rpmArr[RPM_MEAN_SAMPLES] = {0};
static short dutyArr[INJ_MEAN_SAMPLES] = {0};

void injInterrupt(void)
{
  unsigned long m = micros();
  if (digitalRead(INJ_READ_PIN) == LOW) {
    // Injector opening
    // Time elapsed since last opening
    cycleBy4 = (uint16_t)((m - lastOpen) >> 2);
    lastOpen = m;
    // Move micros to millis, keep the carry
    injMillis += injMicros >> 10;  // injMicros / 1024
    injMicros &= 1023U;
  } else {
    // Injector closing
    // Time elapsed since opening
    lastInjMicros = (m - lastOpen) & 0xFFFF;
    // Substract injector offset (plus 100µs per volt under 14V)
    uint16_t correction = INJ_OFFSET_MICROS + ((int16_t)voltageDiff)*10;
    int16_t diff = lastInjMicros - correction;
    if (diff > 0) {
      injMicros += diff;
    }
  }
}

void injTakeSample(short voltage10)
{
  unsigned long curInjMillis;
  unsigned long curTime;

  voltageDiff = (signed char)(140 - voltage10);

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

  if (timeGap && timeGap < 5000uL && injGap > 0uL) {
    dutyArr[sampleId % INJ_MEAN_SAMPLES] = (1024 * injGap) / timeGap;
    // (60 * 1000000 / 4) / (cycle / 4)
    rpmArr[sampleId % RPM_MEAN_SAMPLES] = min(15000000UL / cycleBy4, 9999);
  } else {
    dutyArr[sampleId % INJ_MEAN_SAMPLES] = 0;
    rpmArr[sampleId % RPM_MEAN_SAMPLES] = 0;
  }
  sampleId++;
}

void injCompute(short *dutyCycle10, short *consLiterPerHour10, byte samples)
{
  unsigned long dutySum = 0;
  byte i;
  short id;

  for (i = 0; i < samples; i++) {
    id = (sampleId + INJ_MEAN_SAMPLES - i) % INJ_MEAN_SAMPLES;
    dutySum += dutyArr[id];
  }
  *dutyCycle10 = dutySum / samples;
  *dutyCycle10 = min(1000, *dutyCycle10);

  // cc/min * 0.06 = L/h
  *consLiterPerHour10 = (short)((INJ_CC_BY_MIN * *dutyCycle10) / 1666L);
}

void injGetRpm(short *rpm)
{
  int i;
  long rpmSum = 0;
  for (i = 0; i < RPM_MEAN_SAMPLES; i++) {
    rpmSum += rpmArr[i];
  }
  *rpm = rpmSum / RPM_MEAN_SAMPLES;
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

