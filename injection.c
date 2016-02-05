#include <Arduino.h>

#include "config.h"

#include "injection.h"

#define MILLIS_TO_LITERS (INJ_CC_BY_MIN / 60000000.0 * 1.024)

static unsigned long lastInjMillis = 0;
static unsigned long lastSampleTime = 0;
static volatile uint32_t injMillis = 0, injMicros = 0;
static volatile uint32_t lastOpen;
// Time between two cycles divided by 4, in microseconds
static volatile uint16_t cycle4 = 0xFFFF;
static volatile char voltageDiff = 0;

static byte sampleId = 0;
static short rpmArr[RPM_MEAN_SAMPLES] = {0};
static short dutyArr[INJ_MEAN_SAMPLES] = {0};

void injInterrupt(void)
{
  unsigned long m = micros();
  if (digitalRead(INJ_READ_PIN) == LOW) {
    // Injector opening
    cycle4 = (uint16_t)((m - lastOpen) >> 2);
    lastOpen = m;
    if (injMicros > 1023UL) {
      injMillis += injMicros >> 10;
      injMicros &= 1023UL;
    }
  } else {
    // Injector closing, add 100µs per volt to offset
    long diff = m - lastOpen - INJ_OFFSET_MICROS - ((short)voltageDiff)*10;
    if (diff > 0 && diff < 100000L) {
      injMicros += diff;
    }
  }
}

void injTakeSample(short voltage10)
{
  unsigned long curInjMillis;
  unsigned long curTime;

  voltageDiff = 138 - voltage10;

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
    dutyArr[sampleId % INJ_MEAN_SAMPLES] = (1024 * injGap) / timeGap;
    // (60 * 1000000 / 4) / (cycles / 4)
    rpmArr[sampleId % RPM_MEAN_SAMPLES] = min(15000000UL / cycle4, 9999);
  } else {
    dutyArr[sampleId % INJ_MEAN_SAMPLES] = 0;
    rpmArr[sampleId % RPM_MEAN_SAMPLES] = 0;
  }
  sampleId++;
}

void injCompute(short *dutyCycle10, short *consLiterPerHour10, byte samples)
{
  long dutySum = 0;
  byte i;

  for (i = 0; i < samples; i++) {
    dutySum += dutyArr[(sampleId - i) % (byte)INJ_MEAN_SAMPLES];
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

