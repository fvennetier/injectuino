#ifndef _INJECTION_H
# define _INJECTION_H

#ifndef INJ_MEAN_SAMPLES
# define INJ_MEAN_SAMPLES 4
#endif

#ifndef RPM_MEAN_SAMPLES
# define RPM_MEAN_SAMPLES 2
#endif

#ifndef INJ_READ_PIN
# define INJ_READ_PIN 3
#endif

#ifndef INJ_CC_BY_MIN
# define INJ_CC_BY_MIN 1040L
#endif

#ifndef INJ_OFFSET_MICROS
# define INJ_OFFSET_MICROS 750
#endif

void injInterrupt(void);
void injTakeSample(uint16_t voltage10);
void injCompute(uint16_t *dutyCycle, uint16_t *consLiterPerHour, byte samples);
void injGetRpm(uint16_t *rpm);
void injGetTotalLiters(float *totalLiters);
void injSetTotalLiters(float totalLiters);

#endif

