#ifndef _INJECTION_H
# define _INJECTION_H

#ifndef INJ_MEAN_SAMPLES
# define INJ_MEAN_SAMPLES 4
#endif

#ifndef INJ_READ_PIN
# define INJ_READ_PIN 3
#endif

#ifndef INJ_CC_BY_MIN
# define INJ_CC_BY_MIN 1040.0
#endif

void injInterrupt(void);
void injTakeSample(void);
void injCompute(float *dutyCycle, float *consLiterPerHour, int *rpm);
void injGetTotalLiters(float *totalLiters);

#endif

