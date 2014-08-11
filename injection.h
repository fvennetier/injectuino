#ifndef _INJECTION_H
# define _INJECTION_H

#ifndef INJ_MEAN_SAMPLES
# define INJ_MEAN_SAMPLES 4
#endif

#ifndef INJ_READ_PIN
# define INJ_READ_PIN 3
#endif

void injInterrupt(void);
void injTakeSample(void);
void injCompute(float injectorCcByMin, float *dutyCycle, float *consLiterPerHour, int *rpm);
void injGetTotalLiters(float injectorCcByMin, float *totalLiters);

#endif

