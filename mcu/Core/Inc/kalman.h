#include "arm_math.h"

#define NUM_STATES 3
#define NUM_MEASUREMENTS 2
#define NUM_INPUTS 2

float kalman_filter(float32_t y_data[NUM_MEASUREMENTS], float32_t u_data[NUM_INPUTS]);
