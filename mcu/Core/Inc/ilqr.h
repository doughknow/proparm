#include <arm_math.h>

#define NUMBER_STATES 2
#define NUMBER_CONTROLS 2
#define EPSILON 1e-6
#define MAX_ITER 1000

typedef struct
{
    float state[NUMBER_STATES];
    float u_rest[NUMBER_CONTROLS];
    arm_matrix_instance_f32 Q;
    arm_matrix_instance_f32 R;
    arm_matrix_instance_f32 K;
    arm_matrix_instance_f32 A;
    arm_matrix_instance_f32 B;
} LQR_Controller;

void LQR_init(LQR_Controller *lqr);
void LQR_update(LQR_Controller *lqr, float state[NUMBER_STATES], float control[NUMBER_CONTROLS]);
