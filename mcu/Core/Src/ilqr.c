#include <stdio.h>
#include <math.h>
#include "ilqr.h"
#include "model.h"

void LQR_init(LQR_Controller *lqr)
{
    // Linearize the system
    lqr->A.pData[0] = 0.;
    lqr->A.pData[1] = 1.;
    lqr->A.pData[2] = 0.;
    lqr->A.pData[3] = 0.;

    lqr->B.pData[0] = 0.;
    lqr->B.pData[1] = L / J;

    arm_matrix_instance_f32 Q, R;

    static float32_t Q_data[] = {
        1000, 0,
        0, 1000};
    static float32_t R_data[] = {
        1};

    arm_mat_init_f32(&Q, NUMBER_STATES, NUMBER_STATES, Q_data);
    arm_mat_init_f32(&R, NUMBER_CONTROLS, NUMBER_CONTROLS, R_data);
}

// LQR controller function
void LQR_update(LQR_Controller *lqr, float state[NUMBER_STATES], float control[NUMBER_CONTROLS])
{
	// Origin points
	float z_origin[NUMBER_STATES] = {0};
	float u_rest[NUMBER_CONTROLS] = {0};

    int n = lqr->A.numRows;
    arm_matrix_instance_f32 K, At, Bt, temp1, temp2, P, P_prev;

    // Temporary matrices
    float32_t K_data[NUMBER_CONTROLS * NUMBER_STATES];
    float32_t At_data[n * n];
    float32_t Bt_data[n * n];
    float32_t temp1_data[n * n];
    float32_t temp2_data[n * n];
    float32_t P_data[NUMBER_STATES * NUMBER_STATES];
    float32_t P_prev_data[NUMBER_STATES * NUMBER_STATES];

    // Initialize temporary matrices
    arm_mat_init_f32(&K, NUMBER_CONTROLS, NUMBER_STATES, K_data);
    arm_mat_init_f32(&At, n, n, At_data);
    arm_mat_init_f32(&Bt, n, n, Bt_data);
    arm_mat_init_f32(&temp1, n, n, temp1_data);
    arm_mat_init_f32(&temp2, n, n, temp2_data);
    arm_mat_init_f32(&P, NUMBER_STATES, NUMBER_STATES, P_data);
    arm_mat_init_f32(&P_prev, NUMBER_STATES, NUMBER_STATES, P_prev_data);

    // Initialize P with Q
    arm_mat_copy_f32(lqr->Q, P);

    // Solve Algebraic Riccati Equation using Newton's Method
    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
        // Compute K = R^{-1} B^T P
        arm_mat_trans_f32(&lqr->B, &Bt);      // Bt = B^T
        arm_mat_mult_f32(&Bt, &P, &temp1);    // temp1 = B^T * P
        arm_mat_inverse_f32(&lqr->R, &temp2); // temp2 = R^{-1}
        arm_mat_mult_f32(&temp2, &temp1, &K); // K = R^{-1} * (B^T * P)

        // Compute A_k = A - B * K
        arm_mat_mult_f32(&lqr->B, &K, &temp1); // temp1 = B * K
        arm_mat_sub_f32(&lqr->A, &temp1, &At); // At = A - B * K

        // Solve Lyapunov Equation: At^T * P_next + P_next * At + Q + K^T * R * K = 0
        arm_mat_trans_f32(&At, &Bt);              // Bt = At^T
        arm_mat_mult_f32(&K, &lqr->R, &temp1);    // temp1 = K * R
        arm_mat_mult_f32(&temp1, &K, &temp2);     // temp2 = K * R * K^T
        arm_mat_add_f32(&lqr->Q, &temp2, &temp1); // temp1 = Q + K^T * R * K

        arm_mat_mult_f32(&P, &Bt, &P_prev);   // P_prev = At^T * P
        arm_mat_add_f32(&P_prev, &temp1, &P); // P = P_prev + temp1

        // Convergence check
        float32_t diff = 0.0f;
        for (int i = 0; i < n * n; ++i)
            diff += fabsf(P.pData[i] - P_prev.pData[i]);
        if (diff < EPSILON)
            break;
    }

    // Compute u_stabilize = K * (state - z_origin)
    float32_t delta_state[NUMBER_STATES];
    float32_t u_stabilize[NUMBER_CONTROLS];
    for (int i = 0; i < NUMBER_STATES; i++)
    {
        delta_state[i] = state[i] - z_origin[i];
    }
    arm_matrix_instance_f32 delta_state_mat, u_stabilize_mat;
    arm_mat_init_f32(&delta_state_mat, NUMBER_STATES, 1, delta_state);
    arm_mat_init_f32(&u_stabilize_mat, NUMBER_CONTROLS, 1, u_stabilize);

    arm_mat_mult_f32(&K, &delta_state_mat, &u_stabilize_mat);

    // Compute u_original = u_stabilize + u_rest
    for (int i = 0; i < NUMBER_CONTROLS; i++)
    {
        //u_original[i] = u_stabilize[i] + u_rest[i];
    }
}
