#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "kalman.h"
#include "model.h"

// Function to initialize a matrix
void initialize_matrix(float32_t *data, uint16_t rows, uint16_t cols, const float32_t *values)
{
    for (int i = 0; i < rows * cols; i++)
    {
        data[i] = values[i];
    }
}

float kalman_filter(float32_t y_data[NUM_MEASUREMENTS], float32_t u_data[NUM_INPUTS])
{
    // System parameters
    float32_t a = 0.1, b = 0.2, c = 0.3;

    // State vector [angle; velocity; gyro bias]
    static float32_t x_data[NUM_STATES] = {0, 0, 0};
    static arm_matrix_instance_f32 x;
    arm_mat_init_f32(&x, NUM_STATES, 1, x_data);

    // Covariance matrix P
    static float32_t P_data[NUM_STATES * NUM_STATES] = {
        0, 0, 0,
        0, 0, 0,
        0, 0, 1e3};
    static arm_matrix_instance_f32 P;
    arm_mat_init_f32(&P, NUM_STATES, NUM_STATES, P_data);

    // Process noise covariance matrix Q
    float32_t Q_data[NUM_STATES * NUM_STATES] = {
        c * powf(Ts, 3) / 3 + a * Ts, 0, -c * powf(Ts, 2) / 2,
        0, b * Ts, 0,
        -c * powf(Ts, 2) / 2, 0, c * Ts};
    arm_matrix_instance_f32 Q;
    arm_mat_init_f32(&Q, NUM_STATES, NUM_STATES, Q_data);

    // Transition matrix phi
    float32_t phi_data[NUM_STATES * NUM_STATES] = {
        1, 0, -Ts,
        0, 1, 0,
        0, 0, 1};
    arm_matrix_instance_f32 phi;
    arm_mat_init_f32(&phi, NUM_STATES, NUM_STATES, phi_data);

    // Control input matrix G
    float32_t G_data[NUM_STATES * NUM_INPUTS] = {
        Ts, 0,
        0, L / J * Ts,
        0, 0};
    arm_matrix_instance_f32 G;
    arm_mat_init_f32(&G, NUM_STATES, NUM_INPUTS, G_data);

    // Measurement matrix H
    float32_t H_data[NUM_MEASUREMENTS * NUM_STATES] = {
        1, 0, 0,
        0, 1, 0};
    arm_matrix_instance_f32 H;
    arm_mat_init_f32(&H, NUM_MEASUREMENTS, NUM_STATES, H_data);

    // Measurement noise covariance matrix R
    float32_t R_data[NUM_MEASUREMENTS * NUM_MEASUREMENTS] = {
        0.6, 0,
        0, 0.2};
    arm_matrix_instance_f32 R;
    arm_mat_init_f32(&R, NUM_MEASUREMENTS, NUM_MEASUREMENTS, R_data);

    // Temporary matrices
    float32_t temp1_data[NUM_STATES * NUM_MEASUREMENTS];
    arm_matrix_instance_f32 temp1;
    arm_mat_init_f32(&temp1, NUM_STATES, NUM_MEASUREMENTS, temp1_data);

    float32_t temp2_data[NUM_MEASUREMENTS * NUM_MEASUREMENTS];
    arm_matrix_instance_f32 temp2;
    arm_mat_init_f32(&temp2, NUM_MEASUREMENTS, NUM_MEASUREMENTS, temp2_data);

    float32_t K_data[NUM_STATES * NUM_MEASUREMENTS];
    arm_matrix_instance_f32 K;
    arm_mat_init_f32(&K, NUM_STATES, NUM_MEASUREMENTS, K_data);

    float32_t I_data[NUM_STATES * NUM_STATES] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1};
    arm_matrix_instance_f32 I;
    arm_mat_init_f32(&I, NUM_STATES, NUM_STATES, I_data);

    float32_t estimate[NUM_MEASUREMENTS];
    float32_t error_bound[NUM_STATES];

    // Sensor measurements
    arm_matrix_instance_f32 y;
    arm_mat_init_f32(&y, NUM_MEASUREMENTS, 1, y_data);

    // Control inputs
    arm_matrix_instance_f32 u;
    arm_mat_init_f32(&u, NUM_INPUTS, 1, u_data);

    // Kalman Gain: K = P * H' / (H * P * H' + R)
    arm_mat_trans_f32(&H, &temp1);        // H'
    arm_mat_mult_f32(&P, &temp1, &temp1); // P * H'
    arm_mat_mult_f32(&H, &temp1, &temp2); // H * P * H'
    arm_mat_add_f32(&temp2, &R, &temp2);  // + R
    arm_mat_inverse_f32(&temp2, &temp2);  // Invert
    arm_mat_mult_f32(&temp1, &temp2, &K); // K = P * H' * inv(H * P * H' + R)

    // Update State: x = x + K * (y - H * x)
    float32_t y_est[NUM_MEASUREMENTS];
    arm_mat_mult_f32(&H, &x, &temp2); // H * x
    for (int i = 0; i < NUM_MEASUREMENTS; i++)
    {
        y_est[i] = y_data[i] - temp2.pData[i]; // y - H * x
    }
    for (int i = 0; i < NUM_STATES; i++)
    {
        for (int j = 0; j < NUM_MEASUREMENTS; j++)
        {
            x_data[i] += K.pData[i * NUM_MEASUREMENTS + j] * y_est[j];
        }
    }

    // Update Covariance: P = (I - K * H) * P
    arm_mat_mult_f32(&K, &H, &temp1);    // K * H
    arm_mat_sub_f32(&I, &temp1, &temp1); // I - K * H
    arm_mat_mult_f32(&temp1, &P, &P);    // Update P

    // Store Estimates
    estimate[0] = x_data[0];
    estimate[1] = x_data[1];
    for (int i = 0; i < NUM_STATES; i++)
    {
        error_bound[i] = sqrtf(P.pData[i * NUM_STATES + i]);
    }

    // Predict State: x = phi * x + G * u
    arm_mat_mult_f32(&phi, &x, &temp1); // phi * x
    arm_mat_mult_f32(&G, &u, &temp2);   // G * u
    for (int i = 0; i < NUM_STATES; i++)
    {
        x_data[i] = temp1.pData[i] + temp2.pData[i];
    }

    // Predict Covariance: P = phi * P * phi' + Q
    arm_mat_mult_f32(&phi, &P, &temp1);
    arm_mat_trans_f32(&phi, &temp2);
    arm_mat_mult_f32(&temp1, &temp2, &P);
    arm_mat_add_f32(&P, &Q, &P);

    // Print Results
    /* printf("Angle Estimate: %.2f, Velocity Estimate: %.2f, Error Bounds: [%.2f, %.2f, %.2f]\n",
           estimate[0], estimate[1], error_bound[0], error_bound[1], error_bound[2]);*/

    return estimate[0];
}
