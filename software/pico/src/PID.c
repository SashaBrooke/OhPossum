// TODO: Documentation

// #include <stdio.h> // For debugging
#include <stdlib.h>

#include "PID.h"

/** 
 * @brief Initializes the PID controller 
 * 
 * @note PID struct is dynamically allocated and therefore must be freed after use.
*/
PID_t *PID_setup(float Kp, float Ki, float Kd, float tau, float outLimMin,
                float outLimMax, float intLimMin, float intLimMax, float T) {
    PID_t *pid = (PID_t *)malloc(sizeof(PID_t));
    if (pid == NULL) {
        return NULL;
    }

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->tau = tau;

    pid->outLimMin = outLimMin;
    pid->outLimMax = outLimMax;

    pid->intLimMin = intLimMin;
    pid->intLimMax = intLimMax;

    pid->T = T;

    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->output = 0.0f;

    return pid;
}

/* Update the PID controller */
float PID_update(volatile PID_t *pid, float setpoint, float measurement) {
    // Error signal
    float error = setpoint - measurement;
    // printf("Error: %f\n", error);

    // Proportional term
    float proportional = pid->Kp * error;
    // printf("Proportional: %f\n", proportional);

    // Integral term with anti-windup clamping
    pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);
    // printf("Integrator: %f\n", pid->integrator);

    // if (pid->integrator > pid->intLimMax) {
    //     pid->integrator = pid->intLimMax;
    // } else if (pid->integrator < pid->intLimMin) {
    //     pid->integrator = pid->intLimMin;
    // }

    // Derivative term (band-limited differentiator)
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);
    // printf("Differentiator: %f\n", pid->differentiator);

    // Compute output
    pid->output = proportional + pid->integrator + pid->differentiator;
    // printf("Pre-clamp output: %f\n", pid->output);

    // Clamp output
    if (pid->output > pid->outLimMax) {
        pid->output = pid->outLimMax;
    } else if (pid->output < pid->outLimMin) {
        pid->output = pid->outLimMin;
    }

    // Save state for next update
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    return pid->output;
}
