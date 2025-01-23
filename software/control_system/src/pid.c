/**
 * @file pid.c
 * @brief Source file for PID controller implementation.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "pid.h"

/* Setup the PID controller */
PID_t PID_setup(float Kp, float Ki, float Kd, float tau, 
                float outLimMin, float outLimMax, float intLimMin, 
                float intLimMax, float T, float maxMeasurement) {
    // Prevent invalid configurations
    if (T <= 0.0f) {
        fprintf(stderr, "Error: Sampling time T must be greater than zero.\n");
        exit(EXIT_FAILURE);
    }
    if (maxMeasurement <= 0.0f) {
        fprintf(stderr, "Error: Maximum measurement must be greater than zero.\n");
        exit(EXIT_FAILURE);
    }

    // Initialize the PID structure
    PID_t pid = {
        .Kp = Kp,
        .Ki = Ki,
        .Kd = Kd,
        .tau = tau,
        .outLimMin = outLimMin,
        .outLimMax = outLimMax,
        .intLimMin = intLimMin,
        .intLimMax = intLimMax,
        .T = T,
        .integrator = 0.0f,
        .prevError = 0.0f,
        .differentiator = 0.0f,
        .prevMeasurement = 0.0f,
        .output = 0.0f,
        .maxMeasurement = maxMeasurement
    };

    return pid;
}

/* Reset PID saved state parameters */
void PID_reset(volatile PID_t *pid) {
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->outLimMax = 0.0f;
}

/* Update the PID controller */
float PID_update(volatile PID_t *pid, float setpoint, float measurement) {
    // Error signal
    float error = setpoint - measurement;

    if (error > pid->maxMeasurement / 2) {
        error -= pid->maxMeasurement;
    } else if (error < -(pid->maxMeasurement / 2)) {
        error += pid->maxMeasurement;
    }

    // Proportional term
    float proportional = pid->Kp * error;

    // Integral term with anti-windup clamping
    pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    if (pid->integrator > pid->intLimMax) {
        pid->integrator = pid->intLimMax;
    } else if (pid->integrator < pid->intLimMin) {
        pid->integrator = pid->intLimMin;
    }

    // Derivative term (band-limited differentiator)
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

    // Compute output
    pid->output = proportional + pid->integrator + pid->differentiator;

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

/* Normalise controller output to specified range */
float PID_normaliseOutput(volatile PID_t *pid, float newMin, float newMax) {
    return ((pid->output - pid->outLimMin) / 
        (pid->outLimMax - pid->outLimMin) * 
        (newMax - newMin) + newMin);
}
