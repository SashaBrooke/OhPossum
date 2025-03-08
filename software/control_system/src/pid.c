/**
 * @file pid.c
 * @brief Source file for PID controller implementation.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "pid.h"
#include "rotary_utils.h"
#include "gimbal_configuration.h"

#define NO_OUTPUT 0.0f

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
    pid->output = 0.0f;
}

/* Update the PID controller */
float PID_update(volatile PID_t *pid, float setpoint, float measurement,
                 float lowerLimit, float upperLimit) {
    // Error signal
    float error;
    rotaryutils_result_e result;

    if (lowerLimit == GIMBAL_DEFAULT_UNSET_ROM || upperLimit == GIMBAL_DEFAULT_UNSET_ROM) {
        result = calculate_rotary_error(&error, measurement, setpoint, pid->maxMeasurement);
    } else if (lowerLimit == GIMBAL_360_ROM && upperLimit == GIMBAL_360_ROM) {
        result = calculate_rotary_error(&error, measurement, setpoint, pid->maxMeasurement);
    } else {
        if (in_rotary_limits(measurement, lowerLimit, upperLimit) == ROTARYUTILS_UNALLOWED_REGION) {
            // Get nearest limit
            float toLower, toUpper;
            calculate_rotary_error(&toLower, measurement, lowerLimit, pid->maxMeasurement);
            calculate_rotary_error(&toUpper, measurement, upperLimit, pid->maxMeasurement);

            // Return max output towards nearest limit
            float closest = fminf(fabs(toLower), fabs(toUpper));
            float dirToClosestLimit = fabs(toLower) == closest ? toLower / closest : toUpper / closest;

            pid->output = pid->outLimMax * dirToClosestLimit; // Can lower this to reduce instability near limits
            return pid->output;
        }

        result = calculate_rotary_error__limits(&error, measurement, setpoint, pid->maxMeasurement,
            lowerLimit, upperLimit);
    }

    if (result != ROTARYUTILS_SUCCESS) {
        pid->output = NO_OUTPUT;
        return pid->output;
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
