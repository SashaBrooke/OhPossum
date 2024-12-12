// TODO: Documentation

#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float tau;

    float outLimMin;
    float outLimMax;

    float intLimMin;
    float intLimMax;

    float T;

    float integrator;
    float prevError;
    float differentiator;
    float prevMeasurement;

    float output;

    float maxMeasurement;
} PID_t;

PID_t PID_setup(float Kp, float Ki, float Kd, float tau, float outLimMin,
                float outLimMax, float intLimMin, float intLimMax, 
                float T, float maxMeasurement);

float PID_update(volatile PID_t *pid, float setpoint, float measurement);

float PID_normaliseOutput(volatile PID_t *pid, float newMin, float newMax);

#endif
