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
} PID_t;

void PID_Init(volatile PID_t *pid, float Kp, float Ki, float Kd, float tau, float outLimMin,
                float outLimMax, float intLimMin, float intLimMax, float T);

float PID_Update(volatile PID_t *pid, float setpoint, float measurement);

#endif
