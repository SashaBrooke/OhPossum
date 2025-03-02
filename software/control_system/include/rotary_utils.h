#ifndef ROTARY_UTILS_H
#define ROTARY_UTILS_H

typedef enum {
    ROTARYUTILS_SUCCESS,
    ROTARYUTILS_UNALLOWED_REGION,
} rotaryutils_result_e;

rotaryutils_result_e calculate_rotary_error(float *error, float measurement,
    float setpoint, float maxMeasurement);

rotaryutils_result_e calculate_rotary_error__limits(float *error,
    float measurement, float setpoint, float maxMeasurement, float lowerLimit,
    float upperLimit);

#endif  // ROTARY_UTILS_H
