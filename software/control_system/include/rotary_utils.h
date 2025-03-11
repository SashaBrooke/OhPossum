/**
 * @file pid.h
 * @brief A simple utilities library for rotary aplication calculations.
 */

#ifndef ROTARY_UTILS_H
#define ROTARY_UTILS_H

/**
 * @enum rotaryutils_result_e
 * @brief Available results of rotary utility calculations.
 */
typedef enum {
    ROTARYUTILS_SUCCESS,             /**< Operation was successful. */
    ROTARYUTILS_UNALLOWED_REGION,    /**< A measurement is in an unallowed region. */
} rotaryutils_result_e;

/**
 * @brief Calculates the rotary error considering wrap-around at `maxMeasurement`.
 * 
 * This function calculates the shortest path error between a given measurement
 * and the desired setpoint within a circular measurement space.
 * 
 * @param[out] error Pointer to store the calculated error.
 * @param[in] measurement The current measured value.
 * @param[in] setpoint The desired target value.
 * @param[in] maxMeasurement The value at which wrap-around occurs.
 * 
 * @return ROTARYUTILS_SUCCESS on success, ROTARYUTILS_UNALLOWED_REGION if the operation is invalid.
 */
rotaryutils_result_e calculate_rotary_error(float *error, float measurement,
    float setpoint, float maxMeasurement);

/**
 * @brief Calculates the rotary error with enforced lower and upper limits.
 * 
 * This function calculates the shortest path error between a given measurement
 * and the desired setpoint while ensuring that the 'error path' chosen remains within
 * the specified limits.
 * 
 * @param[out] error Pointer to store the calculated error.
 * @param[in] measurement The current measured value.
 * @param[in] setpoint The desired target value.
 * @param[in] maxMeasurement The value at which wrap-around occurs.
 * @param[in] lowerLimit The minimum allowable measurement/setpoint value.
 * @param[in] upperLimit The maximum allowable measurement/setpoint value.
 * 
 * @return ROTARYUTILS_SUCCESS on success, ROTARYUTILS_UNALLOWED_REGION if the operation is invalid.
 */
rotaryutils_result_e calculate_rotary_error__limits(float *error,
    float measurement, float setpoint, float maxMeasurement, float lowerLimit,
    float upperLimit);

/**
 * @brief Checks if a given value is within specified rotary limits.
 * 
 * @param[in] value The value to check.
 * @param[in] lowerLimit The minimum allowable value.
 * @param[in] upperLimit The maximum allowable value.
 * 
 * @return ROTARYUTILS_SUCCESS if the value is within limits, ROTARYUTILS_UNALLOWED_REGION otherwise.
 */
rotaryutils_result_e in_rotary_limits(float value, float lowerLimit, 
    float upperLimit);

#endif  // ROTARY_UTILS_H
