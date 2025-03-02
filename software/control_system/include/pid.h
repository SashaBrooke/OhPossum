/**
 * @file pid.h
 * @brief A simple PID controller implementation.
 *
 * This implementation uses the classic PID algorithm to implement a simple
 * controller with the added benefits of using derivative low pass filtering 
 * and integral term clamping. The module also contains associated functions 
 * for configuring and updating the PID controller.
 * 
 * @note This implementation has a special use case for rotational control,
 *       able to determine the 'shortest' path to a desired setpoint by
 *       wrapping around the provided maximum measurement to zero.
 * 
 * Modified from 'PID' by Philip Salmony (see LICENSE-PID): 
 * @link https://github.com/pms67/PID
 */

#ifndef PID_H
#define PID_H

/**
 * @struct PID_t
 * @brief Structure representing a PID controller.
 *
 * This structure stores all parameters and internal states for a 
 * Proportional-Integral-Derivative (PID) controller.
 */
typedef struct {
    float Kp;               /**< Proportional gain */
    float Ki;               /**< Integral gain */
    float Kd;               /**< Derivative gain */

    float tau;              /**< Low pass filter coefficient for derivative term */

    float outLimMin;        /**< Minimum output limit */
    float outLimMax;        /**< Maximum output limit */

    float intLimMin;        /**< Minimum integrator limit */
    float intLimMax;        /**< Maximum integrator limit */

    float T;                /**< Sample time (in seconds) */

    float integrator;       /**< Current integrator value */
    float prevError;        /**< Previous error value */
    float differentiator;   /**< Current differentiator value */
    float prevMeasurement;  /**< Previous measurement value */

    float output;           /**< Current output value */
    float maxMeasurement;   /**< Maximum measurement value (for rotational angular wrapping) */
} PID_t;

/**
 * @brief Initializes a PID controller with specified parameters.
 *
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 * @param tau Low pass filter coefficient for derivative term.
 * @param outLimMin Minimum output limit.
 * @param outLimMax Maximum output limit.
 * @param intLimMin Minimum integrator limit.
 * @param intLimMax Maximum integrator limit.
 * @param T Sample time (in seconds).
 * @param maxMeasurement Maximum measurement value (for rotational angular wrapping).
 * @return A `PID_t` structure initialized with the specified parameters.
 */
PID_t PID_setup(float Kp, float Ki, float Kd, float tau, float outLimMin,
                float outLimMax, float intLimMin, float intLimMax, 
                float T, float maxMeasurement);

/**
 * @brief Resets PID saved state parameters to their defaults (0).
 * 
 * @param pid Pointer to the PID controller structure.
 */
void PID_reset(volatile PID_t *pid);

/**
 * @brief Updates the PID controller and computes a new output.
 *
 * @param pid Pointer to the PID controller structure.
 * @param setpoint The desired target setpoint value.
 * @param measurement The current measured value.
 * @param lowerLimit (Optional) To help specify allowed PID output for rotary applications
 * @param upperLimit (Optional) To help specify allowed PID output for rotary applications
 * @return The computed PID output.
 *
 * This function calculates the new PID output based on the error between 
 * the setpoint and the measurement. It updates the integrator and 
 * differentiator states.
 */
float PID_update(volatile PID_t *pid, float setpoint, float measurement,
                 float lowerLimit, float upperLimit);

/**
 * @brief Normalizes the PID output to a new specified range.
 *
 * @param pid Pointer to the PID controller structure.
 * @param newMin The new minimum value for normalization.
 * @param newMax The new maximum value for normalization.
 * @return The normalized PID output.
 *
 * This function scales the PID output to a new range specified by 
 * `newMin` and `newMax`. Useful for applications where outputs need 
 * to be within a specific range.
 */
float PID_normaliseOutput(volatile PID_t *pid, float newMin, float newMax);

#endif // PID_H
