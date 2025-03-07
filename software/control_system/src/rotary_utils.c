#include <math.h>

#include "rotary_utils.h"

/*
    Get the error between two points, accounting for wrapping around `maxMeasurement`
    and with total freedom for 'error path' choice
*/
rotaryutils_result_e
calculate_rotary_error(float *rotError, float measurement, float setpoint, 
    float maxMeasurement)
{
    float tmpRotError = setpoint - measurement;

    if (tmpRotError > maxMeasurement / 2) {
        tmpRotError -= maxMeasurement;
    } else if (tmpRotError < -(maxMeasurement / 2)) {
        tmpRotError += maxMeasurement;
    }

    *rotError = tmpRotError;

    return ROTARYUTILS_SUCCESS;
}

/*
    Get the error between two points, accounting for wrapping around `maxMeasurement`
    and with 'error path' choice restricted by two limits (lower and upper)
*/
rotaryutils_result_e
calculate_rotary_error__limits(float *rotError, float measurement, float setpoint, 
    float maxMeasurement, float lowerLimit, float upperLimit)
{
    if (upperLimit < lowerLimit)
    {
        if (setpoint < lowerLimit && setpoint > upperLimit)
        {
            return ROTARYUTILS_UNALLOWED_REGION;
        }

        if (setpoint <= upperLimit)
        {
            setpoint += maxMeasurement;
        }

        if (measurement <= upperLimit)
        {
            measurement += maxMeasurement;
        }

        *rotError = setpoint - measurement;

        return ROTARYUTILS_SUCCESS;
    }
    else
    {
        if (setpoint < lowerLimit || setpoint > upperLimit)
        {
            return ROTARYUTILS_UNALLOWED_REGION;
        }

        *rotError = setpoint - measurement;
        return ROTARYUTILS_SUCCESS;
    }
}

/* Determine whether a value is within rotary limits or not */
rotaryutils_result_e
in_rotary_limits(float value, float lowerLimit, float upperLimit)
{
    if (upperLimit < lowerLimit && value < lowerLimit && value > upperLimit)
    {
        return ROTARYUTILS_UNALLOWED_REGION;
    }
    else if (upperLimit > lowerLimit && (value < lowerLimit || value > upperLimit))
    {
        return ROTARYUTILS_UNALLOWED_REGION;
    }
    else
    {
        return ROTARYUTILS_SUCCESS;
    }
}
