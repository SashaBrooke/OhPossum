#include <math.h>

#include "rotary_utils.h"

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

        if (setpoint < upperLimit)
        {
            setpoint += maxMeasurement;
        }

        if (measurement < upperLimit)
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
