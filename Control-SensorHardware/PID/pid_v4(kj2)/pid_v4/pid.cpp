#include "pid.h"

PID::PID()
{
	integrator = 0;
	previous_error = 0.;
    frequency = 1.;
}

void PID::set_gains( float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}

float PID::process(float error)
{
    float output;
    integrator += error;

    output  = - kp * error;
    output += - ki * integrator / frequency;
    output += - kd * (error - previous_error) * frequency;

    previous_error = error;
    return output;
}

void PID::set_frequency(float freq)
{
    frequency = freq;
}
