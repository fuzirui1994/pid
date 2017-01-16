/*
 * pid.c
 *
 *  Created on: Sep 11, 2009
 *      Author: pixhawk
 *  Revised on: Jan 16, 2016
 *      Author: Donghao
 */

#include "pid.h"

/**
 *
 * @param pid
 * @param kp
 * @param ki
 * @param kd
 */
void pid_init(PID_t *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->sp = 0;
	pid->error_previous = 0;
	pid->integral = 0;

    pid->features = 0;

}

// Change the reference of PID ;
void pid_set(PID_t *pid, float sp)
{
	pid->sp = sp;
}

/**
 *
 * @param pid
 * @param val: feedback value 
 * @param dt: Sample time
 * @return: value of actuator
 */
 // dt is not is the struct PID_t in case of variant sample time;
float pid_calculate(PID_t *pid, float val, float dt)
{
	float i,d, error, total;

	error = pid->sp - val;
	i = pid->integral + (error * dt);
	d = (error - pid->error_previous) / dt;

    total = (error * pid->kp) + (i * pid->ki) + (d * pid->kd);

    if ( pid.feature_windup )
    {
        if ( i < 0 )
            i = ( i < -pid->intmax ? -pid->intmax : i );
        else
   		    i = ( i < pid->intmax ? i : pid->intmax );
    }

    // Update the integral and error ;
    pid->integral = i;
	pid->error_previous = error;

    if ( (pid->feature_sat_min ) && (total < pid->sat_min) )
        return pid->sat_min;
    if ( (pid->feature_sat_max ) && (total > pid->sat_max) )
        return pid->sat_max;

	return total;
}
