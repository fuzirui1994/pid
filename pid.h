/*
 * pid.h
 *
 *  Created on: Sep 11, 2009
 *      Author: pixhawk
 *  Revised on: Jan 16, 2016
 *      Author: Donghao
 */

#ifndef PID_H_
#define PID_H_

#ifdef  __cplusplus
extern "C" {
#endif

typedef struct {
		float kp;
		float ki;
		float kd;

		float sp; // Set poin of PID: the reference
		float integral;
		float error_previous;

        unsigned int feature_windup ;
        unsigned int feature_sat_max ;
        unsigned int feature_sat_min ;

		float intmax;
        float sat_max;
        float sat_min;
} PID_t;

void    pid_init( PID_t          *pid, 
                  float          kp, 
                  float          ki, 
                  float          kd);

void    pid_set( PID_t          *pid,
                 float          sp);

float   pid_calculate( PID_t          *pid,
                       float          val,
                       float          dt);

#ifdef  __cplusplus
}
#endif

#endif /* PID_H_ */
