/*
 * pid.h
 *
 *  Created on: Apr 25, 2024
 *      Author: LENOVO
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#ifdef __cplusplus
 extern "C" {
#endif


typedef struct _PIDSingle
{
	float kp;
	float ki;
	float kd;

	float reference;
	float meas_value; // currenct angle from MPU
	float meas_value_prev;
	float error;
	float error_sum;
	float error_deriv;
	float error_deriv_filt;

	float p_result;
	float i_result;
	float d_result;

	float pid_result;
}PIDSingle;

typedef struct _PIDDouble
{
	PIDSingle in;
	PIDSingle out;
}PIDDouble;


extern PIDDouble roll;
extern PIDDouble pitch;
extern PIDDouble altitude;
extern PIDSingle yaw_heading;
extern PIDSingle yaw_rate;
extern PIDSingle altitude_heading;
extern PIDSingle altitude_rate;


void Double_Roll_Pitch_PID_Calculation(PIDDouble* axis, float set_point_angle, float angle, float rate);
void Single_Yaw_Rate_PID_Calculation(PIDSingle* axis, float set_point, float value);
void Single_Yaw_Heading_PID_Calculation(PIDSingle* axis, float set_point, float angle, float rate);
void Single_Alt_Heading_PID_Calculation(PIDSingle* axis, float set_point_altitude, float height, float rate);
void Single_Alt_Rate_PID_Calculation(PIDSingle* axis, float set_point_altitude, float rate);
void Reset_PID_Integrator(PIDSingle* axis);
void Reset_All_PID_Integrator(void);


#ifdef __cplusplus
}
#endif

#endif /* INC_PID_H_ */
