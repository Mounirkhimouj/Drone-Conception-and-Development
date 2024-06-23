/*
 * rc_controller.h
 *
 *  Created on: Apr 19, 2024
 *      Author: LENOVO
 */

#ifndef INC_RC_CONTROLLER_H_
#define INC_RC_CONTROLLER_H_


#include "main.h"

#define MAX_VALUES 11 // Adjust this if you expect a different number of values
#define RC_CMD 0
#define PC_CMD 1
#define PC_ORR 4
#define YAW_ID 2
#define ROLL_ID 3
#define PITCH_ID 1
#define ALT_ID	4

typedef struct
{
	int RH; //Right Horizontal
	int RV; //Right Vertical
	int LV; //Left Vertical
	int LH; //Left Horizontal
	uint8_t SwL;
	uint8_t SwR;
}RC_t;

typedef struct
{
	int OrrX; //Right Horizontal
	int OrrY; //Right Vertical
	int OrrZ; //Left Vertical
	int GyroZ;
	int GyroY;
	int GyroX;
	int LACCx;
	int LACCy;
	int LACCz;
}IMU;




extern RC_t rc;
extern IMU imu;


void parseData(unsigned char *receivedData);


#endif /* INC_RC_CONTROLLER_H_ */
