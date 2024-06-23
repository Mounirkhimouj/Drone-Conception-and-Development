/*
 * rc_controller.c
 *
 *  Created on: Apr 19, 2024
 *      Author: LENOVO
 */


#include "rc_controller.h"
#include "pid.h"
#include <stdio.h>
#include <string.h>

RC_t rc;
IMU imu;

void parseData(unsigned char *receivedData) {
	float values[MAX_VALUES];

	char *token;
	int i = 0;

	  // Use strtok to split the string based on commas
	  token = strtok(receivedData, ",");

	  // Loop until there are no more tokens
	  while (token != NULL) {
	    // Convert the token string to an integer using atoi
	    values[i] = atoi(token);

	    // Check for parsing errors (atoi returns 0 on error)
	   // if (values[i] == 0 && strcmp(token, "0") != 0) {
	     // printf("Error parsing value: %s\n", token);
	      //parse_error_flag = 1;
	      //return ; // Exit with error code
	   // }

	    i++;
	    token = strtok(NULL, ",");


	    //Ensure we don't exceed the expected number of values
	    if (i == MAX_VALUES) {
	    	//printf("Warning: More values than expected in the string\n");
	      break;
	    }


	  }


	  //int numParsed = sscanf(receivedData, "%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,",
	    //                        &values[0], &values[1], &values[2], &values[3],
	      //                      &values[4], &values[5], &values[6], &values[7]);

	  // Check if we parsed the expected number of values
	 // if (i < MAX_VALUES) {
	   // printf("Warning: Fewer values than expected in the string\n");
	    //parse_error_flag =1;
	  //}

		// Now you can access individual values using the values array
		if (values[1] == RC_CMD){
			rc.LH = values[3];
			rc.LV = values[6];
			rc.RH = values[5];
			rc.RV = values[4];
			rc.SwL = values[7];
			rc.SwR = values[8];
		}
		else if(values[1] == PC_CMD)
		{
			if (values[2] == YAW_ID)
			{
				yaw_heading.kp = values[3]/1000.0;
				yaw_heading.ki = values[4]/1000.0;
				yaw_heading.kd = values[5]/1000.0;
				yaw_rate.kp = values[6]/1000.0;
				yaw_rate.ki = values[7]/1000.0;
				yaw_rate.kd = values[8]/1000.0;
			}
			else if (values[2] == ROLL_ID)
			{
				roll.in.kp = values[3]/1000.0;
				roll.in.ki = values[4]/1000.0;
				roll.in.kd = values[5]/1000.0;
				roll.out.kp = values[6]/1000.0;
				roll.out.ki = values[7]/1000.0;
				roll.out.kd = values[8]/1000.0;
			}
			else if (values[2] == PITCH_ID)
			{
				pitch.in.kp = values[3]/1000.0;
				pitch.in.ki = values[4]/1000.0;
				pitch.in.kd = values[5]/1000.0;
				pitch.out.kp = values[6]/1000.0;
				pitch.out.ki = values[7]/1000.0;
				pitch.out.kd = values[8]/1000.0;
			}
			else if (values[2] == ALT_ID)
			{
				altitude_heading.kp = values[3]/1000.0;
				altitude_heading.ki = values[4]/1000.0;
				altitude_heading.kd = values[5]/1000.0;
				altitude_rate.kp = values[6]/1000.0;
				altitude_rate.ki = values[7]/1000.0;
				altitude_rate.kd = values[8]/1000.0;
			}
	  }
	  else if(values[1] == PC_ORR)
	  {
		  imu.OrrX = values[5];
		  imu.OrrY = values[6];
		  imu.OrrZ = values[7];
		  imu.GyroZ = values[4];
		  imu.GyroY = values[3];
		  imu.GyroX = values[2];
		  imu.LACCx = values[8];
		  imu.LACCy = values[9];
		  imu.LACCz = values[10];

	  }
}
