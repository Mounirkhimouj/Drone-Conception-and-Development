/*
 * mpu6050.h
 *
 *  Created on: Apr 19, 2024
 *      Author: LENOVO
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include "main.h"
#define MPU6050_ADDR_0  0xD0
#define MPU6050_ADDR_1  0xD2

#define PI 3.14159265358979323846


#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define INT_ENABLE       0x38
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C

#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_REG     0x75
#define ACCEL_CONFIG     0x1C
#define ACCEL_XOUT_H     0x3B
#define TEMP_OUT_H       0x41
#define GYRO_CONFIG      0x1B
#define GYRO_XOUT_H      0x43
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

#define CALIB_ACCEL_SENSITIVITY 16384
#define CALIB_GYRO_SENSITIVITY 131

// MPU6050 structure
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
	float acc_bias[3];   // acc calibration value in ACCEL_FS_SEL: 2g
	float gyro_bias[3];  // gyro calibration value in GYRO_FS_SEL: 250dps
	float Angle_Roll,Angle_Pitch ;


	float q[4];

	float KalmanAngleRoll, KalmanUncertaintyAngleRoll;
	float KalmanAnglePitch, KalmanUncertaintyAnglePitch;
	float KalmanDOutput[2];


} MPU6050_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

void print_float(char *str, float data);

void print_calibration(MPU6050_t *MPU9250);

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx,char);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct,char);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct,char);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct,char);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct,char);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

void calibrate(I2C_HandleTypeDef *I2Cx,MPU6050_t *DataStruc,char addr);

void set_acc_gyro_to_calib(I2C_HandleTypeDef *I2Cx,char addr);

void collect_acc_gyro_data_to(I2C_HandleTypeDef *I2Cx,MPU6050_t *DataStruct,char addr);

void write_accel_offset(I2C_HandleTypeDef *I2Cx,MPU6050_t *DataStruc,char addr);

void write_gyro_offset(I2C_HandleTypeDef *I2Cx,MPU6050_t *MPU6050,char addr);

float getAccBiasX(MPU6050_t *DataStruct);
float getAccBiasY(MPU6050_t *DataStruct);
float getAccBiasZ(MPU6050_t *DataStruct);
float getGyroBiasX(MPU6050_t *DataStruct);
float getGyroBiasY(MPU6050_t *DataStruct);
float getGyroBiasZ(MPU6050_t *DataStruct);

//void mahony(MPU6050_t *DataStruct);

void MPU6050SetDefault(MPU6050_t *MPU6050);

//void update_rpy(MPU6050_t *MPU6050);
void Roll_Pitch(MPU6050_t *MPU6050);
void Kalman_filter(MPU6050_t *MPU6050);
//void Kalman_filter_pitch(MPU6050_t *MPU6050);



#endif INC_MPU6050_H_
