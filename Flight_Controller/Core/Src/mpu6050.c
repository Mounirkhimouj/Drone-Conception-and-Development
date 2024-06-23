#include <math.h>
#include "mpu6050.h"
#include "stdio.h"
#define RAD_TO_DEG 57.295779513082320876798154814105



// Setup MPU6050


const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx,char addr)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, addr , WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
    	Data = 0x80;
    	HAL_I2C_Mem_Write(I2Cx,addr, PWR_MGMT_1,1, &Data,1,i2c_timeout);
    	// Write a one to bit 7 reset bit; toggle reset device
    	HAL_Delay(100);
    	// power management register 0X6B we should write all 0's to wake the sensor up
    	Data = 0;
        HAL_I2C_Mem_Write(I2Cx, addr , PWR_MGMT_1, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, addr , SMPLRT_DIV, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, addr , ACCEL_CONFIG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, addr , GYRO_CONFIG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct,char addr)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, addr , ACCEL_XOUT_H, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct,char addr)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, addr , GYRO_XOUT_H, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct,char addr)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, addr , TEMP_OUT_H, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct,char addr)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, addr , ACCEL_XOUT_H, 1, Rec_Data, 14, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};
void calibrate(I2C_HandleTypeDef *I2Cx,MPU6050_t *DataStruct,char addr){
	HAL_Delay(5000);
	printf("Accel Gyro calibration will start in 5sec.\n\r Please leave the device still on the flat plane. \n\r");
	HAL_Delay(5000);
	set_acc_gyro_to_calib(I2Cx,addr);
	collect_acc_gyro_data_to(I2Cx,DataStruct,addr);
	write_accel_offset(I2Cx,DataStruct,addr);
	write_gyro_offset(I2Cx, DataStruct, addr);
	HAL_Delay(100);
	MPU6050_Init(I2Cx,addr);
	printf("MPU6050 is calibrated .\n\r");
	HAL_Delay(1000);
}

void set_acc_gyro_to_calib(I2C_HandleTypeDef *I2Cx,char addr){
	uint8_t Data;
	// reset device
	Data = 0x00;
			HAL_I2C_Mem_Write(I2Cx,addr, PWR_MGMT_1,1, &Data,1,i2c_timeout);
	    	// Write a one to bit 7 reset bit; toggle reset device
	    	HAL_Delay(100);
	        // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	        // else use the internal oscillator, bits 2:0 = 001
	    	Data = 0x01;
	        HAL_I2C_Mem_Write(I2Cx, addr, PWR_MGMT_1, 1, &Data, 1, i2c_timeout);
	        HAL_I2C_Mem_Write(I2Cx, addr, PWR_MGMT_2, 1, 0x00, 1, i2c_timeout);
	        HAL_Delay(200);

	        // Configure device for bias calculation
	        Data = 0x0C;
	        HAL_I2C_Mem_Write(I2Cx,addr, INT_ENABLE,1, 0x00,1,i2c_timeout);    // Disable all interrupts
	        HAL_I2C_Mem_Write(I2Cx,addr, FIFO_EN,1, 0x00,1,i2c_timeout);       // Disable FIFO
	        HAL_I2C_Mem_Write(I2Cx,addr, PWR_MGMT_1,1, 0x00,1,i2c_timeout);    // Turn on internal clock source
	        HAL_I2C_Mem_Write(I2Cx,addr, I2C_MST_CTRL,1, 0x00,1,i2c_timeout);  // Disable I2C master
	        HAL_I2C_Mem_Write(I2Cx,addr, USER_CTRL,1, 0x00,1,i2c_timeout);     // Disable FIFO and I2C master modes
	        HAL_I2C_Mem_Write(I2Cx,addr, USER_CTRL,1, &Data,1,i2c_timeout);     // Reset FIFO and DMP
	        HAL_Delay(15);

	        // Configure MPU6050 gyro and accelerometer for bias calculation
	        Data = 0x01;
	        HAL_I2C_Mem_Write(I2Cx,addr, CONFIG,1, &Data,1,i2c_timeout);    // Set low-pass filter to 188 Hz
	        HAL_I2C_Mem_Write(I2Cx,addr, SMPLRT_DIV,1, 0x00,1,i2c_timeout);    // Set sample rate to 1 kHz
	        HAL_I2C_Mem_Write(I2Cx,addr, GYRO_CONFIG,1, 0x00,1,i2c_timeout);   // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	        HAL_I2C_Mem_Write(I2Cx,addr, ACCEL_CONFIG,1, 0x00,1,i2c_timeout);  // Set accelerometer full-scale to 2 g, maximum sensitivity

	        // Configure FIFO to capture accelerometer and gyro data for bias calculation
	        Data = 0x40;
	        HAL_I2C_Mem_Write(I2Cx,addr, USER_CTRL,1, &Data,1,i2c_timeout);  // Enable FIFO
	        Data = 0x78 ;
	        HAL_I2C_Mem_Write(I2Cx,addr, FIFO_EN,1, &Data,1,i2c_timeout);    // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	        HAL_Delay(40);

}

void collect_acc_gyro_data_to(I2C_HandleTypeDef *I2Cx,MPU6050_t *DataStruct,char addr){
	uint8_t data[12];                                    // data array to hold accelerometer and gyro x, y, z, data
	HAL_I2C_Mem_Write(I2Cx, addr,FIFO_EN,1,0x00,1,i2c_timeout);// Disable gyro and accelerometer sensors for FIFO
	HAL_I2C_Mem_Read(I2Cx, addr , FIFO_COUNTH, 1, &data[0], 2, i2c_timeout);// read FIFO sample count
	uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
	printf("fifo count : %d\n\r",fifo_count);
	uint16_t packet_count = fifo_count / 12;  // How many sets of full gyro and accelerometer data for averaging

	for (uint16_t ii = 0; ii < packet_count; ii++) {
		            int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};

		            HAL_I2C_Mem_Read(I2Cx,addr,FIFO_R_W,1,&data[0],12,i2c_timeout); // read data for averaging
		            accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
		            accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
		            accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
		            gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
		            gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
		            gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

		            DataStruct->acc_bias[0]  += (float)accel_temp[0];  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		            DataStruct->acc_bias[1]  += (float)accel_temp[1];
		            DataStruct->acc_bias[2]  += (float)accel_temp[2];
		            DataStruct->gyro_bias[0] += (float)gyro_temp[0];
		            DataStruct->gyro_bias[1] += (float)gyro_temp[1];
		            DataStruct->gyro_bias[2] += (float)gyro_temp[2];
		        }
    DataStruct->acc_bias[0]  /= (float)packet_count;  // Normalize sums to get average count biases
    DataStruct->acc_bias[1]  /= (float)packet_count;
    DataStruct->acc_bias[2]  /= (float)packet_count;
    DataStruct->gyro_bias[0] /= (float)packet_count;
    DataStruct->gyro_bias[1] /= (float)packet_count;
    DataStruct->gyro_bias[2] /= (float)packet_count;

    if (DataStruct->acc_bias[2] > 0L) {
        DataStruct->acc_bias[2] -= (float)CALIB_ACCEL_SENSITIVITY;
    }  // Remove gravity from the z-axis accelerometer bias calculation
    else {
        DataStruct->acc_bias[2] += (float)CALIB_ACCEL_SENSITIVITY;
    }
	printf("acc and gyro collected . \n\r");


}


void write_accel_offset(I2C_HandleTypeDef *I2Cx,MPU6050_t *DataStruct,char addr){
	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	        // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	        // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	        // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	        // the accelerometer biases calculated above must be divided by 8.

			uint8_t read_data[2] = {0};
			int16_t acc_bias_reg[3] = {0, 0, 0};                      // A place to hold the factory accelerometer trim biases
			HAL_I2C_Mem_Read(I2Cx,addr,XA_OFFSET_H,1,&read_data[0],2,i2c_timeout);    // Read factory accelerometer trim values
			acc_bias_reg[0] = ((int16_t)read_data[0] << 8) | read_data[1];

			HAL_I2C_Mem_Read(I2Cx,addr,YA_OFFSET_H,1,&read_data[0],2,i2c_timeout);
			acc_bias_reg[1] = ((int16_t)read_data[0] << 8) | read_data[1];

			HAL_I2C_Mem_Read(I2Cx,addr,ZA_OFFSET_H,1,&read_data[0],2,i2c_timeout);
			acc_bias_reg[2] = ((int16_t)read_data[0] << 8) | read_data[1];

	        int16_t mask_bit[3] = {1, 1, 1};  // Define array to hold mask bit for each accelerometer bias axis
	        for (int i = 0; i < 3; i++) {
	            if (acc_bias_reg[i] % 2) {
	                mask_bit[i] = 0;
	            }
	            acc_bias_reg[i] -= (int16_t)DataStruct->acc_bias[i] >> 3;  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
	            if (mask_bit[i]) {
	                acc_bias_reg[i] = acc_bias_reg[i] & ~mask_bit[i];  // Preserve temperature compensation bit
	            } else {
	                acc_bias_reg[i] = acc_bias_reg[i] | 0x0001;  // Preserve temperature compensation bit
	            }
	        }
	        uint8_t write_data[6] = {0};
	        write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
	        write_data[1] = (acc_bias_reg[0]) & 0xFF;
	        write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
	        write_data[3] = (acc_bias_reg[1]) & 0xFF;
	        write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
	        write_data[5] = (acc_bias_reg[2]) & 0xFF;

	        // Push accelerometer biases to hardware registers
	        HAL_I2C_Mem_Write(I2Cx,addr, XA_OFFSET_H,1, write_data[0],1,i2c_timeout);
	        HAL_I2C_Mem_Write(I2Cx,addr, XA_OFFSET_L,1, write_data[1],1,i2c_timeout);
	        HAL_I2C_Mem_Write(I2Cx,addr, YA_OFFSET_H,1, write_data[2],1,i2c_timeout);
	        HAL_I2C_Mem_Write(I2Cx,addr, YA_OFFSET_L,1, write_data[3],1,i2c_timeout);
	        HAL_I2C_Mem_Write(I2Cx,addr, ZA_OFFSET_H,1, write_data[4],1,i2c_timeout);
	        HAL_I2C_Mem_Write(I2Cx,addr, ZA_OFFSET_L,1, write_data[5],1,i2c_timeout);
	    	printf("Accel offset is written. \n\r");


}

void write_gyro_offset(I2C_HandleTypeDef *I2Cx,MPU6050_t *MPU6050,char addr){
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	        uint8_t gyro_offset_data[6] = {0};
	        gyro_offset_data[0] = (-(int16_t)MPU6050->gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	        gyro_offset_data[1] = (-(int16_t)MPU6050->gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
	        gyro_offset_data[2] = (-(int16_t)MPU6050->gyro_bias[1] / 4 >> 8) & 0xFF;
	        gyro_offset_data[3] = (-(int16_t)MPU6050->gyro_bias[1] / 4) & 0xFF;
	        gyro_offset_data[4] = (-(int16_t)MPU6050->gyro_bias[2] / 4 >> 8) & 0xFF;
	        gyro_offset_data[5] = (-(int16_t)MPU6050->gyro_bias[2] / 4) & 0xFF;

	        // Push gyro biases to hardware registers
	        HAL_I2C_Mem_Write(I2Cx,addr, XG_OFFSET_H,1, gyro_offset_data[0],1,i2c_timeout);
	        HAL_I2C_Mem_Write(I2Cx,addr, XG_OFFSET_L,1, gyro_offset_data[1],1,i2c_timeout);
	        HAL_I2C_Mem_Write(I2Cx,addr, YG_OFFSET_H,1, gyro_offset_data[2],1,i2c_timeout);
	        HAL_I2C_Mem_Write(I2Cx,addr, YG_OFFSET_L,1, gyro_offset_data[3],1,i2c_timeout);
	        HAL_I2C_Mem_Write(I2Cx,addr, ZG_OFFSET_H,1, gyro_offset_data[4],1,i2c_timeout);
	        HAL_I2C_Mem_Write(I2Cx,addr, ZG_OFFSET_L,1, gyro_offset_data[5],1,i2c_timeout);
	    	printf("gyro offset is written. \n\r");

}

float getAccBiasX(MPU6050_t *DataStruct) { return DataStruct->acc_bias[0]; }
float getAccBiasY(MPU6050_t *DataStruct) { return DataStruct->acc_bias[1]; }
float getAccBiasZ(MPU6050_t *DataStruct) { return DataStruct->acc_bias[2]; }
float getGyroBiasX(MPU6050_t *DataStruct) { return DataStruct->gyro_bias[0]; }
float getGyroBiasY(MPU6050_t *DataStruct) { return DataStruct->gyro_bias[1]; }
float getGyroBiasZ(MPU6050_t *DataStruct) { return DataStruct->gyro_bias[2]; }

void print_calibration(MPU6050_t *MPU6050){

	float abx = getAccBiasX(MPU6050) * 1000.f / (float)CALIB_ACCEL_SENSITIVITY;
	float aby = getAccBiasY(MPU6050) * 1000.f / (float)CALIB_ACCEL_SENSITIVITY;
	float abz = getAccBiasZ(MPU6050) * 1000.f / (float)CALIB_ACCEL_SENSITIVITY;
	float gbx = getGyroBiasX(MPU6050) / (float)CALIB_GYRO_SENSITIVITY;
	float gby = getGyroBiasY(MPU6050) / (float)CALIB_GYRO_SENSITIVITY;
	float gbz = getGyroBiasZ(MPU6050) / (float)CALIB_GYRO_SENSITIVITY;
	char str[] = "< calibration parameters > ";
	printf(str);
	printf("\n\r accel bias [g]: ");
	print_float(str, abx);
	print_float(str, aby);
	print_float(str, abz);


	printf("\n\r gyro bias [deg/s]: ");
	print_float(str, gbx);
	print_float(str, gby);
	print_float(str, gbz);
	printf("\n\r");

}
void print_float(char *str, float data){
	//sprintf(str, " %d.%05d  ",(uint32_t)fabs(data), (uint16_t)((data - (uint32_t)data)*100000.));
	sprintf(str, "  %.4f  ", data);
	printf(str);
}

void MPU6050_Read(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct,char addr)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, addr , ACCEL_XOUT_H, 1, Rec_Data, 14, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

/*void mahony(MPU6050_t *DataStruct){
		double dt = (double)(HAL_GetTick() - timer) / 1000000;
		timer = HAL_GetTick();

        float recipNorm;
        float vx, vy, vz;
        float ex, ey, ez;  //error terms
        float qa, qb, qc;
        static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
        float tmp;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        tmp = DataStruct->Ax * DataStruct->Ax + DataStruct->Ay * DataStruct->Ay + DataStruct->Az * DataStruct->Az;
        if (tmp > 0.0) {
            // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
            recipNorm = 1.0 / sqrt(tmp);
            DataStruct->Ax *= recipNorm;
            DataStruct->Ay *= recipNorm;
            DataStruct->Az *= recipNorm;

            // Estimated direction of gravity in the body frame (factor of two divided out)
            vx = DataStruct->q[1] * DataStruct->q[3] - DataStruct->q[0] * DataStruct->q[2];
            vy = DataStruct->q[0] * DataStruct->q[1] + DataStruct->q[2] * DataStruct->q[3];
            vz = DataStruct->q[0] * DataStruct->q[0] - 0.5f + DataStruct->q[3] * DataStruct->q[3];

            // Error is cross product between estimated and measured direction of gravity in body frame
            // (half the actual magnitude)
            ex = (DataStruct->Ay * vz - DataStruct->Az * vy);
            ey = (DataStruct->Az * vx - DataStruct->Ax * vz);
            ez = (DataStruct->Ax * vy - DataStruct->Ay * vx);

            // Compute and apply to gyro term the integral feedback, if enabled
            if (DataStruct->Ki > 0.0f) {
                ix += DataStruct->Ki * ex * dt;  // integral error scaled by Ki
                iy += DataStruct->Ki * ey * dt;
                iz += DataStruct->Ki * ez * dt;
                DataStruct->Gx += ix;  // apply integral feedback
                DataStruct->Gy += iy;
                DataStruct->Gz += iz;
            }

            // Apply proportional feedback to gyro term
            DataStruct->Gx += DataStruct->Kp * ex;
            DataStruct->Gy += DataStruct->Kp * ey;
            DataStruct->Gz += DataStruct->Kp * ez;
        }

        // Integrate rate of change of quaternion, q cross gyro term
        dt = 0.5*dt;
        DataStruct->Gx *= dt;  // pre-multiply common factors
        DataStruct->Gy *= dt;
        DataStruct->Gz *= dt;

        qa = DataStruct->q[0];
        qb = DataStruct->q[1];
        qc = DataStruct->q[2];
        DataStruct->q[0] += (-qb * DataStruct->Gx - qc * DataStruct->Gy - DataStruct->q[3] * DataStruct->Gz);
        DataStruct->q[1] += (qa * DataStruct->Gx + qc * DataStruct->Gz - DataStruct->q[3] * DataStruct->Gy);
        DataStruct->q[2] += (qa * DataStruct->Gy - qb * DataStruct->Gz + DataStruct->q[3] * DataStruct->Gx);
        DataStruct->q[3] += (qa * DataStruct->Gz + qb * DataStruct->Gy - qc * DataStruct->Gx);

        // renormalise quaternion
        recipNorm = 1.0 / sqrt(DataStruct->q[0] * DataStruct->q[0] + DataStruct->q[1] * DataStruct->q[1] + DataStruct->q[2] * DataStruct->q[2] + DataStruct->q[3] * DataStruct->q[3]);
        DataStruct->q[0] = DataStruct->q[0] * recipNorm;
        DataStruct->q[1] = DataStruct->q[1] * recipNorm;
        DataStruct->q[2] = DataStruct->q[2] * recipNorm;
        DataStruct->q[3] = DataStruct->q[3] * recipNorm;
    }
*/
void MPU6050SetDefault(MPU6050_t *MPU6050){

	MPU6050->Ax = 0;
	MPU6050->Ay = 0;
	MPU6050->Az = 0;
	MPU6050->Gx = 0;
	MPU6050->Gy = 0;
	MPU6050->Gz = 0;
	MPU6050->KalmanAngleRoll=0;
	MPU6050->KalmanUncertaintyAngleRoll=2*2;
	MPU6050->KalmanAnglePitch=0;
	MPU6050->KalmanUncertaintyAnglePitch=2*2;
	MPU6050->Angle_Roll  = 0 ;
	MPU6050->Angle_Pitch = 0 ;
	MPU6050->q[0]=1;
	MPU6050->q[1]=0;
	MPU6050->q[2]=0;
	MPU6050->q[3]=0;





}

/*void update_rpy(MPU6050_t *MPU6050){
	// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	        // In this coordinate system, the positive z-axis is down toward Earth.
	        // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	        // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	        // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	        // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	        // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	        // applied in the correct order which for this configuration is yaw, pitch, and then roll.
	        // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
	        float a12, a22, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components

	        a12 = 2.0f * (MPU6050->q[1] * MPU6050->q[2] + MPU6050->q[0] * MPU6050->q[3]);
	        a22 = MPU6050->q[0] * MPU6050->q[0] + MPU6050->q[1] * MPU6050->q[1] - MPU6050->q[2] * MPU6050->q[2] - MPU6050->q[3] * MPU6050->q[3];
	        a31 = 2.0f * (MPU6050->q[0] * MPU6050->q[1] + MPU6050->q[2] * MPU6050->q[3]);
	        a32 = 2.0f * (MPU6050->q[1] * MPU6050->q[3] - MPU6050->q[0] * MPU6050->q[2]);
	        a33 = MPU6050->q[0] * MPU6050->q[0] - MPU6050->q[1] * MPU6050->q[1] - MPU6050->q[2] * MPU6050->q[2] + MPU6050->q[3] * MPU6050->q[3];
	        MPU6050->rpy[0] = atan2f(a31, a33);
	        MPU6050->rpy[1] = -asinf(a32);
	        MPU6050->rpy[2] = atan2f(a12, a22);
	        MPU6050->rpy[0] *= 180.0f / PI;
	        MPU6050->rpy[1] *= 180.0f / PI;
	        MPU6050->rpy[2] *= 180.0f / PI;
}*/

void Roll_Pitch(MPU6050_t *MPU6050){

	MPU6050->Angle_Roll = atan(MPU6050->Ay/sqrt(MPU6050->Ax*MPU6050->Ax+MPU6050->Az*MPU6050->Az))*1/(3.142/180);
	MPU6050->Angle_Pitch = -atan(MPU6050->Ax/sqrt(MPU6050->Ay*MPU6050->Ay+MPU6050->Az*MPU6050->Az))*1/(3.142/180);

}

void Kalman_filter(MPU6050_t *MPU6050){

	double dt = (double)(HAL_GetTick() - timer) / 1000;
	timer = HAL_GetTick();

	MPU6050->KalmanAngleRoll = MPU6050->KalmanAngleRoll +dt * MPU6050->Gx;
	MPU6050->KalmanUncertaintyAngleRoll= MPU6050->KalmanUncertaintyAngleRoll+dt*dt*3*3;
	float KalmanGain0= MPU6050->KalmanUncertaintyAngleRoll*1/(1* MPU6050->KalmanUncertaintyAngleRoll+4*4);
	MPU6050->KalmanAngleRoll=MPU6050->KalmanAngleRoll+KalmanGain0*( MPU6050->Angle_Roll-MPU6050->KalmanAngleRoll);
	MPU6050->KalmanUncertaintyAngleRoll=(1-KalmanGain0)*MPU6050->KalmanUncertaintyAngleRoll;

	MPU6050->KalmanAnglePitch = MPU6050->KalmanAnglePitch +dt * MPU6050->Gy;
	MPU6050->KalmanUncertaintyAnglePitch= MPU6050->KalmanUncertaintyAnglePitch+dt*dt*3*3;
	float KalmanGain1= MPU6050->KalmanUncertaintyAnglePitch*1/(1* MPU6050->KalmanUncertaintyAnglePitch+4*4);
	MPU6050->KalmanAnglePitch=MPU6050->KalmanAnglePitch+KalmanGain1*( MPU6050->Angle_Pitch-MPU6050->KalmanAnglePitch);
	MPU6050->KalmanUncertaintyAnglePitch=(1-KalmanGain1)*MPU6050->KalmanUncertaintyAnglePitch;
	//printf("KalmanAngleRoll = %f  , Gx apres  = %f , KalmanGain = %f , KalmanUncertaintyAngleRoll  = %f \n\r",MPU6050->KalmanAngleRoll,MPU6050->Gx, KalmanGain ,MPU6050->KalmanUncertaintyAngleRoll );
}

/*void Kalman_filter_pitch(MPU6050_t *MPU6050){

	double dt = (double)(HAL_GetTick() - timer) / 1000;
	timer = HAL_GetTick();
	MPU6050->KalmanAnglePitch = MPU6050->KalmanAnglePitch +dt * MPU6050->Gy;
	MPU6050->KalmanUncertaintyAnglePitch= MPU6050->KalmanUncertaintyAnglePitch+dt*dt*4*4;
	float KalmanGain= MPU6050->KalmanUncertaintyAnglePitch*1/(1* MPU6050->KalmanUncertaintyAnglePitch+3*3);
	MPU6050->KalmanAnglePitch=MPU6050->KalmanAnglePitch+KalmanGain*( MPU6050->Angle_Pitch-MPU6050->KalmanAnglePitch);
	MPU6050->KalmanUncertaintyAnglePitch=(1-KalmanGain)*MPU6050->KalmanUncertaintyAnglePitch;
	//printf("KalmanAnglePitch = %f  , Gy  = %f , KalmanGain = %f , KalmanUncertaintyAnglePitch  = %f \n\r",MPU6050->KalmanAnglePitch,MPU6050->Gy, KalmanGain ,MPU6050->KalmanUncertaintyAnglePitch );
}*/




