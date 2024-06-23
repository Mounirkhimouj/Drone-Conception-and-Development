/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc_controller.h"
#include "pid.h"
#include <stdio.h>
#include "gps.h"
#include "BME280_STM32.h"
#include "mpu6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

GPS_t GPS;
extern uint8_t USART1_rx_flag;
extern uint8_t USART1_rx_data;
extern uint8_t _rx_buf[52];
extern uint8_t _rx_cplt_flag;
int32_t gps_lat_dec,gps_lon_dec,alt_dec,bat_dec;
char gps_lat_round,gps_lon_round,alt_dec_round,bat_dec_round;
uint8_t data_to_pc[46];

float Temperature, Pressure, Humidity,alt,OrrX_pre,OrrY_pre,OrrZ_pre;

float R1 = 30000.0;
float R2 = 7500.0;
extern uint8_t tim10_1ms_flag ;
unsigned char low_bat_flag = 0;
#define MIN 55
#define MAX 65


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void DMA_TransmitData(void);
int IS_throttle_Min(void);
void ESC_Calibration(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart6) GPS_UART_CallBack();
}

long map (long x, long in_min, long in_max, long out_min, long out_max)
{
return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// PID controller variables
float integral_alt = 0.0;
float previous_error_alt = 0.0;

// Desired altitude
//float desired_altitude = 4;  // Example: 100 meters

// PID controller function
float pidController_alt(float setpoint, float measured_value) {
    float error = setpoint - measured_value;
    integral_alt += error;
    float derivative = error - previous_error_alt;
    previous_error_alt = error;
    return (altitude_heading.kp * error) + (altitude_heading.ki * integral_alt) + (altitude_heading.kd * derivative);
}

// PID controller variables
float integral_yaw = 0.0;
float previous_error_yaw = 0.0;

// Desired altitude
//float desired_altitude = 4;  // Example: 100 meters

// PID controller function
float pidController_yaw(float setpoint, float measured_value) {
    float error = setpoint - measured_value;
    integral_yaw += error;
    float derivative = error - previous_error_yaw;
    previous_error_yaw = error;
    return (yaw_heading.kp * error) + (yaw_heading.ki * integral_yaw) + (yaw_heading.kd * derivative);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  unsigned short ccr1, ccr2, ccr3, ccr4;
  unsigned short adcVal;
  float batVolt, adcVolt;
  float yaw_heading_reference;
  float alt_heading_reference,alt_heading_reference_init;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  GPS_Init();
  // Initialize the RFFT instance
  LL_USART_EnableIT_RXNE(USART1);
  LL_TIM_EnableCounter(TIM4);

  //BMP8CONFIG
  BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_20, IIR_16);
  //MPU6050
  //MPU6050SetDefault(&MPU6050_DOWN);
  //while (MPU6050_Init(&hi2c1,MPU6050_ADDR_0) == 1);

  HAL_ADC_Start_DMA(&hadc1, &adcVal, 1);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  ESC_Calibration();

  //while(IS_throttle_Min() == 0);


  // we should turn these sequence of code into a function
  LL_TIM_CC_EnableChannel(TIM4,LL_TIM_CHANNEL_CH4);

  TIM4->PSC = 2000;
  HAL_Delay(100);
  TIM4->PSC = 1500;
  HAL_Delay(100);
  TIM4->PSC = 1000;
  HAL_Delay(100);

  LL_TIM_CC_DisableChannel(TIM4,LL_TIM_CHANNEL_CH4);



	//BMP280
	BME280_Measure();
	alt_heading_reference_init = alt;

	LL_TIM_EnableCounter(TIM10);
	LL_TIM_EnableIT_UPDATE(TIM10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //MPU6050
		//MPU6050_Read(&hi2c1, &MPU6050_DOWN,MPU6050_ADDR_0);

		//MPU6050_DOWN.KalmanAngleRoll = 0.1 * MPU6050_DOWN.KalmanAngleRoll + KalmanAngleRollPrev * 0.9;
		//KalmanAngleRollPrev = MPU6050_DOWN.KalmanAngleRoll;

		//MPU6050_DOWN.KalmanAnglePitch = 0.1 * MPU6050_DOWN.KalmanAnglePitch + KalmanAnglePitchPrev * 0.9;
		//KalmanAnglePitchPrev = MPU6050_DOWN.KalmanAnglePitch;

		//printf("%.4f %.4f\r\n",MPU6050_DOWN.KalmanAngleRoll,MPU6050_DOWN.KalmanAnglePitch);

		//MPU6050_DOWN.KalmanAngleRoll = filter_roll(MPU6050_DOWN.KalmanAngleRoll);
		//MPU6050_DOWN.KalmanAnglePitch = filter_pitch(MPU6050_DOWN.KalmanAnglePitch) ;


		//printf("%.4f %.4f\r\n",MPU6050_DOWN.KalmanAngleRoll,MPU6050_DOWN.KalmanAnglePitch);

		//float KalmanAngleRoll_map = map_float(MPU6050_DOWN.KalmanAngleRoll,-4.5,4.0,-90,90)   ;
		//float KalmanAnglePitch_map = map_float(MPU6050_DOWN.KalmanAnglePitch,-4.5,4.5,-90,90) ;


		//BMP280
		BME280_Measure();
		alt = alt - alt_heading_reference_init;
		alt = alt < 0 ? 0 : alt;

		adcVolt = adcVal * 0.000806f;
		batVolt = adcVolt * (R1 + R2) / R2;

		gps_lat_dec = GPS.dec_latitude * 10000;
		gps_lon_dec = GPS.dec_longitude * 10000;
		alt_dec = alt * 100;
		bat_dec = batVolt * 100;

		if(batVolt < 10.0f)
		  {
			  low_bat_flag = 1;
		  }
		  else
		  {
			  low_bat_flag = 0;
		  }


	  		if(_rx_cplt_flag == 1)
			  {
	  			printf("%s\r\n",_rx_buf);
				printf("%d\r\n",strlen(_rx_buf));
				  _rx_cplt_flag = 0;
				  if(strlen(_rx_buf) == 52 && _rx_buf[0] == '#')
				  {
					  //printf("%d\r\n",strlen(_rx_buf));

					  //printf("recep comp\r\n");

					  parseData(_rx_buf);

					  if(imu.OrrX>360.0){
						  imu.OrrX = OrrX_pre;
					  }
					  OrrX_pre = imu.OrrX;
					  if(imu.OrrY>360.0){
						  imu.OrrY = OrrY_pre;
					  }
					  OrrY_pre = imu.OrrY;
					  if(imu.OrrZ>360.0){
						  imu.OrrZ = OrrZ_pre;
					  }
					  OrrZ_pre = imu.OrrZ;


					  printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",imu.OrrX,imu.OrrY,imu.OrrZ,imu.GyroX,imu.GyroY,imu.GyroZ,imu.LACCx,imu.LACCy,imu.LACCz);
					  printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", rc.RH, rc.RV, rc.LV, rc.LH, rc.SwL, rc.SwR);

					  	  /*

						  if(tim10_1ms_flag == 1)
						  {
							tim10_1ms_flag == 0;
							printf("pid\r\n");

							Double_Roll_Pitch_PID_Calculation(&pitch, (rc.RV - 510) * 0.1f, imu.OrrZ, imu.GyroZ);
							Double_Roll_Pitch_PID_Calculation(&roll, (rc.RH - 500) * 0.1f, imu.OrrY,imu.GyroY);

							if(alt < 0.03) // we still need to change the values
							{
							  Reset_All_PID_Integrator();
							}

							// we start controlling the height ()
							if((rc.LV < 470 || rc.LV > 550)) // we still need to change the values
							{
							  alt_heading_reference = alt;
							  Single_Alt_Rate_PID_Calculation(&altitude_rate, (rc.LV - 512), imu.LACCx);

							  if((rc.LH < 500 || rc.LH > 530)) // we still need to change the values
							  {
								  yaw_heading_reference = imu.OrrX;
								  printf("alt acc yaw rate\r\n");
								  Single_Yaw_Rate_PID_Calculation(&yaw_rate, (rc.LH - 512), imu.GyroX);
								  ccr1 = MIN + altitude_rate.pid_result - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result;
								  ccr2 = MIN + altitude_rate.pid_result - pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result;
								  ccr3 = MIN + altitude_rate.pid_result + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result;
								  ccr4 = MIN + altitude_rate.pid_result + pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result;
							  }
							  else{
								  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, imu.OrrX,imu.GyroX);
								  printf("alt acc yaw heading\r\n");
								  ccr1 = MIN + altitude_rate.pid_result - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result;
								  ccr2 = MIN + altitude_rate.pid_result - pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result;
								  ccr3 = MIN + altitude_rate.pid_result + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result;
								  ccr4 = MIN + altitude_rate.pid_result + pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result;
							  }
							}
							else
							{
							  Single_Alt_Heading_PID_Calculation(&altitude_heading, alt_heading_reference, alt, imu.LACCx);


							  if((rc.LH < 500 || rc.LH > 530)) // we still need to change the values
							  {
								  yaw_heading_reference = imu.OrrX;
								  printf("alt heading Yaw rate\r\n");
								  Single_Yaw_Rate_PID_Calculation(&yaw_rate, (rc.LH - 512), imu.GyroX);
								  ccr1 = MIN + altitude_heading.pid_result - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result;
								  ccr2 = MIN + altitude_heading.pid_result - pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result;
								  ccr3 = MIN + altitude_heading.pid_result + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result;
								  ccr4 = MIN + altitude_heading.pid_result + pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result;
							  }
							  else{
								  printf("alt heading Yaw heading\r\n");
								  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, imu.OrrX,imu.GyroX);
								  ccr1 = MIN + altitude_heading.pid_result - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result;
								  ccr2 = MIN + altitude_heading.pid_result - pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result;
								  ccr3 = MIN + altitude_heading.pid_result + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result;
								  ccr4 = MIN + altitude_heading.pid_result + pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result;
							  }
							}
							}
							*/

				  	  }
			  }

	  			float mapped_alt= map_float(rc.LV, 10, 1023, 0,3000);
	  			printf("rc.LV: %f\r\n",rc.LV);
	  			alt_heading_reference = map_float(mapped_alt, 2, 3000, 0,4);
				printf("alt_heading_reference: %f\r\n",alt_heading_reference);
				altitude_heading.kp = 1;
				altitude_heading.ki = 0;
				altitude_heading.kd = 0;

				yaw_heading.kp = 0.3;
				yaw_heading.ki = 4;
				yaw_heading.kd = 0;


	  			float Ut_alt = pidController_alt(1,alt);
	  			//float Ut_yaw = pidController_yaw(alt_heading_reference,alt);

	  			pitch.in.kp = 1.5;
				pitch.in.ki = 1.5;
				pitch.in.kd = 0;
				pitch.out.kp = 2;
				pitch.out.ki = 2;
				pitch.out.kd = 0;


				roll.in.kp = 1.5;
				roll.in.ki = 1.5;
				roll.in.kd = 0;
				roll.out.kp = 2;
				roll.out.ki = 2;
				roll.out.kd = 0;

	  			Double_Roll_Pitch_PID_Calculation(&pitch, 0 , imu.OrrZ, imu.GyroZ);
				Double_Roll_Pitch_PID_Calculation(&roll, 0, imu.OrrY,imu.GyroY);
				Single_Yaw_Heading_PID_Calculation(&yaw_heading, 246, imu.OrrX,imu.GyroX);
				ccr1 = MIN /* + Ut_alt  -  pitch.in.pid_result  + roll.in.pid_result */ + yaw_heading.pid_result; // M1
				ccr2 = MIN /*+   Ut_alt - pitch.in.pid_result  - roll.in.pid_result */ - yaw_heading.pid_result;  // M2
				ccr3 = MIN /*+  Ut_alt + pitch.in.pid_result  - roll.in.pid_result */ + yaw_heading.pid_result;	 // M3
				ccr4 = MIN /*+ Ut_alt + pitch.in.pid_result + roll.in.pid_result */ -  yaw_heading.pid_result;  // M4

				printf("yaw_heading.pid_result: %f\r\n ",yaw_heading.pid_result);
				printf("CCR1= %d, CCR2=  %d, CCR3=  %d ,CCR4=  %d\r\n ",ccr1,ccr2,ccr3,ccr4);



	  		ccr1 = ccr1 > MAX ? MAX : ccr1 < MIN ? MIN : ccr1;
	  		ccr2 = ccr2 > MAX ? MAX : ccr2 < MIN ? MIN : ccr2;
	  		ccr3 = ccr3 > MAX ? MAX : ccr3 < MIN ? MIN : ccr3;
	  		ccr4 = ccr4 > MAX ? MAX : ccr4 < MIN ? MIN : ccr4;

		    TIM3->CCR1 = 65;
		    TIM3->CCR2 = 65;
		    TIM3->CCR3 = 65;
		    TIM3->CCR4 = 65;

	  	sprintf(data_to_pc,"0003,0000,0000,0000,%4d,%5d,%07d,%07d,",bat_dec,alt,gps_lat_dec,gps_lon_dec);
	  	//printf("long= %.4f ,lat=  %.4f\r\n ",GPS.dec_longitude,GPS.dec_latitude);
	  	printf("alt: %f, Pres: %f, Temp: %f, Hum: %f \r\n ",alt, Pressure, Temperature, Humidity);
	  	//printf("%d\t%.5f\r\n", adcVal, batVolt);
		//printf("ALT_GAINS : %f\t%f\t%f\t%f\t%f\t%f\r\n", altitude_heading.kp, altitude_heading.ki, altitude_heading.kd, altitude_rate.kp, altitude_rate.ki, altitude_rate.kd);
		//printf("YAW_GAINS : %f\t%f\t%f\t%f\t%f\t%f\r\n", yaw_heading.kp, yaw_heading.ki, yaw_heading.kd, yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
		printf("ROLL_GAINS : %f\t%f\t%f\t%f\t%f\t%f\r\n", roll.in.kp, roll.in.ki, roll.in.kd, roll.out.kp, roll.out.ki, roll.out.kd);
		printf("PITCH_GAINS : %f\t%f\t%f\t%f\t%f\t%f\r\n", pitch.in.kp, pitch.in.ki, pitch.in.kd, pitch.out.kp, pitch.out.ki, pitch.out.kd);
		printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", rc.RH, rc.RV, rc.LV, rc.LH, rc.SwL, rc.SwR);
		 printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",imu.OrrX,imu.OrrY,imu.OrrZ,imu.GyroX,imu.GyroY,imu.GyroZ,imu.LACCx,imu.LACCy,imu.LACCz);


	  	//printf("alt= %f\r\n ",alt);
	  	//printf("%s\r\n",data_to_pc);

	  	DMA_TransmitData();
	  	HAL_Delay(30);
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1680;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 20;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM4);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 10;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM4, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM4 GPIO Configuration
  PB9   ------> TIM4_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM10);

  /* TIM10 interrupt Init */
  NVIC_SetPriority(TIM1_UP_TIM10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  TIM_InitStruct.Prescaler = 1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 41999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM10, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM10);
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 DMA Init */

  /* USART1_TX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_7, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_7, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_7);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
  LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_7, (uint32_t)&data_to_pc, LL_USART_DMA_GetRegAddr(USART1), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  // Enable transmit completion interrupt.
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	//int DataIdx;

	HAL_UART_Transmit(&huart2,(uint8_t*)ptr,len,HAL_MAX_DELAY);
	/*
	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	*/
	return len;
}

void DMA_TransmitData()
{

  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);

  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, sizeof(data_to_pc));

  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);

  LL_USART_EnableDMAReq_TX(USART1);
}


void ESC_Calibration(void)
{
	  TIM3->CCR1 = 100;
	  TIM3->CCR2 = 100;
	  TIM3->CCR3 = 100;
	  TIM3->CCR4 = 100;
	  HAL_Delay(3000);
	  TIM3->CCR1 = 50;
	  TIM3->CCR2 = 50;
	  TIM3->CCR3 = 50;
	  TIM3->CCR4 = 50;
	  HAL_Delay (2000);
}

/*int Is_data_Received(void)
{
	if(_rx_cplt_flag == 1)
	  {
	  _rx_cplt_flag = 0;
	  parseData(_rx_buf);
	   return 1 ;
	  }
    return 0 ;
}*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
