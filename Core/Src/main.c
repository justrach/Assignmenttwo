///******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  * (c) EE2028 Teaching Team
//  ******************************************************************************/
//
//
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
//#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
//#include "stdio.h"
////
////extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)
////static void MX_GPIO_Init(void);
//
//int count;
//int flag;
//uint32_t tickstart;
//uint32_t tickstart2;
//uint32_t tickstart3;
////void temperatureChecker(int x1,int y1){
////	if(x1 > 37){
////
////	}
////}
//static void MX_GPIO_Init(void);
//extern void initialise_monitor_handles(void);
//void SystemClock_Config(void);
//
//
////HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
////{
////	if(GPIO_Pin == BUTTON_EXTI13_Pin)
////	{
////		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
////		printf("\t Blue button is pressed. \n");
////	}
////}
//
////HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
////{
////	if(GPIO_Pin == BUTTON_EXTI13_Pin)
////	{
////		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
////				printf("\t normal mode operation n progress. \n");
////		if (count ==1){
////			count ++;
////		}
////		else if(count ==2) {
////			tickstart = HAL_GetTick();
////			count++;
////		} else if(count ==3){
////			tickstart2 = HAL_GetTick();
////			count = 1;
////		    tickstart3= (tickstart2 - tickstart);
////
////			printf("The value of the third ticker is %d\n\n\n",tickstart3);
////		}
////
////		if(flag == 1){
////			flag =0;
////		}
////		else if((flag ==0) && ((tickstart2 - tickstart)<50)){
////			flag = 1;
////
////		}
////
////	}
////}
//
//
//int main(void)
//{
//	//creating delays using another method.
//	initialise_monitor_handles(); // for semi-hosting support (printf)
//
//	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//	HAL_Init();
//
//	//initialise the HAL
//int currentticktimer;
//	/* Peripheral initializations using BSP functions */
//		// read temperature sensor
//	printf("Initialise");
//
////int normalMode = 1;
////int ICUmode = 0;
//
//int main(void)
//{
//	initialise_monitor_handles();
//	HAL_Init();
//	MX_GPIO_Init();
//	BSP_ACCELERO_Init();
//	BSP_TSENSOR_Init();
//
//	while (1)
//	{
//
//
//			float accel_data[3];
//			int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
//			BSP_ACCELERO_AccGetXYZ(accel_data_i16);
//			float temp_data;// read temp
//			temp_data = temperatureMeasure();
//			printf("Temperature data is %d", temp_data);
//
//						 // the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
//			accel_data[0] = (float)accel_data_i16[0] / 100.0f;
//			accel_data[1] = (float)accel_data_i16[1] / 100.0f;
//			accel_data[2] = (float)accel_data_i16[2] / 100.0f;
//			printf("Accel X : %f; Accel Y : %f; Accel Z : %f; Temperature : %f\n", accel_data[0], accel_data[1], accel_data[2], temp_data);
//
//
//
//						 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
//
//	}
//
//}
//
//
//
//
//
//		//create and start running the main program
//
//
////		HAL_Delay(1000);	// read once a ~second. // CANNOT USE HAL_DELAY anymore!
//
//	}
//
//
//
//
//
//
//
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
////  __HAL_RCC_GPIOB_CLK_ENABLE();
//  	__HAL_RCC_GPIOB_CLK_ENABLE();	// Enable AHB2 Bus for GPIOB
//  	__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
//
//
//  /*Configure GPIO pin LED2_Pin */
//  GPIO_InitStruct.Pin = LED2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//
//  	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
//  	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
//  	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  	GPIO_InitStruct.Pull = GPIO_NOPULL;
//  	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /* Enable interrupt controller*/
//  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
//
//}



/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "stdio.h"

static void MX_GPIO_Init(void);
extern void initialise_monitor_handles(void);
void SystemClock_Config(void);

//to prevent implicit declarations for the C program + make it run smooth like butter, like a criminal undercover.///
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void BSP_HSENSOR_Init();
void BSP_PSENSOR_Init();

int flag =1;
int count = 1;
uint32_t tickstart;
uint32_t tickstart2;
uint32_t tickstart3;


//void BSP_HSENSOR_Init();

float doIHaveFever(float temp1){\

	if(temp1>37.5){
		printf("Fever is detected\n");
	}

}

float Respiratory(float humidity, float pressure){

	if((humidity < 95) || (pressure < 95)){
		printf("Check patient's breath! \n");
	}
}

HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BUTTON_EXTI13_Pin)
	{
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				printf("\t normal mode operation in progress. \n");
		if (count ==1){
			count ++;
		}
		else if(count ==2) {
			tickstart = HAL_GetTick();
			count++;
		} else if(count ==3){
			tickstart2 = HAL_GetTick();
			count = 1;
		    tickstart3= (tickstart2 - tickstart);

			printf("The value of the third ticker is %d\n\n\n",tickstart3);
		}

		if(flag == 1){
			flag =0;
		}
		else if((flag ==0) && ((tickstart2 - tickstart)<50)){
			flag = 1;

		}

	}
}

/*float temperatureMeasure(){
	BSP_TSENSOR_Init();
	return BSP_TSENSOR_ReadTemp();
}
*/

int main(void)
{




	initialise_monitor_handles();
	HAL_Init();
	MX_GPIO_Init();
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_TSENSOR_Init();
	BSP_MAGNETO_Init();

	//logic flow such that when respiratory sens




	while (1)
	{
		if(flag ==0 ){
			//BSP_TSENSOR_Init();

			//Acceleration is located below this line
			float accel_data[3];
			int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
			BSP_ACCELERO_AccGetXYZ(accel_data_i16);
			accel_data[0] = (float)accel_data_i16[0] / 100.0f;
			accel_data[1] = (float)accel_data_i16[1] / 100.0f;
			accel_data[2] = (float)accel_data_i16[2] / 100.0f;
			//////Acceleration Declaration End/////



//Declaration of the temperature and other variables which are
			float temp_data;// read temp
			float humidity_data;
			float pressure_data;

//Magnetometer and the current details for the items would be declared below this line
			float magnetometer_data[3];
			int16_t magneto_int16[3] = { 0 };
			BSP_MAGNETO_GetXYZ(magneto_int16);
			magnetometer_data[0] = (float)magneto_int16[0] / 100.0f;
			magnetometer_data[1] = (float)magneto_int16[1] / 100.0f;
			magnetometer_data[2] = (float)magneto_int16[2] / 100.0f;


			//gyroscope data is located below this line
			float gysoscope_data[3];
			int16_t gyro_int16[3] = { 0 };
		    BSP_GYRO_GetXYZ(gyro_int16);
			gysoscope_data[0] = (float)gyro_int16[0] / 100.0f;
			gysoscope_data[1] = (float)gyro_int16[1] / 100.0f;
			gysoscope_data[2] = (float)gyro_int16[2] / 100.0f;


			temp_data = BSP_TSENSOR_ReadTemp();
			humidity_data = BSP_HSENSOR_ReadHumidity();
			pressure_data = BSP_PSENSOR_ReadPressure();
		//	printf("Temperature data is %f\n", temp_data);
			printf("Humidity data is %f\n", humidity_data);
//			printf("Pressure data is %f\n", pressure_data);
		//	doIHaveFever(temp_data);
		//	Respiratory(humidity_data, pressure_data);
		// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.

			printf("Accel X : %f; Accel Y : %f; Accel Z : %f; Temperature : %f\n\n\n", accel_data[0], accel_data[1], accel_data[2], temp_data);
			printf("Gyro X : %f; Gyro Y : %f; Gyro Z : %f; Pressure : %f\n", gysoscope_data[0], gysoscope_data[1], gysoscope_data[2], pressure_data);
			printf("Magnetometer1 X : %f; Magnetometer2 Y : %f; Magnetometer3 Z : %f; Humidity : %f\n", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2], humidity_data);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		}
	}

}

static void MX_GPIO_Init(void)
{
	  /* GPIO Ports Clock Enable */

		__HAL_RCC_GPIOB_CLK_ENABLE();	// Enable AHB2 Bus for GPIOB
		__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC
		__HAL_RCC_GPIOD_CLK_ENABLE();	// Enable Temperature and pressure sensor GPIOD

	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET); // Reset the LED2_Pin as 0

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Configuration of LED2_Pin (GPIO-B Pin-14) as GPIO output
	GPIO_InitStruct.Pin = LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/*	// Configuration of LPS22HB_Pin (GPIO-D Pin-10) as GPIO output
	GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); */

	// Enable NVIC EXTI line 13
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);






	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);


}

