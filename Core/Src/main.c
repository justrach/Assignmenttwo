/******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "stdio.h"
//
//extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)
//static void MX_GPIO_Init(void);

int count;
int flag;
uint32_t tickstart;
uint32_t tickstart2;
uint32_t tickstart3;
//void temperatureChecker(int x1,int y1){
//	if(x1 > 37){
//
//	}
//}
static void MX_GPIO_Init(void);
extern void initialise_monitor_handles(void);
void SystemClock_Config(void);


//HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == BUTTON_EXTI13_Pin)
//	{
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
//		printf("\t Blue button is pressed. \n");
//	}
//}

//HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == BUTTON_EXTI13_Pin)
//	{
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
//				printf("\t normal mode operation n progress. \n");
//		if (count ==1){
//			count ++;
//		}
//		else if(count ==2) {
//			tickstart = HAL_GetTick();
//			count++;
//		} else if(count ==3){
//			tickstart2 = HAL_GetTick();
//			count = 1;
//		    tickstart3= (tickstart2 - tickstart);
//
//			printf("The value of the third ticker is %d\n\n\n",tickstart3);
//		}
//
//		if(flag == 1){
//			flag =0;
//		}
//		else if((flag ==0) && ((tickstart2 - tickstart)<50)){
//			flag = 1;
//
//		}
//
//	}
//}


int main(void)
{
	//creating delays using another method.
	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	//initialise the HAL
int currentticktimer;
	/* Peripheral initializations using BSP functions */
		// read temperature sensor
	printf("Initialise");

//int normalMode = 1;
//int ICUmode = 0;

int main(void)
{
	initialise_monitor_handles();
	HAL_Init();
	MX_GPIO_Init();
	BSP_ACCELERO_Init();
	BSP_TSENSOR_Init();

	while (1)
	{


			float accel_data[3];
			int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
			BSP_ACCELERO_AccGetXYZ(accel_data_i16);
			float temp_data;// read temp
			temp_data = temperatureMeasure();
			printf("Temperature data is %d", temp_data);

						 // the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
			accel_data[0] = (float)accel_data_i16[0] / 100.0f;
			accel_data[1] = (float)accel_data_i16[1] / 100.0f;
			accel_data[2] = (float)accel_data_i16[2] / 100.0f;
			printf("Accel X : %f; Accel Y : %f; Accel Z : %f; Temperature : %f\n", accel_data[0], accel_data[1], accel_data[2], temp_data);



						 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

	}

}





		//create and start running the main program


//		HAL_Delay(1000);	// read once a ~second. // CANNOT USE HAL_DELAY anymore!

	}







static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOB_CLK_ENABLE();
  	__HAL_RCC_GPIOB_CLK_ENABLE();	// Enable AHB2 Bus for GPIOB
  	__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);


  /*Configure GPIO pin LED2_Pin */
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

  /* Enable interrupt controller*/
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}



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

//#include "main.h"
//
//static void MX_GPIO_Init(void);
//extern void initialise_monitor_handles(void);
//void SystemClock_Config(void);
//
//
//int flag =1;
//int count = 1;
//uint32_t tickstart;
//uint32_t tickstart2;
//uint32_t tickstart3;
//
//
//
//
//int doIHaveFever(int temp1){\
//
//	if(temp1>37.5){
//		printf("Fever is detected\n");
//	}
//
//}
//
//
//HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == BUTTON_EXTI13_Pin)
//	{
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
//				printf("\t normal mode operation n progress. \n");
//		if (count ==1){
//			count ++;
//		}
//		else if(count ==2) {
//			tickstart = HAL_GetTick();
//			count++;
//		} else if(count ==3){
//			tickstart2 = HAL_GetTick();
//			count = 1;
//		    tickstart3= (tickstart2 - tickstart);
//
//			printf("The value of the third ticker is %d\n\n\n",tickstart3);
//		}
//
//		if(flag == 1){
//			flag =0;
//		}
//		else if((flag ==0) && ((tickstart2 - tickstart)<50)){
//			flag = 1;
//
//		}
//
//	}
//}
//
//float temperatureMeasure(){
//	BSP_TSENSOR_Init();
//	return BSP_TSENSOR_ReadTemp();
//}
//
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
//		if(flag ==0 ){
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
//		}
//	}
//
//}
//
//static void MX_GPIO_Init(void)
//{
//	  /* GPIO Ports Clock Enable */
//
//		__HAL_RCC_GPIOB_CLK_ENABLE();	// Enable AHB2 Bus for GPIOB
//		__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC
//
//	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET); // Reset the LED2_Pin as 0
//
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//	//  /* GPIO Ports Clock Enable */
//	  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//	// Configuration of LED2_Pin (GPIO-B Pin-14) as GPIO output
//	GPIO_InitStruct.Pin = LED2_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
//	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//	// Enable NVIC EXTI line 13
//	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
//
//
//
//
//
//	  /*Configure GPIO pin Output Level */
//	  HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
//
//
//}

