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

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)
static void MX_GPIO_Init(void);

int count;
int flag;
uint32_t tickstart;
uint32_t tickstart2;

void temperatureChecker(int x1,int y1){
	if(x1 > 37){

	}
}


HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BUTTON_EXTI13_Pin)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				printf("\t Blue button is pressed. \n");
		if (count ==1){
			count ++;
		}
		else if(count ==2) {
			tickstart = HAL_GetTick();
			count++;
		} else if(count ==3){
			tickstart2 = HAL_GetTick();
			count = 1;
		}

		if(flag == 1){
			flag =0;
		}
		else if((flag ==0) && ((tickstart2 - tickstart)<5000)){
			flag = 1;
		}

	}
}


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

	while (1)




	{
//		while (normalMode ==1 && ICUmode == 0){
//			float temp_data;// read temp
//			temp_data = BSP_TSENSOR_ReadTemp();
//
//			if(temp_data > 37.5){
//
//			}
//
//
//
//
//		}

		if(flag ==0){
		 MX_GPIO_Init();
			BSP_ACCELERO_Init();
			BSP_TSENSOR_Init();
			float accel_data[3];
					int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
					BSP_ACCELERO_AccGetXYZ(accel_data_i16);
					float temp_data;// read temp
					temp_data = BSP_TSENSOR_ReadTemp();
		 for(int x =0 ; x<1000; x++){

			 if(x ==250){

				 // the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
				 		accel_data[0] = (float)accel_data_i16[0] / 100.0f;
				 		accel_data[1] = (float)accel_data_i16[1] / 100.0f;
				 		accel_data[2] = (float)accel_data_i16[2] / 100.0f;
				 		printf("Accel X : %f; Accel Y : %f; Accel Z : %f; Temperature : %f\n", accel_data[0], accel_data[1], accel_data[2], temp_data);
				 		x++;
						 printf("%d\n",x);
						 break;
			 }
			 else if(x == 500){
				 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			 }
			 else if(x == 750){
				 currentticktimer = HAL_GetTick();
				 		printf("The current timing is : %d\n", currentticktimer );
			 }
			 else if(x>750){
				 x =0;
			 }
		 }

		}





		//create and start running the main program


//		HAL_Delay(1000);	// read once a ~second. // CANNOT USE HAL_DELAY anymore!

	}




}





static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

