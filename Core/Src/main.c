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
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "stdio.h"
#include "math.h"
#include "wifi.h"
static void MX_GPIO_Init(void);
//extern void initialise_monitor_handles(void);
void SystemClock_Config(void);

//to prevent implicit declarations for the C program + make it run smooth like butter, like a criminal undercover.///
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
//void BSP_HSENSOR_Init();
//void BSP_PSENSOR_Init();

UART_HandleTypeDef huart1; // struct for uart1
char message_print[50]; //array for uart transmission

#define HEALTHY 0
#define INTENSIVE 1

//Define the thresholds here

#define GYRO_THRESHOLD 200.0
#define TEMP_THRESHOLD 37.6
#define MAG_THRESHOLD 1
#define HUM_THRESHOLD 60.0
#define ACC_THRESHOLD 13
#define P_THRESHOLD 102020
#define WARNING 1
#define SAFE 0
// Threshold end

int mode = HEALTHY;
//wifi definition

#define MAX_LENGTH 400  // adjust it depending on the max size of the packet you expect to send or receive
#define WIFI_READ_TIMEOUT 10000
#define WIFI_WRITE_TIMEOUT 10000

//wifi definiton end


const char* WiFi_SSID = "seth";               // Replacce mySSID with WiFi SSID for your router / Hotspot
const char* WiFi_password = "nmoc1234";   // Replace myPassword with WiFi password for your router / Hotspot
const WIFI_Ecn_t WiFi_security = WIFI_ECN_WPA2_PSK; // WiFi security your router / Hotspot. No need to change it unless you use something other than WPA2 PSK
const uint16_t SOURCE_PORT = 1234;  // source port, which can be almost any 16 bit number

uint8_t ipaddr[4] = {192, 168, 43, 134}; // IP address of your laptop wireless lan adapter, which is the one you successfully used to test Packet Sender above.
                                    // If using IoT platform, this will be overwritten by DNS lookup, so the values of x and y doesn't matter
                                            //(it should still be filled in with numbers 0-255 to avoid compilation errors)

#ifdef USING_IOT_SERVER
    const char* SERVER_NAME = "192.168.43.134";    // domain name of the IoT server used
    const uint16_t DEST_PORT = 3000;          // 'server' port number. Change according to application layer protocol. 80 is the destination port for  protocol.
#else
    const uint16_t DEST_PORT = 3000;        // 'server' port number - this is the port Packet Sender listens to (as you set in Packer Sender)
                                                // and should be allowed by the OS firewall
#endif
SPI_HandleTypeDef hspi3;








//int flag =1;
int count = 1;
uint32_t tickstart;
uint32_t tickstart2;
uint32_t tickstart3;

//Flag defs

int accflag, gyroflag, pressureflag, tempflag, magflag, humidflag;
void ResetFlag(void);
//void BSP_HSENSOR_Init();

//create the uART

//float doIHaveFever(float temp1){\
//
//	if(temp1>TEMP_THRESHOLD){
//		printf("Fever is detected\n");
//	}
//
//}
//
//float Respiratory(float humidity, float pressure){
//
//	if((humidity < HUM_THRESHOLD) || (pressure < MAG_THRESHOLD)){
//		printf("Check patient's breath! \n");
//	}
//}

HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == BUTTON_EXTI13_Pin) {
		//		count++;
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		//printf("\t normal mode operation in progress. \n");
		if (count == 1 && mode == INTENSIVE) {
			tickstart = HAL_GetTick();
			count = 2;
		}
		if (count == 2 && mode == INTENSIVE) {
			tickstart2 = HAL_GetTick();
			if((tickstart2 - tickstart) < 500){
				mode = HEALTHY;
				count = 1;
				sprintf(message_print, "Entering Normal Mode.\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
				ResetFlag();
			}
		}
		//		if (count == 3 && mode == INTENSIVE) {
		//			tickstart2 = HAL_GetTick();
		//			count = 1;
		//			sprintf(message_print,"coount is %d \n", count);
		//						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
		//			tickstart3 = (tickstart2 - tickstart);
		//
		//			//	printf("The value of the third ticker is %d\n\n\n",tickstart3);
		//		}

		else if(mode == INTENSIVE) {
			mode = INTENSIVE;
			sprintf(message_print, "Entering Intensive Mode.\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			count = 1;
			ResetFlag();
		}



	}
	if (GPIO_Pin == GPIO_PIN_1){
		SPI_WIFI_ISR();
	}

}




/*float temperatureMeasure(){
 BSP_TSENSOR_Init();
 return BSP_TSENSOR_ReadTemp();
 }
 */
static void UART1_Init(void);
int main(void) {
	char wifi_message[50];
	//	initialise_monitor_handles();
	HAL_Init();
	MX_GPIO_Init();
	BSP_ACCELERO_Init();
	BSP_TSENSOR_Init();
	BSP_GYRO_Init();
	BSP_HSENSOR_Init();
	BSP_MAGNETO_Init();
	BSP_PSENSOR_Init();
	UART1_Init();
	float temp_data;
	float humidity_data;
	float pressure_data;
	float accel_data[3];
	int16_t accel_data_i16[3] = { 0 };// array to store the x, y and z readings.
	float Mag_Baseline[3], Mag_Difference[3];
	float  gyro_data[3], magneto_data[3];

	float gyro_data_i16[3] = { 0 };	//array to store gyro ODR data
	float gyro_total;
	int16_t magneto_data_i16[3] = { 0 };
	char json[100];

	int countBluff = 0;

	//WIFI initialisation
		  uint8_t req[MAX_LENGTH];  // request packet
		  uint8_t resp[MAX_LENGTH]; // response packet
		  uint16_t Datalen;
		  WIFI_Status_t WiFi_Stat; // WiFi status. Should remain WIFI_STATUS_OK if everything goes well

		  WiFi_Stat = WIFI_Init();                      // if it gets stuck here, you likely did not include EXTI1_IRQHandler() in stm32l4xx_it.c as mentioned above
		  WiFi_Stat &= WIFI_Connect(WiFi_SSID, WiFi_password, WiFi_security); // joining a WiFi network takes several seconds. Don't be too quick to judge that your program has 'hung' :)
		  if(WiFi_Stat!=WIFI_STATUS_OK) while(1);                   // halt computations if a WiFi connection could not be established.

		#ifdef USING_IOT_SERVER
		  WiFi_Stat = WIFI_GetHostAddress(SERVER_NAME, ipaddr); // DNS lookup to find the ip address, if using a connection to an IoT server
		#endif
		   WiFi_Stat = WIFI_Ping(ipaddr, 3, 200);                 // Optional ping 3 times in 200 ms intervals
		  WiFi_Stat = WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, DEST_PORT, SOURCE_PORT); // Make a TCP connection.
		                                                                  // "conn" is just a name and serves no functional purpose

//		  if(WiFi_Stat!=WIFI_STATUS_OK) while(1);                   // halt computations if a connection could not be established with the server


	while (1) {
		// this code here must be polled frequently @ 10 seconds
		// Have to create timers which poll at different speeds and times. How do you do this?
		if (mode == HEALTHY) {
			//temperatuyre is greater than 37.6 create a 5hz blinking led and realise that the fever is detected

			//init temperature data here
			temp_data = BSP_TSENSOR_ReadTemp();

			//accelerator data


			BSP_ACCELERO_AccGetXYZ(accel_data_i16);
			accel_data[0] = (float) accel_data_i16[0] / 100.0f;
			accel_data[1] = (float) accel_data_i16[1] / 100.0f;
			accel_data[2] = (float) accel_data_i16[2] / 100.0f;
			humidity_data = BSP_HSENSOR_ReadHumidity();
			pressure_data = BSP_PSENSOR_ReadPressure() * 100.0f; //pressure in pascal

			if (temp_data > TEMP_THRESHOLD) {
				sprintf(message_print, "\r\nFever is detected\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*) message_print,
						strlen(message_print), 0xFFFF);
//				WiFi_Stat = WIFI_SendData(1, wifi_message, (uint16_t)strlen((char*)wifi_message), &Datalen, WIFI_WRITE_TIMEOUT);

				//				mode = INTENSIVE;//do not enter intensive, only thru respitory
				if (HAL_GetTick() % 100 == 0) {
					HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
				} //insert LED code here
			} else {

//				sprintf(message_print,
//						"%03d TEMP %0.2f ACC %0.2f %0.2f %0.2f\r\n", count / 40,
//						temp_data, accel_data[0], accel_data[1], accel_data[2]);
//				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}

			if (accel_data[0] > ACC_THRESHOLD || accel_data[1] > ACC_THRESHOLD
					|| accel_data[2] > ACC_THRESHOLD) {
//				mode = INTENSIVE;
				//	int mytime;
				sprintf(message_print, "\r\nFall is detected\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*) message_print,
						strlen(message_print), 0xFFFF);

				//	mytime = HAL_GetTick();
				//				printf("\n\n\nticker is %d",mytime);
				if (HAL_GetTick() % 100 < 50) {
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				} //insert 5Hz code here

			}
			//			else {
			//				//				printf("Accel X : %f; Accel Y : %f; Accel Z : %f\n\n\n", accel_data[0], accel_data[1], accel_data[2]);
			//			}

			//if person is having acute respitory needs

			//humidity
			if (humidity_data > HUM_THRESHOLD || pressure_data > P_THRESHOLD) {
				sprintf(message_print, "\r\nCheck patient's breath! \n");
								HAL_UART_Transmit(&huart1, (uint8_t*) message_print,
										strlen(message_print), 0xFFFF);

				mode = INTENSIVE;
				if (HAL_GetTick() % 200 < 50) {
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				} //blink the LED 2 at 5Hz
			} else {

//				sprintf((char*)json,"{\"temperature\":%3d,\"accelerometer\":%f,%f,%f,\"pressure_sensor\":%f}",temp_data,accel_data[0],accel_data[1],accel_data[2],pressure_data);
//				HAL_UART_Transmit(&huart1, (uint8_t*) message_print,
//										strlen(message_print), 0xFFFF);
//				sprintf((char*)req, "POST /api/notes/ HTTP/1.1\r\nHost: https://rach.codes\r\nContent-Length: %d\r\n\r\n%s",strlen(json),json);
//				HAL_UART_Transmit(&huart1, (uint8_t*) message_print,
//														strlen(message_print), 0xFFFF);//				printf("The humidity is %0.2f\n and the pressure data is %f", humidity_data, pressure_data);
//				WiFi_Stat = WIFI_SendData(1, wifi_message, (uint16_t)strlen((char*)wifi_message), &Datalen, WIFI_WRITE_TIMEOUT);
			}









			// The dangerous mode here








			while (mode == INTENSIVE) {
				//has to have polling behaviour and at the same time, it needs to have the following conditionns
				//activate all the sensors here
				//temperatuyre is greater than 37.6 create a 5hz blinking led and realise that the fever is detected
				HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);
				humidity_data = BSP_HSENSOR_ReadHumidity();


				BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
				// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
				accel_data[0] = ((float)accel_data_i16[0] / 100.0f) / 9.8f;
				accel_data[1] = ((float)accel_data_i16[1] / 100.0f) / 9.8f;
				accel_data[2] = ((float)accel_data_i16[2] / 100.0f) / 9.8f;

				//temperature and humidity
				temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor every x seconds


				//gyroscope
				BSP_GYRO_GetXYZ(gyro_data_i16);		//read gyro
				//convert gyro data into meaningful value in terms of dps;
				gyro_data[0] = (gyro_data_i16[0] + 630.0f) / 1000.0f;
				gyro_data[1] = (gyro_data_i16[1] + 280.0f) / 1000.0f;
				gyro_data[2] = (gyro_data_i16[2] + 140.0f) / 1000.0f ;
				//get root mean square value
				gyro_total = sqrt(pow(gyro_data[0],2)+pow(gyro_data[1],2)+pow(gyro_data[2],2));

				//magnetometer
				BSP_MAGNETO_GetXYZ(magneto_data_i16);	//read magneto
				//convert magneto data into meaningful value in terms of gauss
				magneto_data[0] = (float)magneto_data_i16[0] / 1000.0f;
				magneto_data[1] = (float)magneto_data_i16[1] / 1000.0f;
				magneto_data[2] = (float)magneto_data_i16[2] / 1000.0f;
				if (count == 0){						//set the baseline position when first entered intensive mode
					Mag_Baseline[0] = magneto_data[0];
					Mag_Baseline[1] = magneto_data[1];
					Mag_Baseline[2] = magneto_data[2];
				}


					if (HAL_GetTick() % 10000 <50) {
						sprintf(message_print, "%03d TEMP_%0.2f ACC %0.2f %0.2f %0.2f\r\n",countBluff, temp_data, accel_data[0], accel_data[1], accel_data[2]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						countBluff++;
						sprintf(message_print, "%03d GYRO %0.1f MAGNETO %0.2f %0.2f %0.2f\r\n", countBluff, gyro_total, magneto_data[0], magneto_data[1], magneto_data[2]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						countBluff++;
						sprintf(message_print, "%03d HUMIDITY %0.2f and BARO %0.2f\r\n",countBluff,  humidity_data, pressure_data);
						HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						countBluff++;




						for (int i = 0; i < 3; i++)
											Mag_Difference[i] = fabs(Mag_Baseline[i]-magneto_data[i]);

										pressure_data = BSP_PSENSOR_ReadPressure()*100.0f;	//pressure in pascal
										if(Mag_Difference[0] >= MAG_THRESHOLD || Mag_Difference[1] >= MAG_THRESHOLD || Mag_Difference[2] >= MAG_THRESHOLD){
											magflag = WARNING;
											sprintf(message_print, "\r\nCheck patient's abnormal orientation!\r\n");
											HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
											if (HAL_GetTick() % 200 < 50) {
												HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
											}
										}
										if (temp_data >= TEMP_THRESHOLD){
											tempflag = WARNING;
											sprintf(message_print, "\r\nFever is detected!\r\n");
											HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
										}
										if (humidity_data <= HUM_THRESHOLD){
											humidflag = WARNING;
											sprintf(message_print, "\r\nCheck patient's breath!\r\n");
											HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
										}
										if (accel_data[0] > ACC_THRESHOLD || accel_data[1] > ACC_THRESHOLD
												|| accel_data[2] > ACC_THRESHOLD) {
											mode = INTENSIVE;
											//	int mytime;
											sprintf(message_print, "\r\nFall is detected\r\n");
											HAL_UART_Transmit(&huart1, (uint8_t*) message_print,
													strlen(message_print), 0xFFFF);
											if (gyro_total <= GYRO_THRESHOLD){
												gyroflag = WARNING;
												sprintf(message_print, "Patient in pain!\r\n");
												HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
											}




					}
					//pressure
					//
					//				}

				}

				//pressure sensor here

			}
		}
	}
}

	//		if(mode == HEALTHY ){
	//			//BSP_TSENSOR_Init();
	//
	//			//Acceleration is located below this line
	//			float accel_data[3];
	//			int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
	//			BSP_ACCELERO_AccGetXYZ(accel_data_i16);
	//			accel_data[0] = (float)accel_data_i16[0] / 100.0f;
	//			accel_data[1] = (float)accel_data_i16[1] / 100.0f;
	//			accel_data[2] = (float)accel_data_i16[2] / 100.0f;
	//			//////Acceleration Declaration End/////
	//
	//
	//
	////Declaration of the temperature and other variables which are
	//			float temp_data;// read temp
	//			float humidity_data;
	//			float pressure_data;
	//
	////Magnetometer and the current details for the items would be declared below this line
	//			float magnetometer_data[3];
	//			int16_t magneto_int16[3] = { 0 };
	//			BSP_MAGNETO_GetXYZ(magneto_int16);
	//			magnetometer_data[0] = (float)magneto_int16[0] / 100.0f;
	//			magnetometer_data[1] = (float)magneto_int16[1] / 100.0f;
	//			magnetometer_data[2] = (float)magneto_int16[2] / 100.0f;
	//
	//
	//			//gyroscope data is located below this line
	//			float gysoscope_data[3];
	//			int16_t gyro_int16[3] = { 0 };
	//		    BSP_GYRO_GetXYZ(gyro_int16);
	//			gysoscope_data[0] = (float)gyro_int16[0] / 100.0f;
	//			gysoscope_data[1] = (float)gyro_int16[1] / 100.0f;
	//			gysoscope_data[2] = (float)gyro_int16[2] / 100.0f;
	//
	//
	//			temp_data = BSP_TSENSOR_ReadTemp();
	//			humidity_data = BSP_HSENSOR_ReadHumidity();
	//			pressure_data = BSP_PSENSOR_ReadPressure()*100.0f; //pressure in pascal
	//		//	printf("Temperature data is %f\n", temp_data);
	//			printf("Humidity data is %f\n", humidity_data);
	////			printf("Pressure data is %f\n", pressure_data);
	//		//	doIHaveFever(temp_data);
	//		//	Respiratory(humidity_data, pressure_data);
	//		// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
	//
	//			printf("Accel X : %f; Accel Y : %f; Accel Z : %f; Temperature : %f\n\n\n", accel_data[0], accel_data[1], accel_data[2], temp_data);
	//			printf("Gyro X : %f; Gyro Y : %f; Gyro Z : %f; Pressure : %f\n", gysoscope_data[0], gysoscope_data[1], gysoscope_data[2], pressure_data);
	//			printf("Magnetometer1 X : %f; Magnetometer2 Y : %f; Magnetometer3 Z : %f; Humidity : %f\n", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2], humidity_data);
	//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	//
	//
	//			if(temp_data > TEMP_THRESHOLD){
	//				tempflag = WARNING;
	//				mode = INTENSIVE;
	//
	//			}
	//

	//
	//
	//
	//
	//		}
	//	}

	static void MX_GPIO_Init(void) {
		/* GPIO Ports Clock Enable */

		__HAL_RCC_GPIOB_CLK_ENABLE();	// Enable AHB2 Bus for GPIOB
		__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC
		__HAL_RCC_GPIOD_CLK_ENABLE();// Enable Temperature and pressure sensor GPIOD

		//HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET); // Reset the LED2_Pin as 0

		GPIO_InitTypeDef GPIO_InitStruct = { 0 };

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

		// Configuration of LPS22HB_Pin (GPIO-D Pin-10) as GPIO output
		//	 GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin;
		//	 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		//	 GPIO_InitStruct.Pull = GPIO_NOPULL;
		//	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		//	 HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		// Enable NVIC EXTI line 13
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);





		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);

	}
	void ResetFlag(void){
		accflag = SAFE;
		gyroflag = SAFE;
		pressureflag = SAFE;
		tempflag = SAFE;
		magflag = SAFE;
		humidflag = SAFE;
	}

	static void UART1_Init(void) {
		/* Pin configuration for UART. BSP_COM_Init() can do this automatically */
		__HAL_RCC_GPIOB_CLK_ENABLE();
		GPIO_InitTypeDef GPIO_InitStruct = { 0 };
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Configuring UART1 */
		huart1.Instance = USART1;
		huart1.Init.BaudRate = 115200;
		huart1.Init.WordLength = UART_WORDLENGTH_8B;
		huart1.Init.StopBits = UART_STOPBITS_1;
		huart1.Init.Parity = UART_PARITY_NONE;
		huart1.Init.Mode = UART_MODE_TX_RX;
		huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart1.Init.OverSampling = UART_OVERSAMPLING_16;
		huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		if (HAL_UART_Init(&huart1) != HAL_OK) {
			while (1)
				;
		}

	}



	void SPI3_IRQHandler(void)
	{
	    HAL_SPI_IRQHandler(&hspi3);
	}
