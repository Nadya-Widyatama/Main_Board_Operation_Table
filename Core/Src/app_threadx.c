/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.c
 * @author  MCD Application Team
 * @brief   ThreadX applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2020-2021 STMicroelectronics.
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THREAD_STACK_SIZE 1024
//#define THREAD_STACK_SIZE 	512
//#define QUEUE_STACK_SIZE	128
#define FORWARD 1
#define REVERSE 2
#define STOP 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t uart_rx_data;
volatile uint8_t uart_data_received = 0;
uint32_t counter = 0;
uint32_t last_time, current_Time, dian;
int running = 0;
int target = 0;
int pulse = 0;
int speed = 0;
int rpm = 0;
void set(void);
void Selenoid_STOP(void);
void Selenoid_UP(void);
void Selenoid_DOWN(void);

uint8_t thread_Receiver[THREAD_STACK_SIZE];
uint8_t thread_Transmite[THREAD_STACK_SIZE];
uint8_t thread_LED_1[THREAD_STACK_SIZE];
uint8_t thread_ROTARY[THREAD_STACK_SIZE];
uint8_t thread_Set[THREAD_STACK_SIZE];

TX_THREAD ReceiverUART;
TX_THREAD TransmiteUART;
TX_THREAD LED_1_ptr;
TX_THREAD ROTARY;
TX_THREAD setone;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

VOID UART_Receiver(ULONG initial_input);
VOID UART_Transmit(ULONG initial_input);
VOID LD1_thread_entry(ULONG initial_input);
VOID ROTARY_thread(ULONG initial_input);
VOID Set_satu(ULONG initial_input);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
	TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
	(void)byte_pool;
  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
	tx_thread_create(&ReceiverUART, "Receiver", UART_Receiver, 0, thread_Receiver, THREAD_STACK_SIZE, 14, 14, 1, TX_AUTO_START);
	tx_thread_create(&TransmiteUART, "TranSmite", UART_Transmit, 0, thread_Transmite, THREAD_STACK_SIZE, 15, 15, 1, TX_AUTO_START);
	tx_thread_create(&LED_1_ptr, "LED_TASK", LD1_thread_entry, 0, thread_LED_1, THREAD_STACK_SIZE, 15, 15, 1, TX_AUTO_START);
	tx_thread_create(&ROTARY, "ROTARY_TASK", ROTARY_thread, 0, thread_ROTARY, THREAD_STACK_SIZE, 15, 15, 1, TX_AUTO_START);
	tx_thread_create(&setone, "Set", Set_satu, 0, thread_Set, THREAD_STACK_SIZE, 14, 14, 1, TX_AUTO_START);
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */
  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */
  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// Handle received data
	if (huart == &huart5) {
		// Set flag that data is received
		uart_data_received = 1;
		// Prepare for the next reception
		HAL_UART_Receive_DMA(&huart5, &uart_rx_data, 1);
	}
}

void SetMotorSpeed(uint32_t channel, uint16_t speed){
	if (speed > 255) speed = 255;
	__HAL_TIM_SET_COMPARE(&htim1, channel, speed);
}

void SetRotaryMotor(uint32_t channel, uint16_t speed){
	if (speed > 255) speed = 255;
	__HAL_TIM_SET_COMPARE(&htim1, channel, speed);
}

void SetMotorPump(uint32_t channel, uint16_t speed){
	if (speed > 255) speed = 255;
	__HAL_TIM_SET_COMPARE(&htim3, channel, speed);
}
void UART_Receiver (ULONG initial_input) {
	// Enable UART DMA reception
	HAL_UART_Receive_DMA(&huart5, &uart_rx_data, 1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Start PWM on PC0 = HIN A
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // Start PWM on PC2 = HIN B

	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	for (;;) {
		if (uart_data_received) {
			uart_data_received = 0;
			switch (uart_rx_data) {
			case 0x00:
				Selenoid_STOP();
				//SetRotaryMotor(TIM_CHANNEL_3, 0);
				//SetMotorPump(TIM_CHANNEL_4, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); //motor pump
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0); //rotary
				if (target > 0){
					target = 0;
					set();
				}
				break;

			case 0x01:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
				target = 1;
				set();
				Selenoid_UP();
				//printf("ROLL_UP\n");
				break;

			case 0x02:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
				target = 2;
				set();
				SetMotorPump(TIM_CHANNEL_4, 127);
				Selenoid_UP();
				//printf("PITCH_UP\n");
				break;

			case 0x03:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
				target = 3;
				set();
				Selenoid_UP();
				//printf("LIFT_UP\n");
				break;

			case 0x04:
				//CASE UNTUK TES (TOMBOL A)
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1); //motor pump
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
				//Selenoid_DOWN();
				//SetRotaryMotor(TIM_CHANNEL_3, 100);
				//SetMotorPump(TIM_CHANNEL_4, 127);
				//SetMotorSpeed(TIM_CHANNEL_1, 0);
				//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
				break;

			case 0x05:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
				target = 3;
				set();
				Selenoid_DOWN();
				//printf("ROLL_DOWN\n");
				break;

			case 0x06:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
				target = 2;
				set();
				Selenoid_DOWN();
				//printf("PITCH_DOWN\n");
				break;

			case 0x07:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
				target = 1;
				set();
				Selenoid_DOWN();
				//printf("LIFT_DOWN\n");
				break;

			case 0x08:

				break;

			case 0x09:

				break;

			case 0xA:


			default:

				break;
			}
			tx_thread_sleep(50);
		}
	}
}

void UART_Transmit(ULONG initial_input){
	char message[37];
	int counter1 = 0;
	int counter2 = 0;
	int counter3 = 0;
	while (1){
		counter1++;
		counter2++;
		counter3++;
		if (counter1 > 10 || counter2 > 10 || counter3 > 10) {

			counter1 = 0;
			counter2 = 0;
			counter3 = 0;
		}
		sprintf(message, "%d,%d,%d\n", counter1, counter2, counter3);
		HAL_UART_Transmit(&huart3, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
		tx_thread_sleep(100);
	}
}


void LD1_thread_entry (ULONG initial_input){
	while(1){
		//tx_thread_sleep(50);
	}
}

void ROTARY_thread (ULONG initial_input){
	while(1){
		//tx_thread_sleep(100);
	}
}

void Set_satu(ULONG initial_input){
	target = 4;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
	Selenoid_UP();
//	set();
	while(1){
		tx_thread_sleep(TX_WAIT_FOREVER);
	}
}

void set(void){
	running = 1;
	while(pulse != target){
		SetRotaryMotor(TIM_CHANNEL_1, 153);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
		if (pulse == target) {
			running = 0;
			break;
		}
	}
	SetRotaryMotor(TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	tx_thread_sleep(200);
}

void Selenoid_STOP(void){
	SetMotorSpeed(TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	SetMotorSpeed(TIM_CHANNEL_3, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
}

void Selenoid_UP(void){
	SetMotorSpeed(TIM_CHANNEL_1, 127);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	SetMotorSpeed(TIM_CHANNEL_3, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
}

void Selenoid_DOWN(void){
	SetMotorSpeed(TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	SetMotorSpeed(TIM_CHANNEL_3, 255);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
}


void EXTI9_5_IRQHandler(void){
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET){
		current_Time = tx_time_get();
		dian = current_Time - last_time;
		if(dian > 10 && running == 1){
			pulse++;
			if (dian < 60){
				pulse = 0;
			}
		}
		last_time = current_Time;
		//printf("pulse: %d\n", pulse);
		printf("dian: %lu\n", dian);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
	}
}
/* USER CODE END 1 */
