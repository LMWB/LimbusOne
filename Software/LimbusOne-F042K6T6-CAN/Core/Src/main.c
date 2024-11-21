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
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	// 1) init can -> done in cube mx
	// 2) configure can filter -> no filters for this demo all can messages are allowed to pass
  	  CAN_FilterTypeDef sFilterConfig;
  	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  	  sFilterConfig.FilterIdHigh = 0x00;
  	  sFilterConfig.FilterIdLow = 0x00;
  	  sFilterConfig.FilterMaskIdHigh = 0x0;
  	  sFilterConfig.FilterMaskIdLow = 0x0;
  	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  	  sFilterConfig.FilterActivation = ENABLE;
  	  sFilterConfig.FilterBank = 0;
  	  HAL_CAN_ConfigFilter( &hcan, &sFilterConfig );

	// 3) activate interrupts
  	  HAL_StatusTypeDef can_state = 0;
  	can_state = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  	can_state = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  	can_state = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_FULL);
  	can_state = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_OVERRUN);
  	can_state = HAL_CAN_ActivateNotification(&hcan, CAN_IT_ERROR);

	// 4) start can hardware
  	can_state = HAL_CAN_Start(&hcan);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/*
		 * canID => CAN Identifier 11Bit or 29Bit
		 * extID => 0 = Standard, 1 = Extended
		 * aData = Payload, pointer to byte array
		 * DLC = Data length code, must be  <= 8
		 * */
		CAN_TxHeaderTypeDef sTxHeader;
		HAL_StatusTypeDef can_tx_rx_status;

		// choose standard or extended CAN identifier type
		uint8_t extID = 0;
		// choose CAN identifier
		uint32_t canID = 0x101;
		uint32_t tx_mailbox = 0;


		uint8_t DLC = 8;
		uint8_t aData[8] = { 'R', 'O', 'L', 'L', 'E', 'N', 'R', 'A'};

		if (extID) {
			sTxHeader.StdId = 0;
			sTxHeader.ExtId = canID;
			sTxHeader.IDE = CAN_ID_EXT;
		} else {
			sTxHeader.StdId = canID;
			sTxHeader.ExtId = 0;
			sTxHeader.IDE = CAN_ID_STD;
		}
		/* Data Length Code max. 8 byte */
		if (DLC <= 8) {
			sTxHeader.DLC = DLC;
		} else {
			sTxHeader.DLC = 8;
		}
		sTxHeader.RTR = CAN_RTR_DATA;
		sTxHeader.TransmitGlobalTime = DISABLE;
		can_tx_rx_status = HAL_CAN_AddTxMessage(&hcan, &sTxHeader, aData, &tx_mailbox);

		HAL_Delay(200);
		//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 5) action on CAN-RX-interrupt
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef sRxHeader = {0};
	HAL_StatusTypeDef tx_rx_status;
	uint8_t rx_data[8];
	if (hcan->Instance == CAN) {
		/* pick up incoming data and release the CAN Peripheral */
		tx_rx_status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &sRxHeader, rx_data);

		// do stuff with header (metadata)
		sRxHeader.DLC;
		sRxHeader.ExtId;
		sRxHeader.FilterMatchIndex;
		sRxHeader.IDE;
		sRxHeader.RTR;
		sRxHeader.StdId;
		sRxHeader.Timestamp;

		// do stuff with payload

		// indicate something going on on can-interrupts
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);


	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	// indicate something going on on can-interrupts
	//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
	// todo
	//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	uint32_t error = hcan->ErrorCode;
	// todo
	//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

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
	while (1) {
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
