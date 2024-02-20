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
void sctransmit(char x) {
	
	//wait for transmit register to be empty
	while (1) {
		if (USART3->ISR & (1 << 7)) {
			break;
		}
	}
	
	//write character to transmit register
	USART3->TDR = x;
}

void string_transmit (char a[]) {
	int counter = 0;
	while (1) {
		if (a[counter] != 0) {
			sctransmit(a[counter]);
			counter = counter + 1;
		}
		else
			break;
	}
	
}
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
  
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to GPIOC
	
	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable peripheral clock to GPIOB
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable peripheral clock to USART3
	
	
	
	// Initialize pins PC6 (red), PC7 (blue), PC8 (orange), PC9 (green)
	// Set to general purpose output mode
	GPIOC->MODER |= (1 << 18) | (1 << 16) | (1 << 14) | (1 << 12);
	GPIOC->MODER &= ~((1 << 19) |(1 << 17) |(1 << 15) | (1 << 13));
	// Set to push pull output type
	GPIOC->OTYPER &= ~(0x000FF000); //0000 0000 0000 1111 1111 0000 0000 0000
	// Set to low speed
	GPIOC->OSPEEDR &= ~(0x000FF000);
	// Set to no pullup/down resistor
	GPIOC->PUPDR &= ~(0x000FF000);
	
	// Set PB10 and PB11 to alternate function mode
	GPIOB->MODER |= (1 << 23) | (1 << 21);
	GPIOB->MODER &= ~((1 << 22) | (1 << 20));
	// Set to push pull output type
	GPIOB->OTYPER &= ~((1 << 11) | (1 << 10));
	// Set to low speed
	GPIOB->OSPEEDR &= ~((1 << 23) | (1 << 22) | (1 << 21) | (1 << 20));
	// Set to no pullup/down resistor
	GPIOB->PUPDR &= ~((1 << 23) | (1 << 22) | (1 << 21) | (1 << 20));
	
	
	
	//Select AF4 for PB10 and PB11
	GPIOB->AFR[1] |= 0x04 << GPIO_AFRH_AFSEL10_Pos;
	GPIOB->AFR[1] |= 0x04 << GPIO_AFRH_AFSEL11_Pos;
	
	//set baud rate to about 115200
	USART3->BRR |= HAL_RCC_GetHCLKFreq() / 115200;
	
	//HAL_RCC_GetHCLKFreq() / 115200 = 69.44444
	
	//enable transmitter and receiver
	USART3->CR1 |= (1 << 3) | (1 << 2);
	
	//enable USART 3
	USART3->CR1 |= (1 << 0);
	

	
	//GPIOC->ODR &= ~(1 << 8);
	char name[] = {'S', 't', 'e', 'p', 'h', 'e', 'n', 0};
	char error[] = {'e', 'r', 'r', 'o', 'r', '\n', 0};
	char c = 0;
	//char read = 0;
	
	while (1) {
		//string_transmit(name);
		//HAL_Delay(200); // Delay 2000ms
		
		//if read register is not empty, read data
		if (USART3->ISR & (1 << 5)){
			c = USART3->RDR;
			
			if (c == 'g')
				GPIOC->ODR ^= (1 << 9);
			else if (c == 'r')
				GPIOC->ODR ^= (1 << 6);
			else if (c == 'b')
				GPIOC->ODR ^= (1 << 7);
			else if (c == 'o')
				GPIOC->ODR ^= (1 << 8);
			else
				string_transmit(error);
		}
		
	}

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
