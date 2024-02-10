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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	//fdsa
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable peripheral clock to GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // Enable peripheral clock to SYSCFG
	
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
	
	
	
	// Set pin PA0 to input mode
	GPIOA->MODER &= ~((1 << 1) | (1 << 0));
	// Set pin PA0 to low speed
	GPIOA->OSPEEDR &= ~((1 << 1) | (1 << 0));
	// Set pin PA0 to pull down resistor
	GPIOA->PUPDR |= (1 << 1);
	GPIOA->PUPDR &= ~(1 << 0);
	
	EXTI->IMR |= (1 << 0); //unmask EXTI input line 0 interrupt
	EXTI->RTSR |= (1 << 0); //enable rising edge trigger
	SYSCFG->EXTICR[0] &= ~(0x0000000F); //route PA0 to EXTI0
	
	NVIC_EnableIRQ (EXTI0_1_IRQn);
	NVIC_SetPriority (EXTI0_1_IRQn, 1);
	
	
	//turn on LEDs
	GPIOC->ODR |= (1 << 9);
	while (1) {
		HAL_Delay(600); // Delay 200ms
		GPIOC->ODR ^= (1 << 6); //flash 6
	}
	
	//Blink red and blue LEDs
	/*
	while (1) {
		HAL_Delay(200); // Delay 200ms
		GPIOC->ODR ^= (1 << 7) | (1 << 6); //invert 6 and 7
	}
	*/
	
	/*
	//Toggle red and blue LEDs on button press
	uint32_t debouncer = 0;
	while(1) {
	debouncer = (debouncer << 1); // Always shift every loop iteration
	if (GPIOA->IDR & (1 << 0)) { // If input signal is set/high
	debouncer |= 0x01; // Set lowest bit of bit-vector
	}
	if (debouncer == 0xFFFFFFFF) {
	// This code triggers repeatedly when button is steady high!
	}
	if (debouncer == 0x00000000) {
	// This code triggers repeatedly when button is steady low!
	}
	if (debouncer == 0x7FFFFFFF) {
	// This code triggers only once when transitioning to steady high!
		GPIOC->ODR ^= (1 << 7) | (1 << 6); //invert 6 and 7
	}
	// When button is bouncing the bit-vector value is random since bits are set when
	//the button is high and not when it bounces low.
	HAL_Delay(2); // Delay 2ms
	}
	*/
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
