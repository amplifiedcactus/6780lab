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
int main(void)
{
	
	
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to GPIOC

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //Enable peripheral clock to Timer 2
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Enable peripheral clock to Timer 3
	
	
	
	
	TIM2->PSC = 7999; //Set PSC to 7999 to Set frequency to 4Hz 
	TIM2->ARR = 250; //Set auto-reload register to 250 to set frequency to 4Hz
	TIM2->DIER |= (1 << 0); //Enable Update interrupt
	TIM2->CR1 |= (1 << 0); //Set CEN bit to enable counter
	
	TIM3->PSC = 99; //Set PSC to Set frequency to 800Hz 
	TIM3->ARR = 100; //Set auto-reload register to set frequency to 800Hz
	
	//Clear CC1S and CC2S bits to configure channels as outputs
	TIM3->CCMR1 &= ~((1 << 0) | (1 << 1) | (1 << 8) | (1 << 9));
	//Set OC2M (PC7, blue) to PWM mode 1 and OC1M (PC6, red) to PWM mode 2
	TIM3->CCMR1 |= (1 << 14) | (1 << 13) | (0 << 12) | (1 << 6) | (1 << 5) | (1 << 4);
	TIM3->CCMR1 &= ~(1 << 12);
	//Set OC1PE and OC2PE to enable output compare preload
	TIM3->CCMR1 |= (1 << 3) | (1 << 11);
	//Set output enable bits for channel 1 and 2
	TIM3->CCER |= (1 << 0) |  (1 << 4);
	//Set capture/compare registers to .2 of ARR value for both channels
	TIM3->CCR1 = 20;
	TIM3->CCR2 = 20;
	
	
	NVIC_EnableIRQ (TIM2_IRQn); //enable NVIC interrupt
	NVIC_SetPriority (TIM2_IRQn, 3); //set EXTI0 interrupt priority to 3
	
	
	
	
	
	// Initialize pins PC6 (red), PC7 (blue), PC8 (orange), PC9 (green)
	// Set 8 and 9 to general purpose output mode
	GPIOC->MODER |= (1 << 18) | (1 << 16);
	GPIOC->MODER &= ~((1 << 19) |(1 << 17));
	// Set 7 and 6 to alternate function mode
	GPIOC->MODER |= (1 << 15) | (1 << 13);
	GPIOC->MODER &= ~((1 << 14) | (1 << 12));
	// Set to push pull output type
	GPIOC->OTYPER &= ~(0x000FF000); //0000 0000 0000 1111 1111 0000 0000 0000
	// Set to low speed
	GPIOC->OSPEEDR &= ~(0x000FF000);
	// Set to no pullup/down resistor
	GPIOC->PUPDR &= ~(0x000FF000);
	
	
	
	//Select AF0 for PC6 and PC7
	GPIOC->AFR[0] |= 0x00 << GPIO_AFRL_AFRL6_Pos;
	GPIOC->AFR[0] |= 0x00 << GPIO_AFRL_AFRL7_Pos;
	
	TIM3->CR1 |= (1 << 0); //Set CEN bit to enable counter
	
	//turn on LEDs
	GPIOC->ODR |= (1 << 9);
	GPIOC->ODR &= ~(1 << 8);
	
	while (1) {
		
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
