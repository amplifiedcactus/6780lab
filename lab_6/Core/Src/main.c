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
char r = 0; //This is for the character typed into the terminal
int usart_flag = 0; //Flag for if a character has been read by UART3
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
void setupUART3(void) {
	
	// Set PC4 (USART3_TX) and PC5 (USART3_RX) to alternate function mode
	GPIOC->MODER |= (1 << 11) | (1 << 9);
	GPIOC->MODER &= ~((1 << 10) | (1 << 8));
	// Set to push pull output type
	GPIOC->OTYPER &= ~((1 << 5) | (1 << 4));
	// Set to low speed
	GPIOC->OSPEEDR &= ~((1 << 11) | (1 << 10) | (1 << 9) | (1 << 8));
	// Set to no pullup/down resistor
	GPIOC->PUPDR &= ~((1 << 11) | (1 << 10) | (1 << 9) | (1 << 8));
	
	//Select AF1 for PC4 and PC5
	GPIOC->AFR[0] |= 0x01 << GPIO_AFRL_AFSEL4_Pos;
	GPIOC->AFR[0] |= 0x01 << GPIO_AFRL_AFSEL5_Pos;
	
	//set baud rate to about 115200
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;
	
	//enable transmitter and receiver
	USART3->CR1 |= (1 << 3) | (1 << 2);
	
	//enable RXNE interrupt
	USART3->CR1 |= (1 << 5);
	NVIC_EnableIRQ (USART3_4_IRQn); //enable NVIC interrupt
	NVIC_SetPriority (USART3_4_IRQn, 3);
	
	//enable USART 3
	USART3->CR1 |= (1 << 0);
}

//This is a function for transmitting a single character to UART3
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

//This is a function for transmitting a single integer 0-9 to UART3
void inttransmit(int x) {
	//wait for transmit register to be empty
	while (1) {
		if (USART3->ISR & (1 << 7)) {
			break;
		}
	}
	//write character to transmit register
	USART3->TDR = '0' + x;
}

//This is a function for converting a integer to a 16 bit binary and transmitting it to UART3
void transmit16bits(int x) {
	setupUART3();
	sctransmit('\r');
	for (int i = 15; i >= 0; i = i - 1){
		int t = 0;
		if (x & (1 << i))
			t = 1;
		inttransmit(t);
	}
	sctransmit('\r');
	sctransmit('\n');
}

//This is the USART3 interrupt handler that puts the read character into char r and sets the usart_flag
void USART3_4_IRQHandler(void) {
	r = USART3->RDR;
	usart_flag = 1;
}



//This function sets up the GPIOC pins for using the LEDs
void setupLED(void) {
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
	//To turn on LEDs:
	//GPIOC->ODR |= (1 << 7); //Turn on blue LED
	//To toggle LEDs:
	//GPIOC->ODR ^= (1 << 8); //Toggle orange LED
	//To turn off LEDs:
	//GPIOC->ODR &= ~(1 << 6); //Turn off red LED
	//GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
}

//set up pin PC0 as analog input 10
void setupADC(void) {
	//Set to analog mode
	GPIOC->MODER |= (1 << 1) | (1 << 0);
	// Set to push pull output type
	GPIOC->OTYPER &= ~(1 << 0);
	// Set to low speed
	GPIOC->OSPEEDR &= ~(1 << 0);
	// Set to no pullup/down resistor
	GPIOC->PUPDR &= ~((1 << 1) | (1 << 0));
	
	//Set ADC to 8 bit resolution, continuous conversion, hardware triggers disabled
	ADC1->CFGR1 |= (1 << 13) | (1 << 4);
	ADC1->CFGR1 &= ~((1 << 11) | (1 << 10) | (1 << 3));
	//Select channel 10
	ADC1->CHSELR |= (1 << 10);
}

void setupDAC(void) {
	//Set to analog mode
	GPIOA->MODER |= (1 << 9) | (1 << 8);
	// Set to push pull output type
	GPIOA->OTYPER &= ~(1 << 4);
	// Set to low speed
	GPIOA->OSPEEDR &= ~(1 << 4);
	// Set to no pullup/down resistor
	GPIOA->PUPDR &= ~((1 << 9) | (1 << 8));
	
	//set DAC channel 1 to software trigger
	DAC1->CR |= (1 << 5) | (1 << 4) | (1 << 3);
	
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
  /* MCU Configuration--------------------------------------------------------*/


	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable peripheral clock to GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN; //Enable peripheral clock to ADC1
	RCC->APB1ENR |= RCC_APB1ENR_DACEN; //Enable peripheral clock to DAC
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable peripheral clock to USART3
	
	setupADC();
	setupDAC();
	setupLED();
	setupUART3();
	
	
	//Perform self-calibration
	ADC1->CR |= (1 << 31);
	//wait for self-calibration to complete
	while((ADC1->CR & (1 << 31)) != 0) {
		HAL_Delay(100);
	}
	//Enable ADC peripheral
	ADC1->CR |= (1 << 0);
	//wait until ADC is ready
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
		HAL_Delay(100);
	}
	//start ADC by setting ADSTART bit
	ADC1->CR |= (1 << 2);
	
	//enable DAC channel 1
	DAC1->CR |= (1 << 0);
	
	// Sawtooth Wave: 8-bit, 32 samples/cycle
	const uint8_t sawtooth_table[32] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,238,246};
	
	int i = 0; //index for wave table

  /* Infinite loop */

  while (1) {
		//output wave table to DAC
		DAC1->DHR8R1 = sawtooth_table[i];
		i = i + 1;
		if (i == 31)
			i = 0;
		
		//light up different LEDs based on ADC value
		int value = ADC1->DR;
		if ((0 <= value) && (value < 64)) {
			GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
			GPIOC->ODR |= (1 << 6);
		}
		else if ((64 <= value) && (value < 128)) {
			GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
			GPIOC->ODR |= (1 << 7);
		}
		else if ((128 <= value) && (value < 192)) {
			GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
			GPIOC->ODR |= (1 << 8);
		}
		else {
			GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
			GPIOC->ODR |= (1 << 9);
		}
		
		//delay for 1ms
		HAL_Delay(1);
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
