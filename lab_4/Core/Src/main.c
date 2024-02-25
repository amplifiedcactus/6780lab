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
char error[] = {'\r', 'E', 'r', 'r', 'o', 'r', '\n', 0}; //Error message
char ccmd[] = {'\r', 'C', 'o', 'l', 'o', 'r', '?', '\n', '\r', 0}; //Ask for color message
char ncmd[] = {'\r', 'N', 'u', 'm', 'b', 'e', 'r', '?', '\n', '\r', 0}; //Ask for number message

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

//This function takes in a single character and transmits it from USART3_TX
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

//This function takes in an array of characters and transmits it from USART3_TX
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

//This is the USART3 interrupt handler that puts the read character into char r and sets the usart_flag
void USART3_4_IRQHandler(void) {
	r = USART3->RDR;
	usart_flag = 1;
}

//This function takes in a number for color and either turns on, toggles, or turns off that color LED depending on what number is typed in
void led(int color) {
	string_transmit(ncmd);
	while (1) {
		HAL_Delay(10); // Delay 10ms
		//get number input
		if (usart_flag == 1){
			//if 0 is input, turn off led
			if (r == '0')
				GPIOC->ODR &= ~color;
			//if 1 is input, turn on led
			else if (r == '1')
				GPIOC->ODR |= color;
			//if 2 is input, toggle led
			else if (r == '2')
				GPIOC->ODR ^= color;
			else {
				string_transmit(error);
				usart_flag = 0;
			}
			break;
		}
	}
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
}

//This function sets up pins PB10 and PB11 for using UART3
void setupUART3(void) {
	
	// Set PB10 (USART3_TX) and PB11 (USART3_RX) to alternate function mode
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable peripheral clock to GPIOB
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable peripheral clock to USART3
	
	setupLED(); //Set up LED GPIOC pins
	
	setupUART3(); //set up UART GPIOB pins

	char c = 0; //character for storing character typed into terminal
	
	//numbers for editing GPIOC ODR register for each color LED
	int green = 1 << 9;
	int red = 1 << 6;
	int blue = 1 << 7;
	int orange = 1 << 8;
	
	//ask for color for first time
	string_transmit(ccmd);
	
	while (1) {
		HAL_Delay(10); // Delay 10ms, for some reason the if statements in lab checkoff 2 don't work without this
		
		//Lab checkoff 1: toggle each LED whenever r, g, b, or o are typed into UART terminal
		//if read register is not empty, read data
//		if (USART3->ISR & (1 << 5)){
//			c = USART3->RDR;
//			
//			if (c == 'g')
//				GPIOC->ODR ^= (1 << 9);
//			else if (c == 'r')
//				GPIOC->ODR ^= (1 << 6);
//			else if (c == 'b')
//				GPIOC->ODR ^= (1 << 7);
//			else if (c == 'o')
//				GPIOC->ODR ^= (1 << 8);
//			else
//				string_transmit(error);
//		}
		
		//Lab checkoff 2: control LEDs by typing a key, then a number into UART terminal
		//If USART flag as been set, it has read a character
		if (usart_flag == 1){
			//If the character is g, call LED function to edit green LED pin and reset USART flag
			if (r == 'g'){
				usart_flag = 0;
				led(1 << 9);
				usart_flag = 0;
			}
			//Do same thing for other characters
			else if (r == 'r'){
				usart_flag = 0;
				led(1 << 6);
				usart_flag = 0;
			}
			else if (r == 'b'){
				usart_flag = 0;
				led(1 << 7);
				usart_flag = 0;
			}
			else if (r == 'o'){
				usart_flag = 0;
				led(1 << 8);
				usart_flag = 0;
			}
			//If character is not r, g, b, or o, send back error message and reset USART flag
			else{
				string_transmit(error);
				usart_flag = 0;
			}
			//Ask for color again before going back into while loop.
			string_transmit(ccmd);
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
