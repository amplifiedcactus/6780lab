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
	
}

//This function sets up 
void setupI2C(void) {
	
	// Set PB11 (I2C2_SDA) and PB13 (I2C2_SCL) to alternate function mode
	GPIOB->MODER |= (1 << 23) | (1 << 27);
	GPIOB->MODER &= ~((1 << 22) | (1 << 26));
	// Set to open drain output type
	GPIOB->OTYPER |= (1 << 11) | (1 << 13);
	// Set to low speed
	GPIOB->OSPEEDR &= ~((1 << 23) | (1 << 22) | (1 << 27) | (1 << 26));
	// Set to no pullup/down resistor
	GPIOB->PUPDR &= ~((1 << 23) | (1 << 22) | (1 << 27) | (1 << 26));
	
	//Select AF1 for PB11 and AF5 for PB13
	GPIOB->AFR[1] |= 0x01 << GPIO_AFRH_AFSEL11_Pos;
	GPIOB->AFR[1] |= 0x05 << GPIO_AFRH_AFSEL13_Pos;
	
	
	// Set PB14 and PC0 to output mode
	GPIOB->MODER |= (1 << 28);
	GPIOB->MODER &= ~(1 << 29);
	GPIOC->MODER |= (1 << 0);
	GPIOC->MODER &= ~(1 << 1);
	// Set to push pull output type
	GPIOB->OTYPER &= ~(1 << 14);
	GPIOC->OTYPER &= ~(1 << 0);
	// Set to low speed
	//GPIOB->OSPEEDR &= ~((1 << 28) | (1 << 29) | (1 << 0) | (1 << 1));
	// Set to no pullup/down resistor
	//GPIOB->PUPDR &= ~((1 << 28) | (1 << 29) | (1 << 0) | (1 << 1));
	// Set pins to high
	GPIOB->ODR |= (1 << 14);
	GPIOC->ODR |= (1 << 0);
	
	
	//Set the parameters in the TIMINGR register to use 100 kHz standard-mode I2C.

	//Set PRESC to 1
	I2C2->TIMINGR &= ~((1 << 31) | (1 << 30) | (1 << 29));
	I2C2->TIMINGR |= (1 << 28);
	
	//Set SCLL to 0x13 = 0001 0100
	I2C2->TIMINGR &= ~((1 << 7) | (1 << 6) | (1 << 5) | (1 << 3) | (1 << 1) | (1 << 0));
	I2C2->TIMINGR |= (1 << 5) | (1 << 3);
	
	//Set SCLH to 0xF = 0000 1111
	I2C2->TIMINGR &= ~((1 << 15) | (1 << 14) | (1 << 13) | (1 << 12));
	I2C2->TIMINGR |= (1 << 11) | (1 << 10) | (1 << 9) | (1 << 8);
	
	//Set SDADEL to 0x2 = 0010
	I2C2->TIMINGR &= ~((1 << 19) | (1 << 18) | (1 << 16));
	I2C2->TIMINGR |= (1 << 17);
	
	//Set SCLDEL to 0x4 = 0100
	I2C2->TIMINGR &= ~((1 << 23) | (1 << 21) | (1 << 20));
	I2C2->TIMINGR |= (1 << 22);

	//Enable the I2C peripheral using the PE bit in the CR1 register.
	I2C2->CR1 |= (1 << 0);
}
	
	
void transmitI2C(void) {
//	Set the transaction parameters in the CR2 register. (See section 5.5.3)

	//Set the L3GD20 slave address SADD[7:1] = 0x69 = 00 110 1001 0
	I2C2->CR2 |= (1 << 7) | (1 << 6) | (1 << 4) | (1 << 1);

	// Set the number of bytes to transmit = 1
	I2C2->CR2 |= (1 << 16);

	// Set the RD_WRN bit to indicate a write operation.
	I2C2->CR2 &= ~(1 << 10);
	
	// Set the START bit
	I2C2->CR2 |= (1 << 13);


	//Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set.
	while(1) {
		//If NACKF flag is set, light up blue LED and halt program
		if (I2C2->ISR & (1 << 4)){
			GPIOC->ODR |= (1 << 7); //Turn on blue LED
			while (1) {}
		}
		//If TXIS flag is set, continue
		if (I2C2->ISR & (1 << 1)){
			break;
		} 
		HAL_Delay(100); // Delay 10ms
	}
	
	

	//Write the address of the “WHO_AM_I” register into the I2C transmit register (0x0F). (TXDR)
	I2C2->TXDR |= (0x0F << 0);
	
	
	//Wait until the TC (Transfer Complete) flag is set.
	while(1) {
		if (I2C2->ISR & (1 << 6))
			break;
		HAL_Delay(10);
	}
	
	
	//Reload the CR2 register with the same parameters as before, but set the RD_WRN bit to indicate a read operation.
	I2C2->CR2 |= (1 << 7) | (1 << 6) | (1 << 4) | (1 << 1); //Set the L3GD20 slave address SADD[7:1] = 0x69 = 00 110 1001 0
	I2C2->CR2 |= (1 << 16); // Set the number of bytes to read = 1
	I2C2->CR2 |= (1 << 10); // Set the RD_WRN bit to indicate a read operation.
	I2C2->CR2 |= (1 << 13); // Set the START bit
	
	//Wait until either of the RXNE (Receive Register Not Empty) or NACKF (Slave Not-Acknowledge) flags are set.
	while(1) {
		//If NACKF flag is set, light up orange LED and halt program
		if (I2C2->ISR & (1 << 4)){
			GPIOC->ODR |= (1 << 8); //Turn on orange LED
			while (1) {}
		}
		//If RXNE flag is set, continue
		if (I2C2->ISR & (1 << 2)){
			break;
		} 
		HAL_Delay(10); // Delay 10ms
	}
	
	//Wait until the TC (Transfer Complete) flag is set.
	while(1) {
		if (I2C2->ISR & (1 << 6))
			break;
		HAL_Delay(10);
	}

	//Check the contents of the RXDR register to see if it matches 0xD4. (expected value of the “WHO_AM_I” register)
	if (I2C2->RXDR & (0xD3 << 0))
		GPIOC->ODR |= (1 << 6); //Turn on red LED
	
	//Set the STOP bit in the CR2 register to release the I2C bus.
	I2C2->CR2 |= (1 << 14);
	
}

//This function edits the Gyro control register 1 to initialize it
void initializeGyro(void) {
	
	I2C2->CR2 |= (1 << 7) | (1 << 6) | (1 << 4) | (1 << 1); //Set the L3GD20 slave address SADD[7:1] = 0x69 = 00 110 1001 0
	I2C2->CR2 |= (1 << 17); // Set the number of bytes to transmit = 2
	I2C2->CR2 &= ~(1 << 16); // Set the number of bytes to transmit = 2
	I2C2->CR2 &= ~(1 << 10); // Set the RD_WRN bit to indicate a write operation.
	I2C2->CR2 |= (1 << 13); // Set the START bit
	
	//Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set.
	while(1) {
		//If NACKF flag is set, light up blue LED and halt program
		if (I2C2->ISR & (1 << 4)){
			GPIOC->ODR |= (1 << 7); //Turn on blue LED
			while (1) {}
		}
		//If TXIS flag is set, continue
		if (I2C2->ISR & (1 << 1)){
			break;
		} 
		HAL_Delay(100); // Delay 10ms
	}
	
	//Write the address of the “CTRL_REG1” register into the I2C transmit register (0x20).
	I2C2->TXDR |= (0x20 << 0);
	
	//Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set.
	while(1) {
		//If NACKF flag is set, light up blue LED and halt program
		if (I2C2->ISR & (1 << 4)){
			GPIOC->ODR |= (1 << 7); //Turn on blue LED
			while (1) {}
		}
		//If TXIS flag is set, continue
		if (I2C2->ISR & (1 << 1)){
			break;
		} 
		HAL_Delay(100); // Delay 10ms
	}
	
	//Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set.
	while(1) {
		//If NACKF flag is set, light up blue LED and halt program
		if (I2C2->ISR & (1 << 4)){
			GPIOC->ODR |= (1 << 7); //Turn on blue LED
			while (1) {}
		}
		//If TXIS flag is set, continue
		if (I2C2->ISR & (1 << 1)){
			break;
		} 
		HAL_Delay(100); // Delay 10ms
	}
	
	//Write value to “CTRL_REG1” register to enable X and Y axes only, set into normal or sleep mode, and set all other bits to zero (0000 1011)
	I2C2->TXDR |= (0x0B << 0);
	
	//Wait until the TC (Transfer Complete) flag is set.
	while(1) {
		if (I2C2->ISR & (1 << 6))
			break;
		HAL_Delay(100);
	}
	
	GPIOC->ODR |= (1 << 6); //Turn on red LED
	
	//Set the STOP bit in the CR2 register to release the I2C bus.
	I2C2->CR2 |= (1 << 14);
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


	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable peripheral clock to GPIOB
	
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; // Enable peripheral clock to I2C2
	
	setupLED(); //Set up LED GPIOC pins
	
	GPIOC->ODR |= (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9);
	HAL_Delay(500); // Delay 500ms
	GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
	HAL_Delay(500); // Delay 500ms
	
	setupI2C(); //Set up I2C
	
	transmitI2C(); //Transmit I2C
	
	HAL_Delay(500); // Delay 500ms
	GPIOC->ODR &= ~(1 << 6); //Turn off red LED
	
	initializeGyro(); //initialize gyroscope
	
	HAL_Delay(500); // Delay 500ms
	GPIOC->ODR &= ~(1 << 6); //Turn off red LED
	
	
	
	
	
	

	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
