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

//This function sets up pins PC4 and PC5 for using UART3
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

void transmit8bits(int x) {
	setupUART3();
	sctransmit('\r');
	for (int i = 7; i >= 0; i = i - 1){
		int t = 0;
		if (x & (1 << i))
			t = 1;
		inttransmit(t);
	}
	sctransmit('\r');
	sctransmit('\n');
}

//Z = XY
void transmit16bits(int x, int y) {
	int z = (x << 8) | y;
	setupUART3();
	sctransmit('\r');
	for (int i = 15; i >= 0; i = i - 1){
		int t = 0;
		if (z & (1 << i))
			t = 1;
		inttransmit(t);
	}
	sctransmit('\r');
	sctransmit('\n');
}

void transmit16bits2(int x) {
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

//This function takes in an array of characters and transmits it from USART3_TX
void string_transmit (char a[]) {
	sctransmit('\r');
	int counter = 0;
	while (1) {
		if (a[counter] != 0) {
			sctransmit(a[counter]);
			counter = counter + 1;
		}
		else
			break;
	}
	sctransmit('\n');
	sctransmit('\r');
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
	I2C2->CR2 &= ~(1 << 17);

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
		HAL_Delay(1); // Delay 10ms
	}
	
	

	//Write the address of the “WHO_AM_I” register into the I2C transmit register (0x0F). (TXDR)
	I2C2->TXDR = (0x0F << 0);
	
	
	//Wait until the TC (Transfer Complete) flag is set.
	while(1) {
		if (I2C2->ISR & (1 << 6))
			break;
		HAL_Delay(1);
	}
	
	
	//Reload the CR2 register with the same parameters as before, but set the RD_WRN bit to indicate a read operation.
	I2C2->CR2 |= (1 << 7) | (1 << 6) | (1 << 4) | (1 << 1); //Set the L3GD20 slave address SADD[7:1] = 0x69 = 00 110 1001 0
	I2C2->CR2 |= (1 << 16); // Set the number of bytes to read = 1
	I2C2->CR2 &= ~(1 << 17); // Set the number of bytes to read = 1
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
		HAL_Delay(1); // Delay 10ms
	}
	
	//Wait until the TC (Transfer Complete) flag is set.
	while(1) {
		if (I2C2->ISR & (1 << 6))
			break;
		HAL_Delay(1);
	}

	
	

	
	
	
	
	//Check the contents of the RXDR register to see if it matches 0xD4. (expected value of the “WHO_AM_I” register)
	if (I2C2->RXDR & (0xD3 << 0))
		string_transmit("RXDR = 0xD3");
	
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
			string_transmit("NACK1");
			while (1) {}
		}
		//If TXIS flag is set, continue
		if (I2C2->ISR & (1 << 1)){
			string_transmit("PASS1");
			break;
		} 
		HAL_Delay(10); // Delay 10ms
	}
	
	//Write the address of the “CTRL_REG1” register into the I2C transmit register (0x20).
	I2C2->TXDR = (0x20 << 0);
	
	//Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set.
	while(1) {
		//If NACKF flag is set, light up blue LED and halt program
		if (I2C2->ISR & (1 << 4)){
			string_transmit("NACK2");
			while (1) {}
		}
		//If TXIS flag is set, continue
		if (I2C2->ISR & (1 << 1)){
			string_transmit("PASS2");
			break;
		} 
		HAL_Delay(10); // Delay 10ms
	}
	
	

	
	//Write value to “CTRL_REG1” register to enable X and Y axes only, set into normal or sleep mode, and set all other bits to zero (0000 1011)
	I2C2->TXDR = (0x0B << 0);
	
	while(1) {
		//If NACKF flag is set, light up blue LED and halt program
		if (I2C2->ISR & (1 << 4)){
			string_transmit("NACK3");
			while (1) {}
		}
		//If TXIS flag is set, continue
		if (I2C2->ISR & (1 << 1)){
			string_transmit("PASS3");
			break;
		} 
		if (I2C2->ISR & (1 << 6)){
			string_transmit("PASS4");
			break;
		}
		HAL_Delay(100); // Delay 10ms
	}
	
	//Wait until the TC (Transfer Complete) flag is set.
	while(1) {
		if (I2C2->ISR & (1 << 6))
			break;
		HAL_Delay(100);
		GPIOC->ODR ^= (1 << 9); //Turn on red LED
	}
	
	GPIOC->ODR |= (1 << 6); //Turn on red LED
	
	//Set the STOP bit in the CR2 register to release the I2C bus.
	I2C2->CR2 |= (1 << 14);
}



int readGyroX(void) {
	//	Set the transaction parameters in the CR2 register. (See section 5.5.3)

	//Set the L3GD20 slave address SADD[7:1] = 0x69 = 00 110 1001 0
	I2C2->CR2 |= (1 << 7) | (1 << 6) | (1 << 4) | (1 << 1);

	// Set the number of bytes to transmit = 1
	I2C2->CR2 |= (1 << 16);
	I2C2->CR2 &= ~(1 << 17);

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
		HAL_Delay(1); // Delay 10ms
	}
	
	

	//Write the address of the “WHO_AM_I” register into the I2C transmit register (0x0F). (TXDR)
	I2C2->TXDR = (0xA8 << 0);
	
	
	//Wait until the TC (Transfer Complete) flag is set.
	while(1) {
		if (I2C2->ISR & (1 << 6))
			break;
		HAL_Delay(1);
	}
	
	
	//Reload the CR2 register with the same parameters as before, but set the RD_WRN bit to indicate a read operation.
	I2C2->CR2 |= (1 << 7) | (1 << 6) | (1 << 4) | (1 << 1); //Set the L3GD20 slave address SADD[7:1] = 0x69 = 00 110 1001 0
	I2C2->CR2 &= ~(1 << 16); // Set the number of bytes to read = 2
	I2C2->CR2 |= (1 << 17); // Set the number of bytes to read = 2
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
		HAL_Delay(1); // Delay 10ms
	}
	
	//string_transmit("read 1");
	int r1 = I2C2->RXDR;
	//transmit8bits(r1);
	
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
	
	//string_transmit("read 2");
	int r2 = I2C2->RXDR;
	//transmit8bits(r2);
	
	//Wait until the TC (Transfer Complete) flag is set.
	while(1) {
		if (I2C2->ISR & (1 << 6))
			break;
		HAL_Delay(10);
	}

	//transmit16bits(r2, r1);
	
	
	//Set the STOP bit in the CR2 register to release the I2C bus.
	I2C2->CR2 |= (1 << 14);
	int out = (r2 << 8) | r1;
	return out;
	
}






int readGyroY(void) {
	//	Set the transaction parameters in the CR2 register. (See section 5.5.3)

	//Set the L3GD20 slave address SADD[7:1] = 0x69 = 00 110 1001 0
	I2C2->CR2 |= (1 << 7) | (1 << 6) | (1 << 4) | (1 << 1);

	// Set the number of bytes to transmit = 1
	I2C2->CR2 |= (1 << 16);
	I2C2->CR2 &= ~(1 << 17);

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
		HAL_Delay(1); // Delay 10ms
	}
	
	

	//Write the address of the “WHO_AM_I” register into the I2C transmit register (0x0F). (TXDR)
	I2C2->TXDR = (0xAA << 0);
	
	
	//Wait until the TC (Transfer Complete) flag is set.
	while(1) {
		if (I2C2->ISR & (1 << 6))
			break;
		HAL_Delay(1);
	}
	
	
	//Reload the CR2 register with the same parameters as before, but set the RD_WRN bit to indicate a read operation.
	I2C2->CR2 |= (1 << 7) | (1 << 6) | (1 << 4) | (1 << 1); //Set the L3GD20 slave address SADD[7:1] = 0x69 = 00 110 1001 0
	I2C2->CR2 &= ~(1 << 16); // Set the number of bytes to read = 2
	I2C2->CR2 |= (1 << 17); // Set the number of bytes to read = 2
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
		HAL_Delay(1); // Delay 10ms
	}
	
	//string_transmit("read 1");
	int r1 = I2C2->RXDR;
	//transmit8bits(r1);
	
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
	
	//string_transmit("read 2");
	int r2 = I2C2->RXDR;
	//transmit8bits(r2);
	
	//Wait until the TC (Transfer Complete) flag is set.
	while(1) {
		if (I2C2->ISR & (1 << 6))
			break;
		HAL_Delay(10);
	}

	//transmit16bits(r2, r1);
	
	
	//Set the STOP bit in the CR2 register to release the I2C bus.
	I2C2->CR2 |= (1 << 14);
	int out = (r2 << 8) | r1;
	return out;
	
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

	string_transmit("Starting Initialization...");
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable peripheral clock to GPIOB
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable peripheral clock to USART3
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; // Enable peripheral clock to I2C2
	
	setupLED(); //Set up LED GPIOC pins
	setupUART3(); //Set up UART
	string_transmit("LED and UART Setup Complete");
	setupI2C(); //Set up I2C
	string_transmit("I2C Setup Complete");
	transmitI2C(); //Transmit I2C
	string_transmit("I2C WHO AM I check complete");
	initializeGyro(); //initialize gyroscope
	string_transmit("Gyroscope initialization complete");

	GPIOC->ODR &= ~(1 << 6); //Turn off red LED
	
	

	
//	int count = 0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
		string_transmit("while loop start");
//		string_transmit("gyro:");
//    transmit16bits2(readGyroX());
//		transmit16bits2(readGyroY());
		int readX = readGyroX();
		string_transmit("readX");
		int readY = readGyroY();
		string_transmit("readY");
		
		
		
		

		if (readY > (0xFFFF/2)){
			GPIOC->ODR |= (1 << 7); //Turn on blue LED
			GPIOC->ODR &= ~(1 << 6); //Turn off red LED
		}
		else {
			GPIOC->ODR &= ~(1 << 7); //Turn off blue LED
			GPIOC->ODR |= (1 << 6); //Turn on red LED
		}

		if (readX > (0xFFFF/2)){
			GPIOC->ODR &= ~(1 << 9); //Turn off green LED
			GPIOC->ODR |= (1 << 8); //Turn on orange LED
		}
		else {
			GPIOC->ODR |= (1 << 9); //Turn on green LED
			GPIOC->ODR &= ~(1 << 8); //Turn off orange LED
		}
		
		

		
		
		HAL_Delay(100); // Delay 500ms
		string_transmit("while loop complete");
//    count = count + 1;
//		if (count > 14)
//			break;
  }
	string_transmit("while loop complete");
  
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
