/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

#define TOUCHADRESS_READ 0x4B
#define TOUCHADRESS_WRITE 0x4A
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

void writeToLedsRegister(uint8_t[]);
void writeToPotentiometers(uint8_t[],uint16_t, int);
void initializePeripherals(void);
uint8_t readTouches(uint8_t[], uint16_t);
void getCoordinatesLastTouch(uint16_t[]);
void writeToTouchController(uint8_t[],uint16_t);
void initializeLedDriver(void);
void initializeTouchController(void);
void setLedValues(uint8_t[]);
void HAL_GPIO_EXTI_Callback(uint16_t);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Values of leds kept, every value is value from 0-12 for each sliders led array
uint8_t ledValues[11];
uint16_t touchCoordinates[2];


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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SD_Step_Up_Pin|CS_LEDS_Pin|GPIO_PIN_12|CS_POT3_Pin
                          |CS_POT2_Pin|CS_POT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_POT5_Pin|CS_POT4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_Step_Up_Pin CS_LEDS_Pin PB12 CS_POT3_Pin
                           CS_POT2_Pin CS_POT1_Pin */
  GPIO_InitStruct.Pin = SD_Step_Up_Pin|CS_LEDS_Pin|GPIO_PIN_12|CS_POT3_Pin
                          |CS_POT2_Pin|CS_POT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_POT5_Pin CS_POT4_Pin */
  GPIO_InitStruct.Pin = CS_POT5_Pin|CS_POT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0){
		//interrupt generated by touchcontroller, should read out last touch now

		getCoordinatesLastTouch(touchCoordinates);
		//use touchcoordinates to write to LEDs & pots

	}

}



void getCoordinatesLastTouch(uint16_t touchCo[]){
	uint8_t databuf[20];
	readTouches(databuf,6); //6 expected

	/**byte 0 = amount total bytes
	 * byte 1 = TOUCHID, not really relevant
	 * byte 2 = part of touch X coordinate
	 * byte 3 = other part of touch X coordinate
	 * byte 4 = part of touch Y coordinate
	 * byte 5 = other part of touch Y coordinate
	 */

	//writing X out
	touchCo[0] = (((uint16_t)databuf[3])<<7) + ((uint16_t)databuf[2]);

	//writing Y out
	touchCo[1] = (((uint16_t)databuf[5])<<7) + ((uint16_t)databuf[4]);
}



uint8_t readTouches(uint8_t databuf[], uint16_t amount_bytes_expected){
	/** reads via i2c, can only read when controller throws interrupt
	 *     ------nog aan te passen!!!
	 */
	HAL_I2C_Master_Receive(&hi2c1,TOUCHADRESS_READ,databuf,amount_bytes_expected,50);
	uint8_t bytesToReceive = (uint8_t)(databuf[0]);

	return bytesToReceive;
}

void writeToTouchController(uint8_t data[],uint16_t amountData){
	HAL_I2C_Master_Transmit(&hi2c1,TOUCHADRESS_WRITE,data,amountData,50);
	//HAL_I2C_Mem_Write(&hi2c1,TOUCHADRESS_WRITE,memAddress,memAddressSize,data,amountData,50);
}




void initializePeripherals(){
	HAL_GPIO_WritePin(CS_LEDS_GPIO_Port,CS_LEDS_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_POT1_GPIO_Port,CS_POT1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_POT2_GPIO_Port,CS_POT2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_POT3_GPIO_Port,CS_POT3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_POT4_GPIO_Port,CS_POT4_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_POT5_GPIO_Port,CS_POT5_Pin,GPIO_PIN_SET);

}


void writeToLedsRegister(uint8_t data[]){
	/**
	 * uses SPI2 of �c
	 * 2 ic are daisychained after each other
	 * CS is active low, so put it high by default
	 *
	 * Both chips are controlled by daisychaining data; each chip needs 16 bit data,
	 * so two 16 bit values are sent after each other, then CS is pulled high
	*/


	HAL_GPIO_WritePin(CS_LEDS_GPIO_Port, CS_LEDS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,data,4,100);
	HAL_GPIO_WritePin(CS_LEDS_GPIO_Port, CS_LEDS_Pin, GPIO_PIN_SET);

}

void setLedValues(uint8_t data[]){
	//Decoding led values to instructions for led drivers





}


void initializeLedDriver(){
	//LED driver (MAX7219)
	uint8_t data[4];

	//set decode mode for both chips to no decode
	data[0]=0x09;
	data[1]=0x00;
	data[2]=0x09;
	data[3]=0x00;
	writeToLedsRegister(data);

	/**scan-limit mode, set to 8 digits for first one, 3 for other one,
	 * for a total of 11 "digits".
	 * Each "digit" represents a slider with leds,
	 * every segment represents a vertical led layer
	 */
	data[0]=0x0B;
	data[1]=0x07;
	data[0]=0x0B;
	data[1]=0x02;
	writeToLedsRegister(data);


}


void initializeTouchController(){
	//Touch controller (MTCH6301)

	uint8_t data[5];

	//following guide to initialize component
	//writing reset high/low --> to do

	//wait for low state int --> to do

	//disable touch cmd
	data[0]=0x55;
	data[1]=0x01; //# of bytes to send
	data[2]=0x01; //disable touch cmd
	writeToTouchController(data,3);

	//write parameters --> write to registers

	//RX channels
	data[0]=0x55;
	data[1]=0x04; //# of bytes to send
	data[2]=0x15; //write to register cmd
	data[3]=0x00; //index location (see datasheet registers) 	here: general		: 0x00
	data[4]=0x01; //offset location 							here: RX channels 	: 0x01
	data[5]=0x0B; //value to write 								here: 11 -> 		: 0x0B
	writeToTouchController(data,6);

	//TX channels
	data[0]=0x55;
	data[1]=0x04; //# of bytes to send
	data[2]=0x15; //write to register cmd
	data[3]=0x00; //index location (see datasheet registers) 	here: general		: 0x00
	data[4]=0x02; //offset location 							here: TX channels 	: 0x02
	data[5]=0x0C; //value to write 								here: 12 -> 		: 0x0C
	writeToTouchController(data,6);

	//RX scaling <7:0>
	data[0]=0x55;
	data[1]=0x04; //# of bytes to send
	data[2]=0x15; //write to register cmd
	data[3]=0x00; //index location (see datasheet registers) 	here: general			: 0x00
	data[4]=0x04; //offset location 							here: RX scaling <7:0>	: 0x04
	data[5]=0x45; //value to write 								here: 5957 -> 0x1745 -> : 0x45
	writeToTouchController(data,6);

	//RX scaling <15:8>
	data[0]=0x55;
	data[1]=0x04; //# of bytes to send
	data[2]=0x15; //write to register cmd
	data[3]=0x00; //index location (see datasheet registers) 	here: general			: 0x00
	data[4]=0x05; //offset location 							here: RX scaling <15:8>	: 0x05
	data[5]=0x17; //value to write 								here: 5957 -> 0x1745 -> : 0x17
	writeToTouchController(data,6);

	//TX scaling <7:0>
	data[0]=0x55;
	data[1]=0x04; //# of bytes to send
	data[2]=0x15; //write to register cmd
	data[3]=0x00; //index location (see datasheet registers) 	here: general			: 0x00
	data[4]=0x06; //offset location 							here: TX scaling <7:0>	: 0x06
	data[5]=0x55; //value to write 								here: 5461 -> 0x1555 -> : 0x55
	writeToTouchController(data,6);

	//TX scaling <15:8>
	data[0]=0x55;
	data[1]=0x04; //# of bytes to send
	data[2]=0x15; //write to register cmd
	data[3]=0x00; //index location (see datasheet registers) 	here: general			: 0x00
	data[4]=0x07; //offset location 							here: TX scaling <15:8>	: 0x07
	data[5]=0x15; //value to write 								here: 5461 -> 0x1555 -> : 0x15
	writeToTouchController(data,6);


	//Sensormapping

	uint8_t registermap[12] ={0x00,0x01,0x02,0x03, 0x04,0x05,0x06,0x07, 0x08,0x09,0x0A,0x0B};
	// RX map = {8,7,3,10, 6,1,5,0, 4,9,2}
	uint8_t RX_mapping[11]  ={0x08,0x07,0x03,0x0A, 0x06,0x01,0x05,0x00, 0x04,0x09,0x02};
	// TX map = {13,6,3,2, 4,30,29,28, 7,14,15,16}
	uint8_t TX_mapping[12]  ={0x0D,0x06,0x03,0x02, 0x04,0x1E,0x1D,0x1C, 0x07,0x0E,0x0F,0x10};


	//RX Mapping
	for(int i=0; i<11 ;i++){
		data[0]=0x55;
		data[1]=0x04; 					//# of bytes to send
		data[2]=0x15; 					//write to register cmd
		data[3]=0x01; 					//index location (see datasheet registers) 	here: sensor map RX	: 0x01
		data[4]=registermap[i]; 	  	//offset location 							here: RX map i		: see registermap above
		data[5]=RX_mapping[i]; 			//value to write 							here: RX value i 	: see RX mapping above
		writeToTouchController(data,6);
	}

	//TX Mapping

	for(int i=0; i<12 ;i++){
		data[0]=0x55;
		data[1]=0x04; 					//# of bytes to send
		data[2]=0x15; 					//write to register cmd
		data[3]=0x02; 					//index location (see datasheet registers) 	here: sensor map TX	: 0x02
		data[4]=registermap[i]; 	  	//offset location 							here: TX map i		: see registermap above
		data[5]=TX_mapping[i]; 			//value to write 							here: TX value i 	: see TX mapping above
		writeToTouchController(data,6);
	}




	//Decoding :

	//Flip State
	data[0]=0x55;
	data[1]=0x04; //# of bytes to send
	data[2]=0x15; //write to register cmd
	data[3]=0x30; //index location (see datasheet registers) 	here: decoding		: 0x30
	data[4]=0x00; //offset location 							here: flipstate		: 0x00
	data[5]=0x04; //value to write 								here: 0b100		 	: 0x04
	writeToTouchController(data,6);



	//Possibly extra values to change: subject to change



	//send scan baseline command
	data[0]=0x55;
	data[1]=0x01; //# of bytes to send
	data[2]=0x14; //scan baseline cmd
	writeToTouchController(data,3);


	//enable touch cmd
	data[0]=0x55;
	data[1]=0x01; //# of bytes to send
	data[2]=0x00; //enable touch cmd
	writeToTouchController(data,3);



}


void writeToPotentiometers(uint8_t data[], uint16_t amountBytes,  int pot_ic){
	/**
	 * uses SPI1 of �c
	 * 5 different pot_ics, each containing 4 potentiometers.
	 * each pot_ic is addressed with corresponding CS_POTx
	 */

	switch(pot_ic)
	{
		case 1:
			HAL_GPIO_WritePin(CS_POT1_GPIO_Port,CS_POT1_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,data,amountBytes,50);
			HAL_GPIO_WritePin(CS_POT1_GPIO_Port,CS_POT1_Pin, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(CS_POT2_GPIO_Port,CS_POT2_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,data,amountBytes,50);
			HAL_GPIO_WritePin(CS_POT2_GPIO_Port,CS_POT2_Pin, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(CS_POT3_GPIO_Port,CS_POT3_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,data,amountBytes,50);
			HAL_GPIO_WritePin(CS_POT3_GPIO_Port,CS_POT3_Pin, GPIO_PIN_SET);
			break;
		case 4:
			HAL_GPIO_WritePin(CS_POT4_GPIO_Port,CS_POT4_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,data,amountBytes,50);
			HAL_GPIO_WritePin(CS_POT4_GPIO_Port,CS_POT4_Pin, GPIO_PIN_SET);
			break;
		case 5:
			HAL_GPIO_WritePin(CS_POT5_GPIO_Port,CS_POT5_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,data,amountBytes,50);
			HAL_GPIO_WritePin(CS_POT5_GPIO_Port,CS_POT5_Pin, GPIO_PIN_SET);
			break;
		default:
			printf("Error\r\n");

	}


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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
