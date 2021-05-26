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
#include <errno.h>
#include <sys/unistd.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TOUCHADRESS_READ 0x4B
#define TOUCHADRESS_WRITE 0x4A
#define TOUCHSCALING_X 372
#define TOUCHSCALING_Y 341
#define POTVALUE_SCALING 23
#define DELAYCOUNT 50


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void enableInterrupt(void);
void HAL_GPIO_EXTI_Callback(uint16_t);

void initializePeripherals(void);
void initializeLedDriver(void);
void initializeTouchController(void);

void processTouch(void);
uint8_t readTouches(uint8_t[], uint16_t);
void getCoordinatesLastTouch(uint16_t[]);
void writeToTouchController(uint8_t[],uint16_t);

void setLedValues(void);
void writeToLedsRegister(uint8_t[]);

void setPotValues(void);
void writeToPotentiometers(uint8_t[],uint16_t, int);



void __attribute__((naked)) SysTickDelayCount(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne     SysTickDelayCount\n"
          "    bx      lr");
}

int _write(int file, char *ptr, int len) {
    HAL_StatusTypeDef xStatus;
    switch (file) {
    case STDOUT_FILENO: /*stdout*/
		xStatus = HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
		if (xStatus != HAL_OK) {
			errno = EIO;
			return -1;
		}
        break;
    case STDERR_FILENO: /* stderr */
		xStatus = HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
		if (xStatus != HAL_OK) {
			errno = EIO;
			return -1;
		}
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return len;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Values of leds kept, every value is value from 0-12 for each sliders led array


uint8_t ledValues[11];
uint8_t potValues[20];
uint16_t touchCoordinates[2];
int isTouchInitialized = 0;

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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //Initialization of Potvalues to 0

  for(int i=0;i<20;i++){
	  potValues[i]=0xFF;
  }

  //L-R balance default op mid zetten
  potValues[17]=128;

  //initialization of LedValues to 0
  for(int i=0;i<11;i++){
	  ledValues[i]=0x00;
  }
  HAL_Delay(50);

  initializePeripherals();

  //led test

  uint8_t data[4];

  	 /*data[0]=0x0F; //LED display test, all on
    data[1]=0x00;
    data[2]=0x0F;
    data[3]=0x00;

    writeToLedsRegister(data);
*/


  initializeLedDriver();


  enableInterrupt();


 HAL_Delay(10);

 setPotValues();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*data[0]=0x0F; //LED display test, all on
	      data[1]=0x01;
	      data[2]=0x0F;
	      data[3]=0x01;

	      writeToLedsRegister(data);
	  */


	  initializeLedDriver();

	  data[0]=0x01; //LED display test, all on
	  data[1]=0xFF;
	  data[2]=0x01;
	  data[3]=0xFF;

	  writeToLedsRegister(data);

	  HAL_Delay(200);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00C0EAFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_POT1_GPIO_Port, CS_POT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, CS_POT5_Pin|CS_POT3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, CS_POT2_Pin|CS_LEDS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_POT4_GPIO_Port, CS_POT4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_POT1_Pin */
  GPIO_InitStruct.Pin = CS_POT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_POT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_POT5_Pin CS_POT3_Pin */
  GPIO_InitStruct.Pin = CS_POT5_Pin|CS_POT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_INT_Pin */
  GPIO_InitStruct.Pin = TOUCH_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_POT2_Pin CS_LEDS_Pin */
  GPIO_InitStruct.Pin = CS_POT2_Pin|CS_LEDS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_POT4_Pin */
  GPIO_InitStruct.Pin = CS_POT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_POT4_GPIO_Port, &GPIO_InitStruct);


}

/* USER CODE BEGIN 4 */


void enableInterrupt(){
	/* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	printf("interrupt\r\n");

	if(GPIO_Pin == TOUCH_INT_Pin){

		//first time interrupt is generated is for initialization for touchcontroller
		if(!isTouchInitialized){
			initializeTouchController();
			isTouchInitialized = 1;
		}else{

			//interrupt generated by touchcontroller, should read out last touch now
			getCoordinatesLastTouch(touchCoordinates);
			//use touchcoordinates to write to LEDs & pots

			processTouch();
			//converting touchCoordinates to usable values for LEDs & pots

		}
	}
}


void initializePeripherals(){
	HAL_GPIO_WritePin(CS_LEDS_GPIO_Port,CS_LEDS_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_POT1_GPIO_Port,CS_POT1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_POT2_GPIO_Port,CS_POT2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_POT3_GPIO_Port,CS_POT3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_POT4_GPIO_Port,CS_POT4_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_POT5_GPIO_Port,CS_POT5_Pin,GPIO_PIN_SET);

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


	//Intensity setting; register 0x0A
	//by default both set to max intensity, can be adjusted if some are brighter than others
	data[0]=0x0A;
	data[1]=0x0F;
	data[2]=0x0A;
	data[3]=0x0F;
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

	//enabling shutdown register
	data[0]=0x0C;
	data[1]=0x01;		//writing 0x01 to enable
	data[2]=0x0C;
	data[3]=0x01;
	writeToLedsRegister(data);

}



void initializeTouchController(){
	//Touch controller (MTCH6301)

	uint8_t data[5];

	//following guide to initialize component

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
	data[4]=0x02; //offset location 							here: flipstate		: 0x02
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




void processTouch(){
	/* no arguments needed, bc all values used and written to
	 * are saved as global variables.
	 * That being:
	 *  touchCoo, ledValues & potValues
	 */

	//printf("Touchcoordinates (x,y)\r\n%x\r\n%x\r\n",touchCoordinates[0],touchCoordinates[1]);
	//depending on these values, needs to be converted into led values & pot values

	//Touchcontroller get values from 0->4096 for both axis

	int i=0; //used to loop
	int j=0;
	uint8_t slider=50; //give a random value that will never be reached
	uint8_t sliderheight=50;


	//checking for slider with loop
	while(i<11 && slider==50){
		if(i*TOUCHSCALING_X < touchCoordinates[0] && touchCoordinates[0] < (i+1) * TOUCHSCALING_X){
			slider = i;
		}
		i++;
	}

	//default value if something went wrong
	if(slider==50){
		slider=0;
	}


	//checking for heightvalue with loop
	while(j<12 && sliderheight ==50){
		if(j*TOUCHSCALING_Y < touchCoordinates[1] && touchCoordinates[1] <(j+1)*TOUCHSCALING_Y){
			sliderheight = j;
		}
		j++;
	}

	//default value if something went wrong
	if(sliderheight==50){
		sliderheight=0;
	}

	printf("Slider: %d \r\n",slider);
	printf("Sliderheight: %d \r\n",sliderheight);




	//Setting LED values to relevant value
	switch(sliderheight){
		case 0:
			ledValues[slider]=0b00000001;
			break;
		case 1:
			ledValues[slider]=0b00000011;
			break;
		case 2:
			ledValues[slider]=0b00000111;
			break;
		case 3:
			ledValues[slider]=0b00001111;
			break;
		case 4:
			ledValues[slider]=0b00011111;
			break;
		case 5:
			ledValues[slider]=0b00111111;
			break;
		case 6:
			ledValues[slider]=0b01111111;
			break;
		case 7:
			ledValues[slider]=0b11111111;
			break;

		//for cases higher, also just write max value
		case 8:
			ledValues[slider]=0b11111111;
			break;
		case 9:
			ledValues[slider]=0b11111111;
			break;
		case 10:
			ledValues[slider]=0b11111111;
			break;
		case 11:
			ledValues[slider]=0b11111111;
			break;

		default:
			ledValues[slider]=0b00000000;
			break;
	}




	//setting potValues to relevant values

	//SLIDERS NOG AAN TE PASSEN!!!! -> naar welke pot meter er relevant is voor welke slider
	switch(slider){
		case 0:		//BASS LINE1 -> POT6 & POT11
			potValues[5]= POTVALUE_SCALING * (uint8_t)sliderheight;
			potValues[10]= POTVALUE_SCALING * (uint8_t)sliderheight;
			break;
		case 1:		//MID LINE1 -> POT7 & POT10
			potValues[6]= POTVALUE_SCALING * (uint8_t)sliderheight;
			potValues[9]= POTVALUE_SCALING * (uint8_t)sliderheight;
			break;
		case 2:		//TREBLE LINE1 -> POT8 & POT9
			potValues[7]= POTVALUE_SCALING * (uint8_t)sliderheight;
			potValues[8]= POTVALUE_SCALING * (uint8_t)sliderheight;
			break;
		case 3:		//VOL LINE1 -> POT2 & POT3
			potValues[1]= POTVALUE_SCALING * (uint8_t)sliderheight;
			potValues[2]= POTVALUE_SCALING * (uint8_t)sliderheight;
			break;
		case 4:		//VOL LINE2 -> POT4 & POT5
			potValues[3]= POTVALUE_SCALING * (uint8_t)sliderheight;
			potValues[4]= POTVALUE_SCALING * (uint8_t)sliderheight;
			break;
		case 5:		//BASS LINE2 ->POT12 & POT17
			potValues[11]= POTVALUE_SCALING * (uint8_t)sliderheight;
			potValues[16]= POTVALUE_SCALING * (uint8_t)sliderheight;
			break;
		case 6:		//MID LINE2 -> POT13 & POT16
			potValues[12]= POTVALUE_SCALING * (uint8_t)sliderheight;
			potValues[15]= POTVALUE_SCALING * (uint8_t)sliderheight;
			break;
		case 7:		//TREBLE LINE2 -> POT14 & POT15
			potValues[13]= POTVALUE_SCALING * (uint8_t)sliderheight;
			potValues[14]= POTVALUE_SCALING * (uint8_t)sliderheight;
			break;
		case 8:		//MIC VOL -> POT1
			potValues[0]= POTVALUE_SCALING * (uint8_t)sliderheight;
			break;
		case 9:		//MASTER OUT -> POT19 & POT20
			potValues[18]= POTVALUE_SCALING * (uint8_t)sliderheight;
			potValues[19]= POTVALUE_SCALING * (uint8_t)sliderheight;
			break;
		case 10:	//BALANCE L-R -> POT18
			potValues[17]= POTVALUE_SCALING * (uint8_t)sliderheight;
			break;
		default:
			//niks doen
			break;

	}




	//Call functions that handle writing to IC's
	setLedValues();
	setPotValues();

}



uint8_t readTouches(uint8_t databuf[], uint16_t amount_bytes_expected){
	/** reads via i2c, can only read when controller throws interrupt
	 *     ------nog aan te passen!!!
	 */
	HAL_I2C_Master_Receive(&hi2c1,TOUCHADRESS_READ,databuf,amount_bytes_expected,50);
	uint8_t bytesToReceive = (uint8_t)(databuf[0]);

	return bytesToReceive;
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

void writeToTouchController(uint8_t data[],uint16_t amountData){
	HAL_I2C_Master_Transmit(&hi2c1,TOUCHADRESS_WRITE,data,amountData,1);
	SysTickDelayCount(DELAYCOUNT);
	//HAL_I2C_Mem_Write(&hi2c1,TOUCHADRESS_WRITE,memAddress,memAddressSize,data,amountData,50);
}






void setLedValues(){
	//Decoding led values to instructions for led drivers

	/* Noticed a flaw in way led-matrix is set up,
	 * will very likely not be possible to drive LEDs of sliders 9,10 or 11,
	 * neither will LED layers 9,10,11 and 12 be drivable.
	 * There will still be LEDs drivable on Sliders 1-8,
	 *  and of those Led layers 1-8
	 *
	 *
	 *  Data stored is in order:
	 *  7:0 (led layers)
	 *  D7,D0,D1,D2, D3,D4,D5,D6
	 */

	/*  looping over different ledsliders
	 *  LedSlider Mapping : see below
	 */

	uint8_t data[4]; //used to write data out
	uint8_t ledSliderMap[8]= {0x01,0x02,0x03,0x04, 0x05,0x06,0x07,0x08};
	for(int ledSlider=0;ledSlider<8;ledSlider++){

		//easiest to write these manually
		uint8_t ledValue=0x00;
		ledValue = ledValue | ( ledValues[ledSlider] & 0b10000000); 		//writing bit 7
		ledValue = ledValue | ((ledValues[ledSlider] & 0b00000001)<<6);		//writing bit 6
		ledValue = ledValue | ((ledValues[ledSlider] & 0b00000010)<<4);		//writing bit 5
		ledValue = ledValue | ((ledValues[ledSlider] & 0b00000100)<<2);		//writing bit 4
		ledValue = ledValue | ( ledValues[ledSlider] & 0b00001000);			//writing bit 3
		ledValue = ledValue | ((ledValues[ledSlider] & 0b00010000)>>2);		//writing bit 2
		ledValue = ledValue | ((ledValues[ledSlider] & 0b00100000)>>4);		//writing bit 1
		ledValue = ledValue | ((ledValues[ledSlider] & 0b01000000)>>6);		//writing bit 0

		data[0]=ledSliderMap[ledSlider];
		data[1]=ledValue;
		data[2]=0x00; //no-op code, second controller doesn't need to display bc of problem noted above
		data[3]=0x00;
		writeToLedsRegister(data);
	}
}


void writeToLedsRegister(uint8_t data[]){
	/**
	 * uses SPI2 of µc
	 * 2 ic are daisychained after each other
	 * CS is active low, so put it high by default
	 *
	 * Both chips are controlled by daisychaining data; each chip needs 16 bit data,
	 * so two 16 bit values are sent after each other, then CS is pulled high
	*/


	HAL_GPIO_WritePin(CS_LEDS_GPIO_Port, CS_LEDS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,data,4,1);
	HAL_GPIO_WritePin(CS_LEDS_GPIO_Port, CS_LEDS_Pin, GPIO_PIN_SET);
	SysTickDelayCount(DELAYCOUNT);

}



void setPotValues(){
	uint8_t data[2];
	uint8_t addressMap[4]={0b00000000, 0b00000001, 0b00000010, 0b00000011};

	for(int pot=0;pot<5;pot++){
		for(int pot_comp=0;pot_comp<4;pot_comp++){
			data[0]=addressMap[pot_comp];
			data[1]=potValues[pot*4 + pot_comp];
			writeToPotentiometers(data,2,pot+1);
		}
	}
}


void writeToPotentiometers(uint8_t data[], uint16_t amountBytes,  int pot_ic){
	/**
	 * AD5204
	 * uses SPI1 of µc
	 * 5 different pot_ics, each containing 4 potentiometers.
	 * each pot_ic is addressed with corresponding CS_POTx
	 *
	 *  When writing data to pots, only 11 bits are needed.
	 *  3 bits for pot address, 8 bits for value.
	 *  We'll send 2 bytes, first byte send having 5 0's appended first, as they will get shifted out anyway.
	 *
	 *
	 *  No initialization for pots is needed
	 */

	switch(pot_ic)
	{
		case 1:
			HAL_GPIO_WritePin(CS_POT1_GPIO_Port,CS_POT1_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2,data,amountBytes,1);
			HAL_GPIO_WritePin(CS_POT1_GPIO_Port,CS_POT1_Pin, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(CS_POT2_GPIO_Port,CS_POT2_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2,data,amountBytes,1);
			HAL_GPIO_WritePin(CS_POT2_GPIO_Port,CS_POT2_Pin, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(CS_POT3_GPIO_Port,CS_POT3_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2,data,amountBytes,1);
			HAL_GPIO_WritePin(CS_POT3_GPIO_Port,CS_POT3_Pin, GPIO_PIN_SET);
			break;
		case 4:
			HAL_GPIO_WritePin(CS_POT4_GPIO_Port,CS_POT4_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2,data,amountBytes,1);
			HAL_GPIO_WritePin(CS_POT4_GPIO_Port,CS_POT4_Pin, GPIO_PIN_SET);
			break;
		case 5:
			HAL_GPIO_WritePin(CS_POT5_GPIO_Port,CS_POT5_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2,data,amountBytes,1);
			HAL_GPIO_WritePin(CS_POT5_GPIO_Port,CS_POT5_Pin, GPIO_PIN_SET);
			break;
		default:
			printf("Error\r\n");
			break;

	}
	SysTickDelayCount(DELAYCOUNT);


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
