/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/**
 * @brief ECG 2 commands.
 * @details Specified commands for description of ECG 2 Click driver.
 */

    const uint8_t ECG2_WAKEUP_CMD                 = 0x02;
	const uint8_t ECG2_STANDBY_CMD                = 0x04;
	const uint8_t ECG2_START_CONVERSION           = 0x08;
	const uint8_t ECG2_RESET_CMD                  = 0x06;
	const uint8_t ECG2_STOP_CONVERSION            = 0x0A;
	const uint8_t ECG2_STOP_DATA_CONT_MODE        = 0x11;
	const uint8_t ECG2_READ_DATA_CMD 			  = 0x12;
	const uint8_t ECG2_ENABLE_READ_DATA_CONT_MODE = 0x10;
	const uint8_t ECG2_SPI_CMD_WRITE              = 0x40;
	const uint8_t ECG2_SPI_CMD_READ   			  = 0x20;

	// specific ADC constants
#define numOfBytesInSample 19 // number of bytes in ADS1194 sample in data continuous mode
	const double channelGain = 20.00; // amplifier gain
	const double Vref = 2400.00; // reference voltage in millivolts

	// variable for ADC
	unsigned char ECGdataSample[numOfBytesInSample];  // one sample data from ADS1194
	double channelVoltage1; // channel 1 millivolts
	double channelVoltage2; // channel 2 millivolts
	double channelVoltage3; // channel 3 millivolts
	double channelVoltage4; // channel 4 millivolts

	// Function for sending command
	void sendCommand(uint8_t command) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // Select chip
	  HAL_Delay(1);
	  HAL_SPI_Transmit(&hspi1, &command, 1, HAL_MAX_DELAY); // Send command
	  HAL_Delay(2);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // Deselect chip
	  HAL_Delay(2);
	}

	void writeOneRegister(unsigned char regAddress, unsigned char regValue) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // select chip
	  HAL_Delay(1);
	  uint8_t txData[3];
	  txData[0] = ECG2_SPI_CMD_WRITE | regAddress; // send register address
	  txData[1] = 0x00; // write in one register
	  txData[2] = regValue; // send register data
	  HAL_SPI_Transmit(&hspi1, txData, 3, HAL_MAX_DELAY);
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // deselect chip
	  HAL_Delay(5);
	}


	uint8_t readRegisterStatus(uint8_t regAddress) {
		uint8_t returnData = 0;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // select chip
		HAL_Delay(1);
		// Create the buffer to send and receive data
		uint8_t txData[3];
		uint8_t rxData[3];


		// Send the register read command
		txData[0] = ECG2_SPI_CMD_READ | regAddress;
		txData[1] = 0x00;
		HAL_SPI_TransmitReceive(&hspi1, txData, (uint8_t*)&rxData, 3 , HAL_MAX_DELAY);
		returnData = rxData[2];


		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	    HAL_Delay(1);

	    HAL_UART_Transmit(&huart2, &returnData, 1, HAL_MAX_DELAY);


		return returnData;
	}


	// function  to  read channel voltage in millivolts from ADS1194 sample data
	// arguments:
	// sampleArray - one sample data from ADS1194, placeInSample - where is in sampla channel data is
	// refV - reference voltage in millivolts, gain channel gain, offsetVoltage - channnel offset
	double readAnalogChannel( unsigned char *sampleArray, unsigned short placeInSample, double refV, double gain) {
	  int ADCvalue = sampleArray[placeInSample];
	  ADCvalue <<= 8;
	  ADCvalue |= sampleArray[placeInSample + 1];
	  return ( ((double)ADCvalue*(refV/(32768-1))) / gain);
	}

	unsigned short SPI1_Read(unsigned short dummyData) {


	    uint8_t receivedData = 0;


	    // Send the dummy data to generate the clock
	    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&dummyData, &receivedData, 1, HAL_MAX_DELAY);

	    return receivedData;



	}
/*
	unsigned short SPI1_Read(unsigned short dummyData) {
		    uint8_t receivedData;
		    HAL_SPI_Receive(&hspi1, &receivedData, 1, HAL_MAX_DELAY);
		    return receivedData;

		}*/
/*
	void emptySpiReceiveBuffer(void) {
	  while ((hspi1.Instance->SR & SPI_FLAG_RXNE) != 0) {
	    SPI1_Read();
	  }
	}*/


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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  uint16_t i = 0;
  char final_string[20];
  char time_string[20];
  double time_value = 0.0;
  HAL_Delay(300);



  	HAL_GPIO_WritePin(PWD_GPIO_Port, PWD_Pin, GPIO_PIN_SET); //ECG2 Powered up
  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); //CS chip should be high by default
  	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET); // pull RESET bit low for 18 CLK to RESET ECG device

  	// issue RESET pulse
  	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);//0
  	HAL_Delay(1);
  	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);//1

  	HAL_Delay(1000);

  //	emptySpiReceiveBuffer();

   // device is in RDATAC mode, set it to SDATAC mode to edit registers
  	sendCommand(ECG2_STOP_DATA_CONT_MODE);
  	HAL_Delay(1000);




	 // set configuration registers

  	 // setting configuration register 1
  	  writeOneRegister(0x01,0x06); // no clk output and sample rate is 125SPS, ECG data will be out every 8 millisecond
  	  // setting configuration register 2
  	  writeOneRegister(0x02,0x20); //no test signal, default value
  	  // setting configuration register 3
  	  writeOneRegister(0x03,0xCC); // RDL generate internal and enable, reference voltage is 2.4V, RLD signal source is internal
  	  // setting LOFF register
  	  writeOneRegister(0x04,0xF3);  // lead-off is in DC mode and using pull up and down resistors, comparators thresholds are set to 70% and 30%
  	  // channel 1 settings register
  	 /* writeOneRegister(0x05,0x01); // channel is on and gain is 12, input shorted for offset measurements
  	  // channel 2 settings register
  	  writeOneRegister(0x06,0x01); // channel is on and gain is 12, input shorted for offset measurements
  	  // channel 3 settings register
  	  writeOneRegister(0x07,0x01); // channel is on and gain is 12, input shorted for offset measurements
  	  // channel 4 settings register
  	  writeOneRegister(0x08,0x01); // channel is on and gain is 12, input shorted for offset measurements*/
  	  // channel 1 settings register
  	 writeOneRegister(0x05,0x60); // channel is on and gain is 12, normal electrode input
  	  // channel 2 settings register
  	  writeOneRegister(0x06,0x60); // channel is on and gain is 12, normal electrode input
  	  // channel 3 settings register
  	  writeOneRegister(0x07,0x60); // channel is on and gain is 12, normal electrode input
  	  // channel 4 settings register
  	  writeOneRegister(0x08,0x64); // channel is on and gain is 12, temperature sensor
  	  // RDL_SENSP
  	  writeOneRegister(0x0D,0x02); // channels 2 is use for RDL
  	  // RDL_SENSN
  	  writeOneRegister(0x0E,0x02); // channels 2 is use for RDL
  	  // LOFF_SENSP
  	  writeOneRegister(0x0F,0x05); // channel 3P use pull-up resistor for detect LL lead-off, channel 1P use pull-up resistor for detect LA lead-off,
  	  // LOFF_SENSN
  	  writeOneRegister(0x10,0x02); // channel 2N use pull-down resistor for detect RA lead-off
  	  // LOFF_FLIP
  	  writeOneRegister(0x11,0x00); // no flip
  	  // GPIO settings
  	  writeOneRegister(0x14,0x0F); // GPIO are not use, default value
  	  // PACE settings
  	  writeOneRegister(0x15,0x00); // PACE not use, default value
  	  // setting configuration register 4
  	  writeOneRegister(0x17,0x02); // continuous conversion mode, WCT no connect to RLD, LOFF comparators enable
/*
  	  // activate conversion to read and calculate offset
  	  sendCommand(ECG2_START_CONVERSION ); // send START command
  	  HAL_Delay(2);
  	  sendCommand(ECG2_READ_DATA_CMD); // enable read data once
  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // chip select
  	  HAL_Delay(1);

  	while (HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin) == 1) {} // Wait for ADS1194 device to prepare output data.
    HAL_Delay(1);
  	    for (i = 0; i < numOfBytesInSample; i++) {ECGdataSample[i] = SPI1_Read(0);}   // read ADS1194 output data, one sample
  	         // Calculate Voltage Offset
  	         // voltage LA RA
  	       channelVoltageOffset1 = calculateChannel( ECGdataSample, 3, Vref, channelGain,0);
  	         // voltage LL RA - channel 2 is usually used for simple ECG
  	       channelVoltageOffset2 = calculateChannel( ECGdataSample, 5, Vref, channelGain,0);
             // voltage LL LA
  	       channelVoltageOffset3 = calculateChannel( ECGdataSample, 7, Vref, channelGain,0);
  	         // voltage from temperature sensor
  	       channelVoltageOffset4 = calculateChannel( ECGdataSample, 9, Vref, channelGain,0);

HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
HAL_Delay(1);

// stop conversion for offset
  sendCommand(ECG2_STOP_CONVERSION); // send STOP command
  HAL_Delay(1);
  sendCommand(ECG2_STOP_DATA_CONT_MODE ); // SDATAC mode
  // activate conversion
  // channel 1 settings register
  writeOneRegister(0x05,0x60); // channel is on and gain is 12, normal electrode input
  // channel 2 settings register
  writeOneRegister(0x06,0x60); // channel is on and gain is 12, normal electrode input
  // channel 3 settings register
  writeOneRegister(0x07,0x60); // channel is on and gain is 12, normal electrode input
  // channel 4 settings register
  writeOneRegister(0x08,0x64); // channel is on and gain is 12, temperature sensor*/

  sendCommand(ECG2_START_CONVERSION ); // send START command
  HAL_Delay(1);
  sendCommand(ECG2_ENABLE_READ_DATA_CONT_MODE); // enable read data in continuous mode
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // chip select
  HAL_Delay(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

	// readRegisterStatus(0x05);




    while (HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin) == 1) {}
    HAL_Delay(1);
    for (i = 0; i < numOfBytesInSample; i++)  { ECGdataSample[i] = SPI1_Read(0); } // read ADS1194 output data, one sample


            sprintf(final_string, "%.2d", ECGdataSample[i]);
        	 HAL_UART_Transmit(&huart2, (uint8_t*)final_string, strlen(final_string), HAL_MAX_DELAY);
        	 char str[2] = "\r\n";
        	 HAL_UART_Transmit(&huart2, (uint8_t *)str, 2, HAL_MAX_DELAY);

        time_value += 8.0; // increment time value
         // calculate input voltage
         // voltage LA RA

         channelVoltage1 = readAnalogChannel( ECGdataSample, 3, Vref, channelGain);
         // voltage LL RA - channel 2 is usually used for simple ECG
         channelVoltage2 = readAnalogChannel( ECGdataSample, 5, Vref, channelGain);
         sprintf(final_string, "%.2f", channelVoltage2); // convert values to string and send to MikroPlot
         strcat(final_string, ",");
         sprintf(time_string, "%.2f", time_value);
         strcat(final_string, time_string);
         HAL_UART_Transmit(&huart2, (uint8_t *)final_string, strlen(final_string), HAL_MAX_DELAY);
         char str[2] = "\r\n";
         HAL_UART_Transmit(&huart2, (uint8_t *)str, 2, HAL_MAX_DELAY);
          // voltage LL LA
         channelVoltage3 = readAnalogChannel( ECGdataSample, 7, Vref, channelGain);
         // voltage from temperature sensor
         channelVoltage4 = readAnalogChannel( ECGdataSample, 9, Vref, channelGain);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWD_GPIO_Port, PWD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin RST_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin CS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRDY_Pin BOOT1_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PWD_Pin */
  GPIO_InitStruct.Pin = PWD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PACE_Pin */
  GPIO_InitStruct.Pin = PACE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PACE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

