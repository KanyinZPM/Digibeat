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
//#include "ecg2_hal.h"
//#include "ecg2_hw.h"
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
typedef void ( *write_spi_t )( uint16_t data_ );
typedef uint16_t ( *read_spi_t )( uint16_t dummy );

static write_spi_t write_spi_ptr;
static read_spi_t  read_spi_ptr;

// Function for sending command
/*
unsigned short SPI1_Read(unsigned short dummyData) {
	    uint8_t receivedData;
	    HAL_SPI_Receive(&hspi1, &receivedData, 1, HAL_MAX_DELAY);
	    return receivedData;

	}*/

unsigned short SPI1_Read(unsigned short dummyData) {
    uint8_t receivedData = 0;

    // Send the dummy data to generate the clock
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&dummyData, &receivedData, 1, HAL_MAX_DELAY);

    return receivedData;
}

void SPI1_Write(uint16_t data) {

    HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, 1, HAL_MAX_DELAY);
}


int8_t ecg2_hal_read_bytes( uint8_t reg, uint8_t* buffer_in, uint8_t num_bytes)
{
    if (reg > 0b00011111)
        return -1;
    if (num_bytes > 0b00011111)
        return -1;
    else
    {
        uint8_t opcode[2];
        uint8_t *temp_bfr_ptr = buffer_in;
        opcode[0] = 0b00100000;
        opcode[0] |= reg;
        opcode[1] = 0;
        opcode[1] |= num_bytes;

        HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        write_spi_ptr( opcode[0] );
        write_spi_ptr( opcode[1] );
        HAL_Delay(5);

        while(num_bytes--)
            *(temp_bfr_ptr++) = read_spi_ptr(0);

        HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // Unselect chip
    }
}

int8_t ecg2_hal_write_bytes ( uint8_t reg, uint8_t* buffer_out, uint8_t num_bytes)
{
    if (reg > 0b00011111)
        return -1;
    if (num_bytes > 0b00011111)
        return -1;
    else
    {
        uint8_t opcode[2];
        uint8_t *temp_bfr_ptr = buffer_out;
        opcode[0] = 0b01000000;
        opcode[0] |= reg;
        opcode[1] = 0;
        opcode[1] |= num_bytes;

        HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // Select chip
        HAL_Delay(1);
        write_spi_ptr(opcode[0]);
        write_spi_ptr(opcode[1]);

        while(num_bytes--)
            write_spi_ptr(*(temp_bfr_ptr++));
        HAL_Delay(10);
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // Unselect chip
    }
}


	// specific ADC constants
#define num_bytes_sample 19 // number of bytes in ADS1194 sample in data continuous mode
	const double channel_gain = 20.00; // amplifier gain
	const double vref = 2400.00; // reference voltage in millivolts


	// variable for ADC
	unsigned char ecg_data_sample[num_bytes_sample];  // one sample data from ADS1194
	double channel1_voltage; // channel 1 millivolts
	double channel2_voltage; // channel 2 millivolts
	double channel3_voltage; // channel 3 millivolts
	double channel4_voltage; // channel 4 millivolts
	double channel1_voltage_offset; // channel 1 offset millivolts
	double channel2_voltage_offset; // channel 2 offset millivolts
	double channel3_voltage_offset; // channel 3 offset millivolts
	double channel4_voltage_offset; // channel 4 offset millivolts


	void ecg2_hal_init()
	{

	    write_spi_ptr = SPI1_Write;
	    read_spi_ptr  = SPI1_Read;

	}

	void ecg2_hal_send_command (uint8_t command)
	{
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // Select chip
	    HAL_Delay(1);
	    HAL_SPI_Transmit(&hspi1, &command, 1, HAL_MAX_DELAY); // Send command
	    HAL_Delay(2);
	    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // Unselect chip
	    HAL_Delay(2);
	}
	// function  to  read channel voltage in millivolts from ADS1194 sample data
	// arguments:
	// sampleArray - one sample data from ADS1194, placeInSample - where is in sampla channel data is
	// refV - reference voltage in millivolts, gain channel gain, offsetVoltage - channnel offset
	double calculate_ecg_channel( unsigned char *buffer, unsigned short index, double refV, double gain, double offset_voltage )
	{
	  int adc_value = 0;
	  adc_value = 0;
	  adc_value = buffer[index];
	  adc_value <<= 8;
	  adc_value |= buffer[index + 1];
	  return ( ((double)adc_value * (refV / (32768 - 1))) / gain ) - offset_voltage;
	}


	void setup_ecg2()
	{
	  uint8_t tempctr;
	  ecg2_oscillator_clock_enable(true);
	  ecg2_set_output_data_rate(SPS_125);
	  //no test signal, default value
	  ecg2_set_test_source(TEST_SOURCE_EXTERNAL);
	  // RDL internal and enabled, reference voltage is 2.4V, RLD signal source is internal
	  ecg2_power_down_reference_buffer_enable(true);
	  ecg2_set_reference_voltage(vref);
	  ecg2_rld_measurement_enable(true);
	  ecg2_rldref_source_select(RLDEF_SIGNAL_INTERNAL);
	  ecg2_rld_buffer_enable(true);
	  // lead-off is in DC mode and using pull up and down resistors, comparator thresholds are set to 70% and 30%
	  ecg2_lead_off_comparator_threshold_set(POSITIVE_70);
	  ecg2_vlead_off_enable(false);
	  ecg2_ilead_off_magnitude_set(NA_4);
	  ecg2_flead_off_frequency_set(3);

	  // channel is on and gain is 12, input shorted for offset measurements
	  ecg2_configure_channel(1, false, 0, 1);

	  // channel is on and gain is 12, input shorted for offset measurements
	  ecg2_configure_channel(2, false, 0, 1);

	  // channel is on and gain is 12, input shorted for offset measurements
	  ecg2_configure_channel(3, false, 0, 1);
	  // channel is on and gain is 12, input shorted for offset measurements
	  ecg2_configure_channel(4, false, 0, 1);
	  // channel 2 is used for RDL
	  ecg2_right_leg_positive_drive_set(2);
	  // channel 2 is used for RDL
	  ecg2_right_leg_negative_drive_set(2);
	  // channel 3P uses pull-up resistor to detect LL lead-off, channel 1P uses pull-up resistor to detect LA lead-off,
	  ecg2_lead_off_positive_channel_select(5);
	  // channel 2N uses pull-down resistor to detect RA lead-off
	  ecg2_lead_off_negative_channel_select(2);
	  // no flip
	  ecg2_lead_off_current_direction_select(0);
	  // continuous conversion mode, WCT not connected to RLD, LOFF comparators enabled
	  ecg2_lead_off_comparator_enable(true);
	  // activate conversion to read and calculate offset
	  ecg2_hal_send_command(START_COMMAND); // send START command
	  HAL_Delay(2);
	  ecg2_hal_send_command(RDATA_COMMAND); // enable read data once
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // chip select
	  HAL_Delay(1);

	  while (HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin) == 1) {} // Wait for ADS1194 device to prepare output data.
	 HAL_Delay(1);  // Delay_us(5);
	  for (tempctr = 0; tempctr < num_bytes_sample; tempctr++) { ecg_data_sample[tempctr] = SPI1_Read(0); } // read ADS1194 output data, one sample
	  // calculate offset
	  // offset voltage LA RA
	  channel1_voltage_offset = calculate_ecg_channel( ecg_data_sample, 3, vref, channel_gain, 0 );
	  // offset voltage LL RA
	  channel2_voltage_offset = calculate_ecg_channel( ecg_data_sample, 5, vref, channel_gain, 0 );
	  // offset voltage LL LA
	  channel3_voltage_offset = calculate_ecg_channel( ecg_data_sample, 7, vref, channel_gain, 0 );
	  // offset voltage
	  channel4_voltage_offset = calculate_ecg_channel( ecg_data_sample, 9, vref, channel_gain, 0 );
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); //Deselect Chip
	  HAL_Delay(1); //Delay_us(10);

	  // stop conversion
	  ecg2_hal_send_command(STOP_COMMAND); // send STOP command
	  HAL_Delay(2); //Delay_us(100);
	  ecg2_hal_send_command(SDATAC_COMMAND); // SDATAC mode
	  // activate conversion

	  // channel 1 settings register
	  // channel is on and gain is 12, normal electrode input
	  ecg2_configure_channel(1, false, 6, 0);
	  // channel 2 settings register
	  // channel is on and gain is 12, normal electrode input
	  ecg2_configure_channel(2, false, 6, 0);
	  // channel 3 settings register
	  // channel is on and gain is 12, normal electrode input
	  ecg2_configure_channel(3, false, 6, 0);
	  // channel 4 settings register
	  // channel is on and gain is 12, temperature sensor
	  ecg2_configure_channel(4, false, 6, 4);
	  ecg2_hal_send_command(START_COMMAND); // send START command
	  HAL_Delay(2); //Delay_us(100);
	  ecg2_hal_send_command(RDATAC_COMMAND); // enable read data in continuous mode

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // chip select
	  HAL_Delay(1); //Delay_us(1);
	}

	unsigned char readOneRegister( unsigned char regAddress ) {
		 /* unsigned char returnData = 0;
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // select chip
		  HAL_Delay(1);
		  SPI1_Write(RREG_COMMAND|regAddress); // send register address
		  SPI1_Write(0x00); // read one register
		  HAL_Delay(1);
		  returnData = SPI1_Read(0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);;
		  HAL_Delay(10);
		  return returnData;*/
		 unsigned char returnData = 0;
		    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // select chip
		    HAL_Delay(1);
		     // Create the buffer to send and receive data
		    uint8_t txData[2];
		    uint8_t rxData;
            // Send the register read command
		    txData[0] = RREG_COMMAND | regAddress;
		    txData[1] = 0x00;
		    HAL_SPI_TransmitReceive(&hspi1, txData, &rxData, 2, HAL_MAX_DELAY);
            returnData = rxData;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
		    HAL_Delay(10);

		    return returnData;
	}

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
	char uart_buf[50];
	int uart_buf_len;
	char spi_buf[20];

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
  	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); //CS chip should be high by default
  	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET); // pull RESET bit low for 18 CLK to RESET ECG device

  	ecg2_hal_init();

  	// issue RESET pulse
  	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);//0
  	HAL_Delay(1);
  	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);//1

  	HAL_Delay(1000);


  	// device is in RDATAC mode, set it to SDATAC mode to edit registers
  	  ecg2_hal_send_command(SDATAC_COMMAND);
  	  HAL_Delay(1000);
  	  setup_ecg2();

  	 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
  	 HAL_SPI_Transmit(&hspi1, (uint8_t*))


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    while (HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin) == 1) {}
    HAL_Delay(1);

    for (i = 0; i < num_bytes_sample; i++)  {ecg_data_sample[i] = SPI1_Read(0);}   // read ADS1194 output data, one sample

         time_value += 8.0; // increment time value
         // calculate input voltage
         // voltage LA RA
         channel1_voltage = calculate_ecg_channel( ecg_data_sample, 3, vref, channel_gain, channel1_voltage_offset );         // voltage LL RA - channel 2 is usually used for simple ECG
         channel2_voltage = calculate_ecg_channel( ecg_data_sample, 5, vref, channel_gain, channel2_voltage_offset );
         sprintf(final_string, "%.2f", channel2_voltage); // convert values to string and send to MikroPlot
         strcat(final_string, ",");
         sprintf(time_string, "%.2f", time_value);
         strcat(final_string, time_string);
         HAL_UART_Transmit(&huart2, (uint8_t *)final_string, strlen(final_string), HAL_MAX_DELAY);
         char str[2] = "\r\n";
         HAL_UART_Transmit(&huart2, (uint8_t *)str, 2, HAL_MAX_DELAY);
          // voltage LL LA
         channel3_voltage = calculate_ecg_channel( ecg_data_sample, 7, vref, channel_gain, channel3_voltage_offset );
         // voltage from temperature sensor
         channel4_voltage = calculate_ecg_channel( ecg_data_sample, 9, vref, channel_gain, channel4_voltage_offset );
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
