
### Capturing ECG Sensor Data via SPI

Code and documentation for interfacing an STM32F407G-DISC1 develeopment board with an Electrocardiogram (ECG) sensor using their  Serial Peripheral Interface (SPI) communication protocols. The content herein demonstrates how to establish communication with the ECG sensor viz. read ECG signal data. This document aims to provide a clear and comprehensive guide to help you integrate an ECG sensor with your projects using the SPI interface from scratch.

### Requirements


Before starting, ensure you have the following:

- STM32CubeIDE installed on your development machine.
- STM32F407G-DISC1 development board.
- ECG Click 2 sensor.
- TTL-USB converter to enable UART communication between STM32F407G board and PC.
- Necessary cables to connect the ECG Click 2 sensor to the STM32F407G-DISC1 board.


### Hardware Setup

- Power STM32F407G-DISC1 board up, by connecting it to your PC via USB plug.
- ECG sensor is powered up by both 3V/5V, so power the sensor board to its corresponding voltages on the STM32 board via jumper wires.
- Connect the SPI communication lines (MISO, MOSI, SCK, and CS) between the STM32F407G-DISC1 board and the ECG Click 2 sensor.
- Connect DRDY, PWD and RST pins to any available GPIO pins on STM32 board.


### Software Setup

- Launch STM32Cube IDE 
- Create a new project for your STM board, make sure you specify the right board when prompted. 
- Select CUBEMX and not empty file which enables you to easily configure the board and gives you access to the HAL libraries.
- Navigate to "Pinout & Configuration" click on SPI1 in order to define its parameters.
- Set CPOL - Low and CPHA - 2 Edge, this clock mode is the preferred mode seen in the datasheet of the ecg2click sensor.
- Aim to have your SPI clock to be around 500khz. This is done by dividing your system clock by the prescaler.
- Now navigate to system core and click on GPIO, choose three free pins for  DRDY, RST, and PWD of the sensor board. 
- Now press ctrl + s and when prompted click yes and you would be on moved onto a main.c file.

  
### Documentation

- [ECG2 Click Sensor Website](https://www.mikroe.com/ecg-2-click)

- [ECG2 Click Sensor Datasheet](https://www.ti.com/lit/ds/symlink/ads1194.pdf)

- [STM32F407G-DISC1 Website](https://www.st.com/en/evaluation-tools/stm32f4discovery.html)

 ### Code Implementation

- Open the main code file

- First include these header files in the appropriate section.
```
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

```
- After the private functions generated by the Cube MX, you start defining your MACROS
```
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);
```
- First you want to define the specific operation commands for the ECG for communication.

```

  const uint8_t ECG2_WAKEUP_CMD                 = 0x02;
	const uint8_t ECG2_STANDBY_CMD                = 0x04;
	const uint8_t ECG2_START_CONVERSION           = 0x08;
	const uint8_t ECG2_RESET_CMD                  = 0x06;
	const uint8_t ECG2_STOP_CONVERSION            = 0x0A;
	const uint8_t ECG2_STOP_DATA_CONT_MODE        = 0x11;
	const uint8_t ECG2_READ_DATA_CMD 			        = 0x12;
	const uint8_t ECG2_ENABLE_READ_DATA_CONT_MODE = 0x10;
	const uint8_t ECG2_SPI_CMD_WRITE              = 0x40;
	const uint8_t ECG2_SPI_CMD_READ   			      = 0x20;
```
- And now you will input the needed functions shown here below please read the commented lines to understand the purpose of each function.
```
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

	// Function for sending commands
	void sendCommand(uint8_t command) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // Select chip
	  HAL_Delay(1);
	  HAL_SPI_Transmit(&hspi1, &command, 1, HAL_MAX_DELAY); // Send command
	  HAL_Delay(2);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // Deselect chip
	  HAL_Delay(2);
	  
	  //Function for writing register values
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

    //Function to read register status for debugging purposes
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
	
	
  //Function to generate SPI Clock 
	unsigned short SPI1_Read(unsigned short dummyData) {
  uint8_t receivedData = 0;

 // Send the dummy data to generate the clock
 HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&dummyData, &receivedData, 1, HAL_MAX_DELAY);
 return receivedData;
	}
```
- Now you are in the  main function "int (main) void after /* Initialize all configured peripherals */
These are the variables you need to set:
```

  uint16_t i = 0;
  char final_string[20];
  char time_string[20];
  double time_value = 0.0;
  HAL_Delay(300);
```
TODO :: And then

























