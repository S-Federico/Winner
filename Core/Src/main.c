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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Indirizzo I2C del sensore ISL29125
// Indirizzo I2C del sensore ISL29125
#define ISL29125_ADDR 0x44

// Indirizzi dei registri di interesse (ad es. potresti aver bisogno di altri registri in base alle tue necessità)
#define DEVICE_ID_REG 0x00
#define GREEN_DATA_REG_LOW_BYTE 0X09
#define GREEN_DATA_REG_HIGH_BYTE 0X0A
#define RED_DATA_REG_LOW_BYTE 0X0B
#define RED_DATA_REG_HIGH_BYTE 0X0C
#define BLUE_DATA_REG_LOW_BYTE 0X0D
#define BLUE_DATA_REG_HIGH_BYTE 0X0E
#define CONFIG1_REG 0x01
#define SENSOR1_PIN GPIO_PIN_0
#define SENSOR2_PIN GPIO_PIN_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

typedef enum {
    MODE_FULL,
    MODE_HB
} ReadMode;

char THINGSPEAK_API_KEY[] = "TRZOK7CVA05TTGZR";
uint8_t Temp_byte1_1, Temp_byte2_1, Temp_byte1_2, Temp_byte2_2;
uint8_t Presence1, Presence2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

//DS18B20
uint8_t DS18B20_Read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void DS18B20_Write(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t data);
uint8_t DS18B20_Start(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void delay(uint32_t);
void Set_Input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Set_Output(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
double ds18b20_convert(uint8_t lsb, uint8_t msb);
double computeValue(uint8_t lsb, uint8_t msb);
double DS18B20_ReadTemperature(GPIO_TypeDef* GPIOx, uint16_t SENSOR_PIN);

//ISL29125
void configureISL29125();
uint8_t readISL29125Register(uint8_t reg);
void readISL29125Colors(uint8_t* red, uint8_t* green, uint8_t* blue, ReadMode mode);

//ESP8266
void sendCommandAndPrintResponse(UART_HandleTypeDef *huart, const char *command, UART_HandleTypeDef *response_uart, uint32_t delay_time);
void sendDataToThingSpeak(UART_HandleTypeDef *huart, int data1, int data2, int data3, int data4, int data5, uint32_t delay_time);


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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  configureISL29125();

  uint32_t delay_time=2*1000;

  //Controllo corretto funzionamento
  sendCommandAndPrintResponse(&huart1, "AT\r\n", &huart2,delay_time);

   // Imposta l'ESP8266 in modalità station
  sendCommandAndPrintResponse(&huart1, "AT+CWMODE=1\r\n", &huart2, delay_time);

   // Connettiti alla rete WiFi
   sendCommandAndPrintResponse(&huart1, "AT+CWJAP=\"A\",\"\"\r\n", &huart2, delay_time);
   HAL_Delay(8000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 double temperatura1 = DS18B20_ReadTemperature(GPIOA, SENSOR1_PIN);

	 double temperatura1 = DS18B20_ReadTemperature(GPIOA, SENSOR2_PIN);

	 uint8_t red, green, blue;
	 readISL29125Colors(&red, &green, &blue, MODE_FULL);  // Per la modalità completa

	 sendDataToThingSpeak(&huart1, temperatura1, temperatura2, red, green, blue, delay_time);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Funzione per stampare un messaggio sul UART specificato
void printToUART(UART_HandleTypeDef *uart, const char *message) {
    HAL_UART_Transmit(uart, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}
uint8_t readISL29125Register(uint8_t reg) {
    uint8_t value;
    HAL_I2C_Mem_Read(&hi2c1, ISL29125_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
    return value;
}
void configureISL29125() {
    uint8_t configValue = 0b00000101;
    HAL_I2C_Mem_Write(&hi2c1, ISL29125_ADDR << 1, CONFIG1_REG, I2C_MEMADD_SIZE_8BIT, &configValue, 1, HAL_MAX_DELAY);
}

void readISL29125Colors(uint8_t* red, uint8_t* green, uint8_t* blue, ReadMode mode) {
    uint16_t redValue, greenValue, blueValue;

    if(mode == MODE_FULL) {
        uint8_t redLB = readISL29125Register(RED_DATA_REG_LOW_BYTE);
        uint8_t redHB = readISL29125Register(RED_DATA_REG_HIGH_BYTE);
        redValue = (redHB << 8) | redLB;

        uint8_t greenLB = readISL29125Register(GREEN_DATA_REG_LOW_BYTE);
        uint8_t greenHB = readISL29125Register(GREEN_DATA_REG_HIGH_BYTE);
        greenValue = (greenHB << 8) | greenLB;

        uint8_t blueLB = readISL29125Register(BLUE_DATA_REG_LOW_BYTE);
        uint8_t blueHB = readISL29125Register(BLUE_DATA_REG_HIGH_BYTE);
        blueValue = (blueHB << 8) | blueLB;
    } else if(mode == MODE_HB) {
        redValue = readISL29125Register(RED_DATA_REG_HIGH_BYTE) << 8;
        greenValue = readISL29125Register(GREEN_DATA_REG_HIGH_BYTE) << 8;
        blueValue = readISL29125Register(BLUE_DATA_REG_HIGH_BYTE) << 8;
    }

    *red = redValue * 255 / 65535;
    *green = greenValue * 255 / 65535;
    *blue = blueValue * 255 / 65535;
}


double DS18B20_ReadTemperature(GPIO_TypeDef* GPIOx, uint16_t SENSOR_PIN) {
    // Inizia la comunicazione con il DS18B20
    uint8_t Presence = DS18B20_Start(GPIOx, SENSOR_PIN);
    HAL_Delay(1);  // Piccola pausa per assicurarsi che il DS18B20 sia pronto

    // Manda il comando "Skip ROM", che dice al DS18B20 di saltare il controllo dell'indirizzo (ROM)
    DS18B20_Write(GPIOx, SENSOR_PIN, 0xCC);

    // Manda il comando per iniziare la conversione della temperatura
    DS18B20_Write(GPIOx, SENSOR_PIN, 0x44);

    // Aspetta che la conversione della temperatura sia completata.
    // Il DS18B20 impiega fino a 750ms per effettuare una conversione
    HAL_Delay(800);

    // Ri-inizia la comunicazione con il DS18B20
    Presence = DS18B20_Start(GPIOx, SENSOR_PIN);
    HAL_Delay(1);  // Piccola pausa per assicurarsi che il DS18B20 sia pronto

    // Manda di nuovo il comando "Skip ROM"
    DS18B20_Write(GPIOx, SENSOR_PIN, 0xCC);

    // Manda il comando per leggere lo scratchpad, dove il DS18B20 conserva i dati della temperatura appena convertita
    DS18B20_Write(GPIOx, SENSOR_PIN, 0xBE);

    // Legge i due byte che rappresentano la temperatura.
    uint8_t Temp_byte1 = DS18B20_Read(GPIOx, SENSOR_PIN);
    uint8_t Temp_byte2 = DS18B20_Read(GPIOx, SENSOR_PIN);

    // Converte i due byte letti in un valore di temperatura comprensibile e lo restituisce
    return computeValue(Temp_byte1, Temp_byte2);
}

void sendCommandAndPrintResponse(UART_HandleTypeDef *huart, const char *command, UART_HandleTypeDef *response_uart, uint32_t delay_time) {

    uint8_t esp_buffer[100];
    char str_buffer[100]; // Buffer per la formattazione della risposta

    // Invia comando all'ESP8266
    HAL_UART_Transmit(huart, (uint8_t *)command, strlen(command), delay_time);

    // Inizializza esp_buffer con zeri
    memset(esp_buffer, 0, sizeof(esp_buffer));

    // Aspetta e ricevi la risposta dall'ESP8266
    HAL_StatusTypeDef status = HAL_UART_Receive(huart, esp_buffer, sizeof(esp_buffer) - 1, delay_time);

    // Verifica lo stato della ricezione
    if (status == HAL_OK || (status == HAL_TIMEOUT && strlen(esp_buffer) > 0)) {
    	HAL_Delay (delay_time);
        // Formatta la risposta ricevuta
        sprintf(str_buffer, "Risposta a %s: %s\r\n",command, esp_buffer);
        // Stampa la risposta usando la funzione appena creata
        printToUART(response_uart, str_buffer);
    } else {
        char error_msg[] = "Nessuna risposta dall'ESP8266\r\n";
        printToUART(response_uart, error_msg);
    }
	  HAL_Delay (delay_time);
}
void sendDataToThingSpeak(UART_HandleTypeDef *huart, int data1, int data2, int data3, int data4, int data5, uint32_t delay_time) {
    // Connessione TCP a ThingSpeak
    sendCommandAndPrintResponse(huart, "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n", &huart2, delay_time);

    // Preparazione della richiesta GET
    char get_request[256];
    sprintf(get_request, "GET /update?api_key=%s&field1=%d&field2=%d&field3=%d&field4=%d&field5=%d\r\n",
            THINGSPEAK_API_KEY, data1, data2, data3, data4, data5);

    char length_cmd[50];
    sprintf(length_cmd, "AT+CIPSEND=%d\r\n", strlen(get_request));

    sendCommandAndPrintResponse(huart, length_cmd, &huart2, delay_time);
    sendCommandAndPrintResponse(huart, get_request, &huart2, delay_time);
    // La connessione TCP non viene chiusa in questa funzione, come richiesto
}

double computeValue(uint8_t lsb, uint8_t msb) {
    double result = 0.0;

    // Gestione dei primi 8 bit dell'LSB
    for (int i = 0; i < 8 && i < 11; ++i) {
        if (lsb & (1 << i)) {
            result += pow(2, i - 4);
        }
    }

    // Gestione dei bit restanti dell'MSB
    int remaining_bits = 11 - 8;
    for (int i = 0; i < remaining_bits; ++i) {
        if (msb & (1 << i)) {
            result += pow(2, i + 4);
        }
    }

    return result;
}

uint8_t DS18B20_Read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    uint8_t value = 0;
    Set_Input(GPIOx, GPIO_Pin);

    for (int i = 0; i < 8; i++)
    {
        Set_Output(GPIOx, GPIO_Pin); // set as output

        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0); // pull the data pin LOW
        delay(2); // wait for 2 us

        Set_Input(GPIOx, GPIO_Pin); // set as input
        if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) // if the pin is HIGH
        {
            value |= 1 << i; // read = 1
        }
        delay(60); // wait for 60 us
    }
    return value;
}

void DS18B20_Write(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t data)
{
    Set_Output(GPIOx, GPIO_Pin); // set as output

    for (int i = 0; i < 8; i++)
    {
        if ((data & (1 << i)) != 0) // if the bit is high
        {
            Set_Output(GPIOx, GPIO_Pin); // set as output
            HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0); // pull the pin LOW
            delay(1); // wait for 1 us

            Set_Input(GPIOx, GPIO_Pin); // set as input
            delay(50); // wait for 60 us
        }
        else // if the bit is low
        {
            Set_Output(GPIOx, GPIO_Pin);
            HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0); // pull the pin LOW
            delay(50); // wait for 50 us

            Set_Input(GPIOx, GPIO_Pin);
        }
    }
}

uint8_t DS18B20_Start(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    uint8_t Response = 0;
    Set_Output(GPIOx, GPIO_Pin); // set the pin as output
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0); // pull the pin low
    delay(480); // delay according to datasheet

    Set_Input(GPIOx, GPIO_Pin); // set the pin as input
    delay(80); // delay according to datasheet
    if (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))) Response = 1; // if the pin is low i.e the presence pulse is detected
    else Response = -1;

    delay(400); // 480 us delay totally.
    return Response;
}

void Set_Input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Output(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void delay(uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
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
