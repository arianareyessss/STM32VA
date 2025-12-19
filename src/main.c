/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
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
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define TRIG_PIN GPIO_PIN_1
#define TRIG_PORT GPIOB
#define ECHO_PIN GPIO_PIN_0
#define ECHO_PORT GPIOB
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;  // cm
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
#define TCS34725_ADDRESS          (0x29 << 1) /* I2C address */
/* Datasheet is at https://cdn-shop.adafruit.com/datasheets/TCS34725.pdf */
#define TCS34725_COMMAND_BIT      (0x80)      /* Command bit */
#define TCS34725_ENABLE           (0x00)      /* Enable register */
#define TCS34725_ENABLE_AEN       (0x02)      /* RGBC Enable */
#define TCS34725_ENABLE_PON       (0x01)      /* Power on */
#define TCS34725_ATIME            (0x01)      /* Integration time */
#define TCS34725_CONTROL          (0x0F)      /* Set the gain level */
#define TCS34725_ID               (0x12)
/* 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_CDATAL           (0x14)      /* Clear channel data */
#define TCS34725_CDATAH           (0x15)
#define TCS34725_RDATAL           (0x16)      /* Red channel data */
#define TCS34725_RDATAH           (0x17)
#define TCS34725_GDATAL           (0x18)      /* Green channel data */
#define TCS34725_GDATAH           (0x19)
#define TCS34725_BDATAL           (0x1A)      /* Blue channel data */
#define TCS34725_BDATAH           (0x1B)
/* set the correct delay time in void getRawData() for TCS34725_INTEGRATIONTIME */
#define TCS34725_INTEGRATIONTIME_50MS   0xEB  /* 50ms  - 20 cycles */
#define TCS34725_GAIN_4X                0x01  /* 4x gain  */

uint8_t _tcs34725Initialised = 0;
int red, green, blue;

void write8 (uint8_t reg, uint32_t value);
uint8_t read8(uint8_t reg);
uint16_t read16(uint8_t reg);
void enable(void);
void disable(void);
void setIntegrationTime(uint8_t it);
void setGain(uint8_t gain);
void tcs3272_init( void );
void getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
void getRGB(int *R, int *G, int *B);

/* Writes a register and an 8 bit value over I2C */
void write8 (uint8_t reg, uint32_t value)
{
    uint8_t txBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    txBuffer[1] = (value & 0xFF);
    HAL_I2C_Master_Transmit(&hi2c3, TCS34725_ADDRESS, txBuffer, 2, 100);
}

/* Reads an 8 bit value over I2C */
uint8_t read8(uint8_t reg)
{
    uint8_t buffer[1];
    buffer[0] = (TCS34725_COMMAND_BIT | reg);

    HAL_I2C_Master_Transmit(&hi2c3, TCS34725_ADDRESS, buffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c3, TCS34725_ADDRESS, buffer, 1, 100);
    return buffer[0];
}

/* Reads a 16 bit values over I2C */
uint16_t read16(uint8_t reg)
{
  uint16_t ret;
    uint8_t txBuffer[1],rxBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    HAL_I2C_Master_Transmit(&hi2c3, TCS34725_ADDRESS, txBuffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c3, TCS34725_ADDRESS, rxBuffer, 2, 100);
    ret = rxBuffer[1];
    ret <<= 8;
    ret |= rxBuffer[0] & 0xFF;
  return ret;
}

void enable(void)
{
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  HAL_Delay(3);
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
  HAL_Delay(50);
}

void disable(void)
{
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = read8(TCS34725_ENABLE);
  write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

void setIntegrationTime(uint8_t itime)
{
  if (_tcs34725Initialised == 0) tcs3272_init();
  write8(TCS34725_ATIME, itime);
}

void setGain(uint8_t gain)
{
  if (_tcs34725Initialised == 0) tcs3272_init();
  write8(TCS34725_CONTROL, gain);
}

void tcs3272_init(void)
{
  /* Make sure we're actually connected */
  uint8_t readValue = read8(TCS34725_ID);
  if ((readValue != 0x44) && (readValue != 0x4D))
  {
    return;
  }
  _tcs34725Initialised = 1;
  /* Set default integration time and gain */
  setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
  setGain(TCS34725_GAIN_4X);
  /* Note: by default, the device is in power down mode on bootup */
  enable();
}

/* Get raw data */
void getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (_tcs34725Initialised == 0) tcs3272_init();

  *c = read16(TCS34725_CDATAL);
  *r = read16(TCS34725_RDATAL);
  *g = read16(TCS34725_GDATAL);
  *b = read16(TCS34725_BDATAL);
  /* Delay time is from page no 16/26 from the datasheet  (256 − ATIME)* 2.4ms */
  HAL_Delay(50); // Set delay for (256 − 0xEB)* 2.4ms = 50ms
}

/* Get Red, Green and Blue color from Raw Data */
void getRGB(int *R, int *G, int *B)
{
    uint16_t rawRed, rawGreen, rawBlue, rawClear;
    getRawData(&rawRed, &rawGreen, &rawBlue, &rawClear);
    if(rawClear == 0)
    {
      *R = 0;
      *G = 0;
      *B = 0;
    }
    else
    {
      *R = (int)rawRed * 255 / rawClear;
      *G = (int)rawGreen * 255 / rawClear;
      *B = (int)rawBlue * 255 / rawClear;
    }
}

// -------- LEDs en PA3/PA4/PA5 --------
#define LED_R_PIN GPIO_PIN_3
#define LED_G_PIN GPIO_PIN_4
#define LED_B_PIN GPIO_PIN_5

static void leds_off(void){
  HAL_GPIO_WritePin(GPIOA, LED_R_PIN|LED_G_PIN|LED_B_PIN, GPIO_PIN_RESET);
}

static void leds_show_red(void){
  leds_off(); HAL_GPIO_WritePin(GPIOA, LED_R_PIN, GPIO_PIN_SET);
}
static void leds_show_green(void){
  leds_off(); HAL_GPIO_WritePin(GPIOA, LED_G_PIN, GPIO_PIN_SET);
}
static void leds_show_blue(void){
  leds_off(); HAL_GPIO_WritePin(GPIOA, LED_B_PIN, GPIO_PIN_SET);
}

// -------- Lectura 1 sola vez: devuelve R,G,B (0..255) y C raw --------
static void getRGBC_255(uint8_t *R, uint8_t *G, uint8_t *B, uint16_t *C)
{
  uint16_t r,g,b,c;
  getRawData(&r,&g,&b,&c);
  *C = c;

  if (c == 0) { *R=0; *G=0; *B=0; return; }

  *R = (uint8_t)((r * 255U) / c);
  *G = (uint8_t)((g * 255U) / c);
  *B = (uint8_t)((b * 255U) / c);
}

// -------- SOLO 3 COLORES por RANGOS (con tolerancia) --------
typedef enum { COL_NONE=0, COL_RED, COL_GREEN, COL_BLUE } Color_t;

// 1) "Hay caja" si C supera este umbral.
// AJÚSTALO mirando tu print cuando NO hay caja vs con caja.
#define C_PRESENT_MIN  35

// 2) Rangos/tolerancias (error permitido)
// (más grande = más permisivo)
#define TOL_RED   10
#define TOL_GREEN 10
#define TOL_BLUE  10

static int in_range(int v, int center, int tol){
  return (v >= (center - tol)) && (v <= (center + tol));
}

static Color_t classify_3colors(uint8_t R, uint8_t G, uint8_t B, uint16_t C)
{
	if (C < C_PRESENT_MIN) return COL_NONE;

	  // ROJO (centro aprox: 197,145,112)
	  if (in_range(R, 185, TOL_RED) &&
	      in_range(G, 47, TOL_RED) &&
	      in_range(B, 39, TOL_RED))
	    return COL_RED;

	  // VERDE (centro aprox: 210,168,131)
	  if (in_range(R, 98, TOL_GREEN) &&
	      in_range(G, 101, TOL_GREEN) &&
	      in_range(B, 62, TOL_GREEN))
	    return COL_GREEN;

	  // AZUL (centro aprox: 55,45,45)  (con C>=120 no se va a confundir con "sin caja")
	  if (in_range(R, 104, TOL_BLUE) &&
	      in_range(G, 85, TOL_BLUE) &&
	      in_range(B, 85, TOL_BLUE))
	    return COL_BLUE;

	  return COL_NONE;
}

/* USER CODE END 0 */

static void UART_Send(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 100);
}


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
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim15);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
    /* USER CODE END 2 */

  /* Infinite loop */
  uint8_t R8,G8,B8;
  uint16_t C;
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* SENSOR DE COLOR */
	getRGBC_255(&R8, &G8, &B8, &C);

	// Debug (opcional)
	char msg[64];
	snprintf(msg, sizeof(msg), "C:%u R:%u G:%u B:%u\r\n", C, R8, G8, B8);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

	if (classify_3colors(R8, G8, B8, C) == COL_NONE)
	{
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);   // LED ON = OK
	}
	else
	{
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // LED OFF = FAIL
	}

	Color_t col = classify_3colors(R8, G8, B8, C);

	switch(col){
	case COL_RED:   leds_show_red();   break;
	case COL_GREEN: leds_show_green(); break;
	case COL_BLUE:  leds_show_blue();  break;
	default:        leds_off();        break;
	}

	HAL_Delay(50);

    /* HC-04 */
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	__HAL_TIM_SET_COUNTER(&htim15, 0);
	while (__HAL_TIM_GET_COUNTER (&htim15) < 10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	// wait for the echo pin to go high
	while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
	Value1 = __HAL_TIM_GET_COUNTER (&htim15);

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	// wait for the echo pin to go low
	while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
	Value2 = __HAL_TIM_GET_COUNTER (&htim15);

	Distance = (Value2-Value1)* 0.034/2;
	HAL_Delay(50);

	char m1sg[64];
	snprintf(m1sg, sizeof(m1sg), "Distancia: %u cm\r\n", Distance);
	UART_Send(m1sg);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00F12981;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 79;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Si el compilador aún necesita la función (aunque no se llame), 
// puedes dejarla vacía o eliminarla completamente
// void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
// {
//     // Función vacía ya que no usamos PWM
//     UNUSED(htim);
// }

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
#ifdef USE_FULL_ASSERT
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