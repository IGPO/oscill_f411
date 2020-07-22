/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_DATA_SIZE 		2048	// 137
#define BIT24							0x800000
#define SPI1_nCS_activate  HAL_GPIO_WritePin(GPIOB, NSS_Pin, GPIO_PIN_RESET);
#define SPI1_nCS_deactivate  HAL_GPIO_WritePin(GPIOB, NSS_Pin, GPIO_PIN_SET);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct CDS_AHRS_Dual_KF_data_struct {
	//данные от датчиков
	float Start;//если не HID
	float accel[3];
	float gyro[3];
	float mag[3];
	//кватернион
	float q[4];
	//углы поворота
	float pitch;
	float roll;
	float yaw;
} CDS_AHRS_Dual_KF_data_t, *CDS_AHRS_Dual_KF_data_p;

void CDS_AHRS_Quaternion_Dual_KF(CDS_AHRS_Dual_KF_data_p data,float dt);

struct CDS_AHRS_Dual_KF_data_struct Data_to_send, *Data_to_send_p;
uint8_t dataToSend_TRND[68], *p1;
uint8_t dataToSend_SPI[SPI_DATA_SIZE * 3];

uint32_t convert(uint32_t);
uint16_t *adc1_p, *adc2_p, *spi_p, cou = 0, m_cou, test_matrix[1024], index;
uint8_t *byte_p;
void *p;
uint8_t SPI_read = 1, num = 1;
uint32_t data_tmp;
int32_t spi_data_to_send_1, spi_data_to_send_2, spi_data_to_send_3, spi_data_to_send_1_ni, spi_data_to_send_2_ni, spi_data_to_send_3_ni;
float bb, spi_data_to_send_1_f, spi_data_to_send_2_f, spi_data_to_send_3_f;
uint32_t shift = 7;
uint32_t b, delay = 10; 
uint16_t spi_m_data;
extern int USB_send_massage;

						float convert_f(uint32_t a) {
								float fa = a;
					if (a & BIT24) {
						fa = -1 * (float)((~a) & 0x7fffff);
					__NOP();}
					else fa = a;
					__NOP();
					return (fa);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	
	LL_SPI_Enable(SPI1); // enable SPI3
	p = &Data_to_send;
	p1 = p;
//	p = &ADC1_value;
	adc1_p	= p;
//	p = &ADC2_value;
	adc2_p	= p;
	p = &dataToSend_SPI;
	spi_p	= p;
			p = &dataToSend_SPI[cou];
			byte_p = p;
			spi_p	= p;	
	__DMA2_CLK_ENABLE();
	__USART1_CLK_ENABLE();
	Data_to_send.Start = NAN;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
				for(cou = 0; cou < (SPI_DATA_SIZE * 3); cou+=3){	
				SPI1_nCS_deactivate;			
				SPI1_nCS_activate;
				SPI1_nCS_activate;
		/*			
			SPI1->CR1 |= SPI_CR1_SPE;					
		  while (!(SPI1->SR & SPI_SR_RXNE));
			SPI1->CR1 &= ~SPI_CR1_SPE;					
			dataToSend_SPI[cou + 2] = SPI1->DR;

			SPI1->CR1 |= SPI_CR1_SPE;					
		  while (!(SPI1->SR & SPI_SR_RXNE));
			SPI1->CR1 &= ~SPI_CR1_SPE;					
			dataToSend_SPI[cou + 0] = SPI1->DR;

			SPI1->CR1 |= SPI_CR1_SPE;					
		  while (!(SPI1->SR & SPI_SR_RXNE));
			SPI1->CR1 &= ~SPI_CR1_SPE;					
			dataToSend_SPI[cou + 1] = SPI1->DR;					
					*/
					
					
			SPI1->CR1 |= SPI_CR1_SPE;		
		  while (!(SPI1->SR & SPI_SR_RXNE));
			SPI1->CR1 &= ~SPI_CR1_SPE;					
			*(spi_p + cou) = SPI1->DR;			

					
			SPI1->CR1 |= SPI_CR1_SPE;					
		  while (!(SPI1->SR & SPI_SR_RXNE));
			SPI1->CR1 &= ~SPI_CR1_SPE;					
			dataToSend_SPI[cou + 2] = (SPI1->DR) >> 8;
			SPI1->CR1 &= ~SPI_CR1_SPE;	
								SPI1->CR1 &= ~SPI_CR1_SPE;	
								SPI1->CR1 &= ~SPI_CR1_SPE;	
								SPI1->CR1 &= ~SPI_CR1_SPE;	
								SPI1->CR1 &= ~SPI_CR1_SPE;	
								SPI1->CR1 &= ~SPI_CR1_SPE;	
								SPI1->CR1 &= ~SPI_CR1_SPE;									
					
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
						// By Systick interrupt Start USB transmission
					index=0;
				for(cou = 0; cou < (SPI_DATA_SIZE * 3); cou+=3){	

																				//index++;
																				//if(index >= 1023) index = 1023;
																				test_matrix[index] = cou;		
																			spi_data_to_send_1_ni = (uint32_t)(dataToSend_SPI[cou + 2] << 16) + (uint32_t)(dataToSend_SPI[cou + 1] << 8) + (uint32_t)(dataToSend_SPI[cou + 0]);// 
																			spi_data_to_send_1_f = convert_f(spi_data_to_send_1_ni);																			
																			
																		while(USB_send_massage == 0){
																																__NOP();
																			
																																}
																				USB_send_massage = 0;
																			 Data_to_send.accel[0]	= index++;
																																
																			 Data_to_send.gyro[0] = 1000;
																			 Data_to_send.gyro[1] = 2000;
																			 Data_to_send.gyro[2] = 3000;

																			Data_to_send.mag[2] = 100* index++;																																																														
																																
																					for(int i = 0; i < 68; i++){
																					dataToSend_TRND[i] = *(p1+i);
																					}
																					HAL_UART_Transmit_DMA(&huart1, dataToSend_TRND, sizeof(dataToSend_TRND));

																															}
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI1 GPIO Configuration  
  PB3   ------> SPI1_SCK
  PB4   ------> SPI1_MISO 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_SIMPLEX_RX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RX_PULL_GPIO_Port, RX_PULL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RX_PULL_Pin */
  GPIO_InitStruct.Pin = RX_PULL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RX_PULL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

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
