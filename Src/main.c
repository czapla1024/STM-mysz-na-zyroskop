/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
#define PWR_BASE_ADDRESS     0x40007000
#define PWR_CR               PWR_BASE_ADDRESS + 0x00  // PWR power control register

#define RCC_BASE_ADDRESS     0x40023800
#define RCC_CR               RCC_BASE_ADDRESS + 0x00  // RCC clock control register
#define RCC_PLLCFGR          RCC_BASE_ADDRESS + 0x04  // RCC PLL configuration register
#define RCC_CFGR             RCC_BASE_ADDRESS + 0x08  // RCC clock configuration register
#define RCC_AHB1ENR          RCC_BASE_ADDRESS + 0x30  // RCC AHB1 peripheral clock enable register (pg 116)
#define RCC_APB1ENR          RCC_BASE_ADDRESS + 0x40  // RCC APB1 peripheral clock enable register (pg 117)
#define RCC_APB2ENR          RCC_BASE_ADDRESS + 0x44  // RCC APB2 peripheral clock enable register (pg 120)

#define GPIOA_BASE_ADDRESS   0x40020000
#define GPIOA_MODER          GPIOA_BASE_ADDRESS + 0x00 // GPIO port mode register
#define GPIOA_OSPEEDR        GPIOA_BASE_ADDRESS + 0x08 // GPIO port output speed register
#define GPIOA_PUPR           GPIOA_BASE_ADDRESS + 0x0C // GPIO port pull-up/pull-down register
#define GPIOA_ODR            GPIOA_BASE_ADDRESS + 0x14 // GPIO port output data register
#define GPIOA_AFRL           GPIOA_BASE_ADDRESS + 0x20 // GPIO alternate function low register
#define GPIOA_AFRH           GPIOA_BASE_ADDRESS + 0x24 // GPIO alternate function high register

#define GPIOB_BASE_ADDRESS   0x40020400
#define GPIOB_MODER          GPIOB_BASE_ADDRESS + 0x00 // GPIO port mode register
#define GPIOB_OSPEEDR        GPIOB_BASE_ADDRESS + 0x08 // GPIO port output speed register
#define GPIOB_PUPR           GPIOB_BASE_ADDRESS + 0x0C // GPIO port pull-up/pull-down register
#define GPIOB_ODR            GPIOB_BASE_ADDRESS + 0x14 // GPIO port output data register
#define GPIOB_AFRL           GPIOB_BASE_ADDRESS + 0x20 // GPIO alternate function low register
#define GPIOB_AFRH           GPIOB_BASE_ADDRESS + 0x24 // GPIO alternate function high register

#define GPIOE_BASE_ADDRESS   0x40021000
#define GPIOE_MODER          GPIOE_BASE_ADDRESS + 0x00 // GPIO port mode register
#define GPIOE_OSPEEDR        GPIOE_BASE_ADDRESS + 0x08 // GPIO port output speed register
#define GPIOE_PUPR           GPIOE_BASE_ADDRESS + 0x0C // GPIO port pull-up/pull-down register
#define GPIOE_ODR            GPIOE_BASE_ADDRESS + 0x14 // GPIO port output data register
#define GPIOE_BSRR           GPIOE_BASE_ADDRESS + 0x18 // GPIO port bit set/reset register
#define GPIOE_AFRL           GPIOE_BASE_ADDRESS + 0x20 // GPIO alternate function low register
#define GPIOE_AFRH           GPIOE_BASE_ADDRESS + 0x24 // GPIO alternate function high register

#define SPI1_BASE_ADDRESS    0x40013000
#define SPI1_CR1             SPI1_BASE_ADDRESS + 0x00 // SPI control register 1
#define SPI1_SR              SPI1_BASE_ADDRESS + 0x08 // SPI status register
#define SPI1_DR              SPI1_BASE_ADDRESS + 0x0C // SPI data register
#define SPI1_BSRR            SPI1_BASE_ADDRESS + 0x18 // GPIO port bit set/reset register
#define ACCESS(address)      *((volatile unsigned int*)(address))

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
short xturn, yturn, zturn;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t bufor[4]={0};
void MouseSend(uint8_t* buff, uint8_t click, uint8_t osx ,uint8_t osy) {
	buff[0]=0x02;
	buff[1]=click;
	buff[2]=osx;
	buff[3]=osy;
}
void WaitForSPI1RXReady()
{
	while((ACCESS(SPI1_SR) & 1) == 0 || (ACCESS(SPI1_SR) & (1 << 7)) == 1) { }
}

void WaitForSPI1TXReady()
{
	while((ACCESS(SPI1_SR) & (1 << 1)) == 0 || (ACCESS(SPI1_SR) & (1 << 7)) == 1) { }
}
unsigned char ReadFromGyro(unsigned char gyroRegister)
{
	ACCESS(GPIOE_BSRR) |= (1 << 19);
	WaitForSPI1TXReady();
	ACCESS(SPI1_DR) = (gyroRegister | 0x80);
	WaitForSPI1RXReady();
	ACCESS(SPI1_DR);  // I believe we need this simply because a read must follow a write
	WaitForSPI1TXReady();
	ACCESS(SPI1_DR) = 0xFF;
	WaitForSPI1RXReady();
	volatile unsigned char readValue = (unsigned char)ACCESS(SPI1_DR);
	ACCESS(GPIOE_BSRR) |= (1 << 3);

	return readValue;
}
void WriteToGyro(unsigned char gyroRegister, unsigned char value)
{
	ACCESS(GPIOE_BSRR) |= (1 << 19);
	WaitForSPI1TXReady();
	ACCESS(SPI1_DR) = gyroRegister;
	WaitForSPI1RXReady();
	ACCESS(SPI1_DR);
	WaitForSPI1TXReady();
	ACCESS(SPI1_DR) = value;
	WaitForSPI1RXReady();
	ACCESS(SPI1_DR);
	ACCESS(GPIOE_BSRR) |= (1 << 3);
}
short GetAxisValue(unsigned char lowRegister, unsigned char highRegister)
{
	float scaler = 8.75 * 0.0001;
	short temp = (ReadFromGyro(lowRegister) | (ReadFromGyro(highRegister) << 8));
	return (short)((float)temp * scaler);
}

void GetGyroValues(short* x, short* y, short* z)
{
	*x = GetAxisValue(0x28, 0x29);
	*y = GetAxisValue(0x2A, 0x2B);
	*z = GetAxisValue(0x2C, 0x2D);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ACCESS(RCC_AHB1ENR) |=1;
  ACCESS(RCC_AHB1ENR) |=(1 << 4);
  ACCESS(GPIOA_MODER) |= ((1 << 11) | (1 << 13) | (1 << 15));
  ACCESS(GPIOE_MODER) |= (1 << 6);
  ACCESS(GPIOA_AFRL) |= ((5 << 20) | (5 << 24) | (5 << 28));
  ACCESS(GPIOA_OSPEEDR) |= ((2 << 10) | (2 << 12) | (2 << 14));
  ACCESS(GPIOE_OSPEEDR) |= (2 << 6);
  ACCESS(RCC_APB2ENR) |= (1 << 12);
  ACCESS(SPI1_CR1) |= (1 | (1 << 1) | (1 << 2) | (2 << 3) | (1 << 8) | (1 << 9));
  ACCESS(SPI1_CR1) |= (1 << 6);
  ACCESS(GPIOE_BSRR) |= (1 << 3);
  WriteToGyro(0x20, 0x0F);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  GetGyroValues(&xturn, &yturn, &zturn);
	  if(HAL_GPIO_ReadPin(but_GPIO_Port, but_Pin))
		  MouseSend(bufor, 0x01, xturn, yturn);
	  else
		  MouseSend(bufor, 0x00, xturn, yturn);
	  USBD_HID_SendReport (&hUsbDeviceFS, bufor, 4);
	  for(volatile int i = 0; i < 100000; ++i);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : but_Pin */
  GPIO_InitStruct.Pin = but_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(but_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
	  if(HAL_GPIO_ReadPin(but_GPIO_Port, but_Pin))
		  MouseSend(bufor, 0x01, 0x01, 0x01);
	  else
		  MouseSend(bufor, 0x00, 0x01, 0x01);
	  USBD_HID_SendReport(&hUsbDeviceFS, bufor, 4);
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
