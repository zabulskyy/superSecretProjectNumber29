/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "i2c.h"
#include "gpio.h"


/* USER CODE BEGIN Includes */


#define I2C1_DEVICE_ADDRESS      0x68
uint8_t xBuffer[16];

#define seconds 	xBuffer[0]&0xF
#define dseconds 	(xBuffer[0]&(0xF<<4))>>4
#define minutes 	xBuffer[1]&0xF
#define	dminutes 	(xBuffer[1]&(0xF<<4))>>4
#define hours 		xBuffer[2]&0xF
#define dhours 		(xBuffer[2]&(0xF<<4))>>4
#define weekday 	xBuffer[3]&0xF

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t scldel;
uint8_t sdadel;

void i2c_conf() {
    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB -> OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR6 | ~GPIO_OSPEEDER_OSPEEDR7;
    GPIOB -> MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
    GPIOB -> OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;
    GPIOB -> AFR[0] |= (4 << 24) | (4 << 28);

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN ;

    I2C1->CR1 |= I2C_CR1_ANFOFF;

    scldel = 4;
    sdadel = 5;
    I2C1->TIMINGR = 0xF0001317 | ((scldel & 0x0F) << 20) | ((sdadel & 0x0F) << 16);
    I2C1->CR2 |= (0x1E << 1);
    I2C1->CR1 |= I2C_CR1_PE;
}

void i2c_write_byte(uint8_t addr, uint8_t data) {
    I2C1->CR2 &= ~(I2C_CR2_RD_WRN);
    I2C1->CR2 |= I2C_CR2_START |  (2 << 16);
    while(I2C1->CR2 & I2C_CR2_START);
    I2C1->TXDR = addr;
    while (!(I2C1->ISR & I2C_ISR_TXE));

    I2C1->TXDR = data;
    while (!(I2C1->ISR & I2C_ISR_TXE));
    I2C1->CR2 |= I2C_CR2_STOP;
    while(I2C1->CR2 & I2C_CR2_STOP);
}

uint8_t i2c_read_byte(uint8_t addr) {
    uint8_t data = 0;

    I2C1->CR2 &= ~(I2C_CR2_RD_WRN);
    I2C1->CR2 &= ~(0xff << 16);
    I2C1->CR2 |= I2C_CR2_START | (1 << 16);
    while(I2C1->CR2 & I2C_CR2_START);

    I2C1->TXDR = addr;
    while (!(I2C1->ISR & I2C_ISR_TXE));

    I2C1->CR2 |= I2C_CR2_RD_WRN;
    I2C1->CR2 |= I2C_CR2_START | (1 << 16);
    while(I2C1->CR2 & I2C_CR2_START);
    while (!(I2C1->ISR & I2C_ISR_RXNE));
    data = I2C1->RXDR;
    I2C1->CR2 |= I2C_CR2_STOP;
    while(I2C1->CR2 & I2C_CR2_STOP);
    return data;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  extern void initialise_monitor_handles(void);
  initialise_monitor_handles();
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
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint8_t regn[]={0};
  //uint8_t buf[16];
  int i = 1;

  //uint8_t zero[] = {0};
  //uint8_t seconds[] = {0 << 4 | 0};
  //uint8_t minutes[] = {4 << 4 | 3};
  //uint8_t hours[] =   {0 << 4 | 2};
  //uint8_t days[] =    {2};

  //HAL_I2C_Mem_Write(&hi2c1, 0x68*2, 0, 1, /*value*/seconds, 1, 500);  // seconds
  //HAL_I2C_Mem_Write(&hi2c1, 0x68*2, 1, 1, /*value*/minutes, 1, 500);  // minutes
  //HAL_I2C_Mem_Write(&hi2c1, 0x68*2, 2, 1, /*value*/hours, 1, 500);    // hours
  //HAL_I2C_Mem_Write(&hi2c1, 0x68*2, 3, 1, /*value*/days, 1, 500);     // weekday

  while (1)
  {
	  //HAL_StatusTypeDef res = HAL_I2C_Mem_Read(&hi2c1, 0x68*2, 0, 1, xBuffer, /*3*/2, 500);
	  //printf("%i %i:%i %i\n",(xBuffer[0]&(0xF<<4))>>4 ,xBuffer[0]&0xF, xBuffer[1]&(0xF<<4),xBuffer[1]&0xF);
	  //continue;


	  /* HOW TO GET DATA: DOCUMENTATION
	   * seconds:		xBuffer[0]&0xF
	   * 10 seconds:	(xBuffer[0]&(0xF<<4))>>4
	   *
	   * minutes:		xBuffer[1]&0xF
	   * 10 minutes:	(xBuffer[1]&(0xF<<4))>>4
	   *
	   * hours:			xBuffer[2]&0xF
	   * 10 hours:		(xBuffer[2]&(0xF<<4))>>4
	   *
	   * weekday (1-7):	xBuffer[3]&0xF
	   */

	 // HAL_StatusTypeDef res = HAL_I2C_Mem_Read(&hi2c1, 0x68*2, 0, 1, xBuffer, 4, 500);
	  HAL_I2C_Mem_Read(&hi2c1, 0x68*2, 0, 1, xBuffer, 4, 500);
	  printf("weekday: %i\n", weekday);
	  printf("%i%i : %i%i : %i%i\n\n", dhours, hours, dminutes, minutes, dseconds, seconds);
	  continue;






	  // first set the register pointer to the register wanted to be read
	  //HAL_StatusTypeDef res1 = HAL_I2C_Master_Transmit(&hi2c1, 0x68, regn, 1, 100);  // note the & operator which gives us the address of the register_pointer variable
	 // HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);

	  HAL_StatusTypeDef res1 = HAL_I2C_Mem_Read(&hi2c1, i, 0, 1, xBuffer, 1, 500);
	    // receive the 2 x 8bit data into the receive buffer
	  //HAL_StatusTypeDef res2 = HAL_I2C_Master_Receive(&hi2c1, 0x68, xBuffer, 2, 100);

	//  HAL_I2C_Mem_Write(&hi2c1, (uint16_t) I2C1_DEVICE_ADDRESS<<1, 0x00, 1, xBuffer, 1, 5);
	 // HAL_I2C_Mem_Read(&hi2c1, (uint16_t) I2C1_DEVICE_ADDRESS<<1, 0x00, 1, xBuffer, 1, 5);
	  if(res1 == 0)
		  printf("%x: %i, %d\n", i, res1, xBuffer[0]);
	  ++i;
	  if (i==256)
		  i = 1;
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
