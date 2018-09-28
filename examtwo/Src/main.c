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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */
int enable=0;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void pwm_control(int pulse1)//���ռ�ձȱ仯����
{
  if(pulse1>1000)
  {
    pulse1=1000;
    //pulseleft=1000;
  }
   if(pulse1<-1000)
   {
     pulse1=-1000;
     //pulseleft=-1000;
   }
  if(pulse1>0)
  {
   TIM4->CCR1=1000-pulse1;
   TIM4->CCR2=1000;//��
  }
  else if(pulse1<0)
  {
    TIM4->CCR1=1000;
   TIM4->CCR2=1000+pulse1;//��
  }
  if(pulse1==0)
  {
    TIM4->CCR1=1000;
    TIM4->CCR2=1000; 
  }
  //uprintf("TIM4->CCR1=%d,TIM4->CCR2=%d\n",TIM4->CCR1,TIM4->CCR2);
    //uprintf("pulse1=%d,pulse2=%d",pulse1,pulse2);

}
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  TIM4->CCR1=1000;
  TIM4->CCR2=1000;//��
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
int enable1=0;
void HAL_SYSTICK_Callback(){ 
  static int time_1ms;
  time_1ms++;
  if(time_1ms==5){
    time_1ms=0;
   // int speed=TIM2->CNT;
    //TIM2->CNT=0;
    // uprintf("speed=%d\n",speed);
    if(enable1==1)
    {
    sitepidcontrol();
    pidcontrol1();
    }
    else
    {
    pidcontrol2();
    }
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
int speed=0;
//�ٶȻ�
int speedset=0;//300;
int speederroracc=0;
float speedkp=0.6;//0.33
float speedki=0.005;//0.001
float speedkd=-0.4;//-0.38
float speederrorlast=0;
void pidcontrol1()
{
    float error=speedset-speed;
    speederroracc+=error;
    if(speederroracc>100000)
    {
      speederroracc=100000;
    }
    int speedPIDcontrol=(int)(speedkp*error+speedki*speederroracc+speedkd*(error-speederrorlast));
    speederrorlast=error;
    // uprintf("speed=%d\n",speed);
    //uprintf("speedki*speederroracc=%d\n",(int)(speedki*speederroracc));
    pwm_control(speedPIDcontrol);
    //send_wave((float)speed,(float)speedPIDcontrol,0.0,0.0);
}
void pidcontrol2()
{
    speed=TIM2->CNT;
    TIM2->CNT=0;
    if(speed>30000)
     {
        speed=speed-65536;
      }  
    float error=speedset-speed;
    speederroracc+=error;
    /*if(speederroracc>1600000)
    {
      speederroracc=1600000;
    }*/
    int speedPIDcontrol=(int)(speedkp*error+speedki*speederroracc+speedkd*(error-speederrorlast));
    speederrorlast=error;
    // uprintf("speed=%d\n",speed);
    //uprintf("speedki*speederroracc=%d\n",(int)(speedki*speederroracc));
    pwm_control(speedPIDcontrol);
    //send_wave((float)speed,(float)speedPIDcontrol,(float)speedset,0.0);
}
//λ�û�
int sitespeedset=0;//100000;
int sitespeederroracc=0;
float sitespeedkp=0.005;//0.33
//float sitespeedki=0.0004;//0.001
float sitespeedkd=-0.08;//-0.38
float sitespeederrorlast=0;
void sitepidcontrol()
{
    speed=TIM2->CNT;
    TIM2->CNT=0;
    if(speed>30000)
     {
        speed=speed-65536;
      }  
   sitespeedset=sitespeedset-speed;
  if(sitespeedset>200000)
    {
      speedset=1298;
    }
    else
    {
      float error=sitespeedset;
      speedset=(int)(sitespeedkp*error+sitespeedkd*(error-sitespeederrorlast));
      sitespeederrorlast=error;
      
    }
  send_wave((float)speedset,(float)sitespeedset,0.0,0.0);
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
