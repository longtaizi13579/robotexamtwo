/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"
#include "main.h"
#include "gpio.h"
#include "stdarg.h"
#include "string.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */
char buffer_rx_temp;
void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }
   HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
   HAL_NVIC_EnableIRQ(USART1_IRQn);
   HAL_UART_Receive_IT(&huart1,(uint8_t *)&buffer_rx_temp,1);
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
} 
char buffer[255];
int counter=0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
   HAL_UART_Receive_IT(&huart1,(uint8_t *)&buffer_rx_temp,1);
    if(buffer_rx_temp!='\n')
   {
     buffer[counter++]=buffer_rx_temp;   
   }
   else
   {
    examthree(buffer,counter);
    uprintf("counter=%d",counter);
    memset(buffer,'\0',sizeof(char)*255);
    counter=0;
   }       
       
}
char uart_buffer[100 + 1];
void uprintf(char *fmt, ...)
{
	int size;
	
	va_list arg_ptr;
	
	va_start(arg_ptr, fmt);  
	
	size=vsnprintf(uart_buffer, 100 + 1, fmt, arg_ptr);
	va_end(arg_ptr);
        HAL_UART_Transmit(&huart1,(uint8_t *)uart_buffer,size,1000);
}
char s[22]={'b','y',16,6};
void send_wave(float arg1,float arg2,float arg3,float arg4){

  s[2]=16;  //length
  s[3]=6;   //type
  s[20]='\r';
  s[21]='\n';
  memcpy(s+4,&arg1,sizeof(arg1));
  memcpy(s+8,&arg2,sizeof(arg1));
  memcpy(s+12,&arg3,sizeof(arg1));
  memcpy(s+16,&arg4,sizeof(arg1));
  HAL_UART_Transmit(&huart1,(uint8_t *)s, 22,1000);

}
/* USER CODE BEGIN 1 */
void examthree(char* command,int count)//��������
{
  char one[5];//char* a,b;b��char����char*?
  char two[6];
  char three[13];
  char other[15];
  //char four[4];
  memset(one,'\0',sizeof(char)*5);
  memset(two,'\0',sizeof(char)*6);
  memset(three,'\0',sizeof(char)*13);
   memset(other,'\0',sizeof(char)*15);
  for(int i=0;i<4;i++)
  {
    one[i]=command[i];
  }
  for(int i=0;i<5;i++)
  {
    two[i]=command[i];
  }
  for(int i=0;i<12;i++)
  {
    three[i]=command[i];
  }
  //�������
  if(!(strcmp(one,"site")))
  {
    for(int i=5;i<count;i++)
    {
      other[i-5]=command[i];
       uprintf("%c",command[i]);
    }
   sitespeedset=trans(other);//100000;
   enable1=1;
   //uprintf("1%s\n",other);
   //uprintf("2%d\n",sitespeedset);
  }
  else if(!(strcmp(one,"help")))
  {
    uprintf("����λ��:site ***\n�����ٶȣ�speed ***\n������ԣ�chassis goto * * *\n���������help\n");
  }
  else if(!(strcmp(two,"speed")))
  {
  for(int i=6;i<count;i++)
    {
      other[i-6]=command[i];
      uprintf("%c",command[i]);
    }
  speedset=trans(other);
  enable1=0;
  //uprintf("3%s\n",other);
 // uprintf("4%d\n",speedset);
  }
  else if(!(strcmp(three,"chassis goto")))
  {
    uprintf("chassis goto13456789\n");
  
  }
  else
  {
    uprintf("������������\n");
  }
}

int trans(char* s)
{
  int re=0;
  while(*s!='\0')
  {
    re=re*10+*s-'0';
    s++;
  }
  return re;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
