#include "sys.h"
#include "usart.h"
#include "common.h"


RingBuf ring_fifo;
uint8_t rx_fifo[NET_BUF_MAX_LEN];
int8_t dl_buf_id = -1;
int8_t tx_fifo[NET_BUF_MAX_LEN];

FIFO(dl_buf,1,NET_BUF_MAX_LEN);

//#define UART_DMA 1
#define MAX_RCV_LEN NET_BUF_MAX_LEN
#ifdef UART_DMA
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
DMA_Channel_TypeDef *USARx_DMA[4]={NULL,DMA1_Channel5,DMA1_Channel6,DMA1_Channel3};
DMA_Channel_TypeDef *USATx_DMA[4]={NULL,DMA1_Channel4,DMA1_Channel7,DMA1_Channel2};
#endif


u16 Usart1RxCnt = 0;
u16 OldUsart1RxCnt = 0;
u16 Usart1FrameLen = 0;
u8 Usart1RxBuf[Usart1RxLen];
u8 Usart1TxBuf[Usart1TxLen];
u8 Usart1RecvEnd = 0;
u8 Usart1Busy = 0;
u16 Usart1SendLen = 0;
u16 Usart1SendNum = 0;


//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
	int handle;

};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
_sys_exit(int x)
{
	x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
    USART1->DR = (u8)ch;
	return ch;
}
#endif

void USART1_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	USART_Cmd(USART1, DISABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);		//使能USART1，GPIOA时钟
	USART_DeInit(USART1);  															//复位串口1

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 										//PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;									//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure); 											//初始化PA9

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;										//PA.10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;							//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);  										//初始化PA10

	USART_InitStructure.USART_BaudRate = bound;										//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

	USART_Init(USART1, &USART_InitStructure); 										//初始化串口
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);									//开启中断
	USART_Cmd(USART1, ENABLE);                    									//使能串口
}


u8 UsartSendString(USART_TypeDef* USARTx,u8 *str, u16 len)
{
	u16 i;
	for(i=0; i<len; i++)
    {
		USART_ClearFlag(USARTx,USART_FLAG_TC);
		USART_SendData(USARTx, str[i]);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
		USART_ClearFlag(USARTx,USART_FLAG_TC);
	}
	return 1;
}

void USART1_IRQHandler(void)
{
	u8 rxdata;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  	{
		rxdata =USART_ReceiveData(USART1);

		if(Usart1RxCnt<Usart1RxLen)
		{
			Usart1RxBuf[Usart1RxCnt]=rxdata;
			Usart1RxCnt++;
		}
  	}

	if(USART_GetITStatus(USART1,USART_IT_TC)!=RESET)
	{
		Usart1FrameSend();
	}

	//以下为串口中断出错后的处理  经验之谈
	else if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
	{
		rxdata = USART_ReceiveData(USART1);
		rxdata = rxdata;
		USART_ClearFlag(USART1, USART_FLAG_ORE);
	}
	else if(USART_GetFlagStatus(USART1, USART_FLAG_NE) != RESET)
	{
		USART_ClearFlag(USART1, USART_FLAG_NE);
	}
	else if(USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET)
	{
		USART_ClearFlag(USART1, USART_FLAG_FE);
	}
	else if(USART_GetFlagStatus(USART1, USART_FLAG_PE) != RESET)
	{
		USART_ClearFlag(USART1, USART_FLAG_PE);
	}
}


void Usart1ReciveFrameEnd(void)
{
	if(Usart1RxCnt)
	{
		if(OldUsart1RxCnt == Usart1RxCnt)
		{
			Usart1FrameLen = Usart1RxCnt;
			OldUsart1RxCnt = 0;
			Usart1RxCnt = 0;
			Usart1RecvEnd = 0xAA;
		}
		else
		{
			OldUsart1RxCnt = Usart1RxCnt;
		}
	}
}


void Usart1FrameSend(void)
{
	u8 send_data = 0;
	send_data = Usart1TxBuf[Usart1SendNum];
	USART_SendData(USART1,send_data);
	Usart1SendNum ++;
	if(Usart1SendNum >= Usart1SendLen)					//发送已经完成
	{
		Usart1Busy = 0;
		Usart1SendLen = 0;								//要发送的字节数清零
		Usart1SendNum = 0;								//已经发送的字节数清零
		USART_ITConfig(USART1, USART_IT_TC, DISABLE);	//关闭数据发送中断
		memset(Usart1TxBuf,0,Usart1TxLen);
	}
}

/*
 *  @brief USART2初始化函数
 */
void USART2_Init(u32 bound)
{
	u8 first = 1;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
#ifdef UART_DMA
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	USART_DeInit(USART2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	DIR_485_RX;

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	if(first == 1)
	{
		first = 0;
		
		ringbuf_init(&ring_fifo, rx_fifo, sizeof(rx_fifo));
		dl_buf_id=fifo_init(&dl_buf);
	}

	USART_Init(USART2, &USART_InitStructure);
	
#ifdef UART_DMA
	//DMA1通道4配置
	DMA_DeInit(USARx_DMA[2]);
	//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);
	//内存地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rx_fifo;
	//dma传输方向单向
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	//设置DMA在传输时缓冲区的长度
	DMA_InitStructure.DMA_BufferSize = MAX_RCV_LEN;
	//设置DMA的外设递增模式，一个外设
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//设置DMA的内存递增模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//外设数据字长
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	//内存数据字长
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	//设置DMA的传输模式
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	//设置DMA的优先级别
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	//设置DMA的2个memory中的变量互相访问
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(USARx_DMA[2],&DMA_InitStructure);

	DMA_Cmd(USARx_DMA[2],ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
#else
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
#endif
	USART_Cmd(USART2, ENABLE);
}

/*
*  @brief USART2串口发送api
*/
void USART2_Write(uint8_t *Data, uint32_t len)
{

#ifndef UART_DMA
    uint32_t i;
	
    USART_ClearFlag(USART2, USART_FLAG_TC);
	
    for(i = 0; i < len; i++)
    {
        USART_SendData(USART2, *Data++);
		
        while( USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET );
    }

#else
    DMA_InitTypeDef DMA_InitStruct;
    DMA_Cmd(USATx_DMA[2],DISABLE);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);
    DMA_InitStruct.DMA_MemoryBaseAddr = (u32)Data;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_BufferSize = len;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(USATx_DMA[2],&DMA_InitStruct);
    DMA_Cmd(USATx_DMA[2],ENABLE);
#endif
}

/*
*  @brief USART2串口中断
*/
void USART2_IRQHandler(void)
{
	unsigned int data;
	
	if(USART2->SR & 0x0F)
	{
		data = USART2->DR;
		
		data = data;		//没有实际用途，只是为了消除警告
	}
#ifndef UART_DMA
	else if(USART2->SR & USART_FLAG_RXNE)
	{
		data = USART2->DR;
		ringbuf_put(&ring_fifo,data);
		
		if(ringbuf_elements(&ring_fifo) == 1)
			USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	}
#endif
	else if(USART2->SR & USART_FLAG_IDLE)
	{
		data = USART2->SR;
		data = USART2->DR;
		
#ifndef UART_DMA
		USART_ITConfig(USART2, USART_IT_IDLE, DISABLE);
#else
		DMA_Cmd(USARx_DMA[2], DISABLE);  //先停止DMA才行设置缓冲区大小
		ring_fifo.put_ptr = MAX_RCV_LEN - DMA_GetCurrDataCounter(USARx_DMA[2]);
#endif

		fifo_put(dl_buf_id,ringbuf_elements(&ring_fifo),ring_fifo.data);
		
//		if((uint8_t *)strstr((const char *)ring_fifo.data, "+MIPL") != NULL && (uint8_t *)strstr((const char *)ring_fifo.data, "+MIPLCREATE") == NULL)
//		{
//			fifo_put(dl_buf_id,ringbuf_elements(&ring_fifo),ring_fifo.data);
//		}
//		else if((uint8_t *)strstr((const char *)ring_fifo.data, "REBOOT_CAUSE_UNKNOWN") != NULL)
//		{
//			ReConnectToServer = 0x81;
//		}
//		else if((uint8_t *)strstr((const char *)ring_fifo.data, "+CEREG:1") != NULL)
//		{

//		}
//		else
//		{
//			rsp_ok = 1;
//			ring_fifo1.get_ptr = ring_fifo.get_ptr;
//			ring_fifo1.put_ptr = ring_fifo.put_ptr;
//			memcpy(ring_fifo1.data,ring_fifo.data,ringbuf_elements(&ring_fifo));
//		}
		
		ringbuf_clear(&ring_fifo);
	}
	
#ifdef UART_DMA
	DMA_SetCurrDataCounter(USARx_DMA[2],MAX_RCV_LEN);   //重新设置DMA的读取缓冲区长度
	DMA_Cmd(USARx_DMA[2], ENABLE);  //开启DMA
#endif
}


void TIM2_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 		//时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; 					//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 					//设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 				//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 			//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_ITConfig(TIM2,TIM_IT_Update ,ENABLE);
	 							//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM2, ENABLE);  									//使能TIMx外设
}

void TIM2_IRQHandler(void)
{
	static u8 tick_10ms = 0;

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) 			//检查指定的TIM中断发生与否:TIM 中断源
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  			//清除TIMx的中断待处理位:TIM 中断源

		SysTick10msAdder();			//10ms滴答计数器累加

		Usart1ReciveFrameEnd();		//检测USART1是否接收数据结束

		tick_10ms ++;
		if(tick_10ms >= 10)
		{
			tick_10ms = 0;

			SysTick100msAdder();	//100ms滴答计数器累加
		}
	}
}



























