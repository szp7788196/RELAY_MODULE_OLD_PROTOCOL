#include "task_hci.h"
#include "delay.h"
#include "usart.h"
#include "at_protocol.h"
#include "net_protocol.h"


TaskHandle_t xHandleTaskHCI = NULL;

void vTaskHCI(void *pvParameters)
{
	u16 send_len1 = 0;
	u16 send_len2 = 0;
	u16 head_pos = 0xFFFF;
	u16 tail_pos = 0xFFFF;
	u16 recv_pos = 0;
	u16 recv_len = 0;
	u16 frame_len = 0;
	u8 head_buf[6] = {0};
	u8 tail_buf[6] = {0};
	u8 recv_buf[NET_BUF_MAX_LEN] = {0};
	time_t time_s = 0;

	head_buf[0] = 0x68;
	head_buf[1] = 0;
	head_buf[2] = 0;
	head_buf[3] = 0;
	head_buf[4] = 0;
	head_buf[5] = 0;

	tail_buf[0] = 0xFE;
	tail_buf[1] = 0xFD;
	tail_buf[2] = 0xFC;
	tail_buf[3] = 0xFB;
	tail_buf[4] = 0xFA;
	tail_buf[5] = 0xF9;

	AT_CommandInit();

	UsartSendString(USART1,"READY\r\n", 7);

	while(1)
	{
		if(Usart1RecvEnd == 0xAA)
		{
			Usart1RecvEnd = 0;

			send_len1 = AT_CommandDataAnalysis(Usart1RxBuf,Usart1FrameLen,Usart1TxBuf,HoldReg);

			memset(Usart1RxBuf,0,Usart1FrameLen);
		}

		if(send_len1 != 0)
		{
			UsartSendString(USART1,Usart1TxBuf, send_len1);

			memset(Usart1TxBuf,0,send_len1);

			send_len1 = 0;
		}

		recv_len = fifo_get(dl_buf_id,&recv_buf[recv_pos]);

		if(recv_len != 0)
		{
			time_s = GetSysTick1s();

			if(recv_buf[0] == 'A' && recv_buf[1] == 'T')
			{
				send_len2 = AT_CommandDataAnalysis(recv_buf,recv_len,(u8 *)tx_fifo,HoldReg);

				recv_pos = 0;
				recv_len = 0;
			}
			else
			{
				recv_pos = recv_pos + recv_len;
				recv_len = recv_pos;

				ANALYSIS_LOOP:
				head_pos = MyStrstr(recv_buf, head_buf, recv_len, 6);
				tail_pos = MyStrstr(recv_buf, tail_buf, recv_len, 6);

				if(head_pos != 0xFFFF && tail_pos != 0xFFFF)
				{
					if(tail_pos > head_pos)
					{
						frame_len = tail_pos + 6 - head_pos;

						send_len2 += NetDataAnalysis(&recv_buf[head_pos],frame_len,(u8 *)(&tx_fifo[send_len2]),HoldReg);

						if(recv_len > frame_len)	//还有未处理完的数据
						{
							recv_len = recv_len - tail_pos - 6;
							recv_pos = recv_len;

							memcpy(recv_buf,&recv_buf[tail_pos + 6],recv_len);

							goto ANALYSIS_LOOP;
						}
						else						//所有数据均处理完
						{
							recv_pos = 0;
							recv_len = 0;
							frame_len = 0;
						}
					}
				}
			}
		}

		if(recv_pos != 0 || recv_len != 0)
		{
			if(GetSysTick1s() - time_s >= 3)	//接收到数据但是不完整，超时n秒数据作废
			{
				recv_pos = 0;
				recv_len = 0;
				frame_len = 0;
			}
		}

		if(send_len2 != 0)
		{
			DIR_485_TX;

			UsartSendString(USART2,(u8 *)tx_fifo, send_len2);

			DIR_485_RX;

			memset((u8 *)tx_fifo,0,send_len2);

			send_len2 = 0;
		}

		delay_ms(50);
	}
}






































