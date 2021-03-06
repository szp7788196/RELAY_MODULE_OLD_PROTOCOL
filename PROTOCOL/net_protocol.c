#include "net_protocol.h"
#include "rtc.h"
#include "usart.h"
#include "24cxx.h"
#include "common.h"
#include "relay.h"

//网络数据帧协议解析
u16 NetDataAnalysis(u8 *buf,u16 len,u8 *outbuf,u8 *hold_reg)
{
	u8 i = 0;
	u16 ret = 0;
	u16 pos1 = 0;
	u16 got_uuid = 0xFFFF;
	u8 uuid_type = 0;		//0:HEX		1:ASCII
	u8 response = 0;

	u8 area_id = 0;
	u8 box_id = 0;
	u8 *uuid = NULL;
	u8 cmd_code = 0;
	u16 data_len = 0;
	u8 *data = NULL;
	u8 read_check_sum = 0;
	u8 cal_check_sum = 0;

	u8 buf_tail[6] = {0};
	u8 address_buf[6] = {0x00,0x00,0x00,0x00,0x00,0x02};

	buf_tail[0] = 0xFE;
	buf_tail[1] = 0xFD;
	buf_tail[2] = 0xFC;
	buf_tail[3] = 0xFB;
	buf_tail[4] = 0xFA;
	buf_tail[5] = 0xF9;

	pos1 = MyStrstr(buf,buf_tail,len,6);

	area_id = *(buf + 8);								//获取逻辑区码
	box_id = *(buf + 9);								//获取逻辑区码
	uuid = buf + 10;									//获取UUID
	cmd_code = *(buf + 27);								//获取功能码
//	data_len = ((((u16)(*(buf + 28))) << 8) & 0xFF00) +
//	           (((u16)(*(buf + 29))) & 0x00FF);			//获取数据长度

	data_len = *(buf + 28);
	data = buf + 29;									//获取数据域

	if(pos1 != 0xFFFF)
	{
		if(*(buf + 0) == 0x68 && \
			*(buf + 7) == 0x68 && \
			*(buf + pos1 - 1) == 0x16)							//判断包头和包尾
		{
			if(MyStrstr(buf + 1,address_buf,6,6) != 0xFFFF)
			{
				read_check_sum = *(buf + pos1 - 2);					//获取校验和
				cal_check_sum = CalCheckSum(buf, pos1 - 2);			//计算校验和

				if(read_check_sum == cal_check_sum)
				{
					if(area_id == 0xFF && box_id == 0xFF)
					{
						if(DeviceUUID != NULL)
						{
							got_uuid = MyStrstr(uuid,DeviceUUID,len - 10,UU_ID_LEN - 2);

							if(got_uuid == 0xFFFF)
							{
								for(i = 0; i < 17; i ++)
								{
									*(uuid + i) = *(uuid + i) + 0x30;
								}

								got_uuid = MyStrstr(uuid,DeviceUUID,len - 10,UU_ID_LEN - 2);
							}
							else
							{
								uuid_type = 1;
							}

							if(got_uuid == 0xFFFF)
							{
								return 0;
							}
							else
							{
								response = 1;
							}
						}
					}
					else if((area_id == DeviceAreaID && box_id == DeviceBoxID) ||
							(area_id == DeviceAreaID && box_id == 0xFE) ||
							(area_id == 0xFE && box_id == 0xFE))
					{
						response = 2;
					}
					else
					{
						return 0;
					}

					if(response != 0)
					{
						switch(cmd_code)
						{
							case 0xD0:									//发送固定信息(心跳)，上行
								ret = UpdateRelayModeInfo(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0xD1:									//控制继电器开闭状态
								ret = ControlRelayState(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0xD2:									//重启设备
								ret = ControlDeviceReset(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0xD3:									//设置定时发送间隔,下行
								ret = SetDeviceUpLoadINCL(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0xD4:									//读取/发送信息
								ret = ReadDeviceInfo(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0xD5:									//从服务器获取时间
								ret = GetTimeDateFromServer(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0xD6:									//设置继电器定时策略，下行
								ret = SetRegularTimeGroups(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0xD7:									//设置设备工作模式
								ret = SetDeviceWorkMode(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0xD8:									//设置定时发送间隔,下行
								ret = SetRelayActionINCL(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0xD9:									//设置定时发送间隔,下行
								ret = SetRS485BuarRate(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0xDA:									//设置逻辑区和物理区,下行
								ret = SetAreaID_BoxID(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0xDB:									//设置逻辑区和物理区,下行
								ret = SetUpdateFirmWareInfo(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0xDC:									//设置逻辑区和物理区,下行
								ret = WriteFrameWareBags(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0xDD:									//设置位置信息
								ret = SetPosition(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;
							
							case 0xDE:									//设置位置信息
								ret = GetRegularTimeGroups(cmd_code,data,data_len,outbuf,response,uuid_type);
							break;

							case 0x80:									//应答，下行,上行在别处处理
								UnPackAckPacket(cmd_code,data,data_len);
							break;

							default:									//此处要给云端应答一个功能码错误信息

							break;
						}
					}
				}
			}
		}
		else	//此处可以给云端应答一个帧头错误信息
		{

		}
	}
	else		//此处可以给云端应答一个校验错误信息
	{

	}

	return ret;
}

//解析ACK包
u8 UnPackAckPacket(u8 cmd_code,u8 *buf,u16 len)
{
	u8 ret = 0;

	if(len == 2)
	{
		if(*(buf + 1) == 0)
		{
			ret = 1;
		}
	}

	return ret;
}

//ACK打包
u16 PackAckPacket(u8 cmd_code,u8 *data,u8 *outbuf,u8 id_type)
{
	u16 len = 0;

	len = PackNetData(0x80,data,2,outbuf,id_type);

	return len;
}

u16 UpdateRelayModeInfo(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;

	u8 data_buf[8] = {0,0,0,0,0,0,0,0};

	if(len == 0)
	{
		if(resp == 1)
		{
			out_len = PackDataOfRelayInfo(data_buf);
			out_len = PackNetData(cmd_code,data_buf,out_len,outbuf,id_type);
		}
	}

	return out_len;
}

//控制继电器开闭
u16 ControlRelayState(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	u16 bit = 0;
	u16 state = 0;

	u32 relay_state = 0;

	data_buf[0] = cmd_code;

	if(len == 4)												//数据长度必须是64
	{
		bit = ((u16)(*(buf + 0)) << 8) + (*(buf + 1));
		state = ((u16)(*(buf + 2)) << 8) + (*(buf + 3));

//		if(bit <= 0x0FFF && state <= 0x0FFF)
//		{
			relay_state = bit;
			relay_state = relay_state << 16;
			relay_state = relay_state + state;

			if(xQueueSend(xQueue_RelayState,(void *)&relay_state,(TickType_t)10) != pdPASS)
			{
#ifdef DEBUG_LOG
				printf("send p_tSensorMsg fail 2.\r\n");
#endif
			}

			HaveNewActionCommand = 1;
//		}
//		else
//		{
//			data_buf[1] = 1;
//		}
	}
	else
	{
		data_buf[1] = 2;
	}

	if(resp == 1)
	{
		out_len = PackAckPacket(cmd_code,data_buf,outbuf,id_type);
	}

	return out_len;
}

//下发OTA命令
u16 SetUpdateFirmWareInfo(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u16 i = 0;
	u8 out_len = 0;
	u8 data_buf[5] = {0,0,0,0,0};

	if(len == 6)
	{
		if(*(buf + 0) <= 100)
		{
			FrameWareInfo.version = (((u16)(*(buf + 0))) << 8) + (u16)(*(buf + 1));

			FrameWareInfo.length = (((u32)(*(buf + 2))) << 24) +
								   (((u32)(*(buf + 3))) << 16) +
								   (((u32)(*(buf + 4))) << 8) +
								   (((u32)(*(buf + 5))) << 0);

			memcpy(&HoldReg[SOFT_WARE_INFO_ADD],buf,SOFT_WARE_INFO_LEN - 2);

			WriteDataFromHoldBufToEeprom(&HoldReg[SOFT_WARE_INFO_ADD],
										SOFT_WARE_INFO_ADD,
										SOFT_WARE_INFO_LEN - 2);	//将数据写入EEPROM

			if(FrameWareInfo.length > FIRMWARE_SIZE)
			{
				data_buf[0] = 0;
			}
			else
			{
				u16 page_num = 0;

				FrameWareState.state 			= FIRMWARE_DOWNLOADING;
				FrameWareState.total_bags 		= FrameWareInfo.length % FIRMWARE_BAG_SIZE != 0 ?
												  FrameWareInfo.length / FIRMWARE_BAG_SIZE + 1 : FrameWareInfo.length / FIRMWARE_BAG_SIZE;
				FrameWareState.current_bag_cnt 	= 1;
				FrameWareState.bag_size 		= FIRMWARE_BAG_SIZE;
				FrameWareState.last_bag_size 	= FrameWareInfo.length % FIRMWARE_BAG_SIZE != 0 ?
												  FrameWareInfo.length % FIRMWARE_BAG_SIZE : FIRMWARE_BAG_SIZE;
				FrameWareState.total_size 		= FrameWareInfo.length;

				WriteFrameWareStateToEeprom();	//将固件升级状态写入EEPROM

				page_num = (FIRMWARE_MAX_FLASH_ADD - FIRMWARE_BUCKUP_FLASH_BASE_ADD) / 2048;	//得到备份区的扇区总数

				FLASH_Unlock();						//解锁FLASH

				for(i = 0; i < page_num; i ++)
				{
					FLASH_ErasePage(i * 2048 + FIRMWARE_BUCKUP_FLASH_BASE_ADD);	//擦除当前FLASH扇区
				}

				FLASH_Lock();						//上锁

				data_buf[0] = 1;
				data_buf[1] = (u8)(FrameWareState.total_bags >> 8);
				data_buf[2] = (u8)(FrameWareState.total_bags & 0x00FF);
				data_buf[3] = (u8)(FrameWareState.current_bag_cnt >> 8);
				data_buf[4] = (u8)(FrameWareState.current_bag_cnt & 0x00FF);
			}
		}
	}
	else
	{
		data_buf[0] = 0;
	}

	if(resp == 1)
	{
		out_len = PackNetData(cmd_code,data_buf,5,outbuf,id_type);
	}

	return out_len;
}

//下发更新固件命令
u16 WriteFrameWareBags(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;
	u8 data_buf[5] = {0,0,0,0,0};

	u16 i = 0;
	u16 total_bags = 0;
	u16 current_bags = 0;
	u16 bag_size = 0;
	u8 *msg = NULL;
	u16 crc_read = 0;
	u16 crc_cal = 0;
	u32 crc32_cal = 0xFFFFFFFF;
	u32 crc32_read = 0;
	u8 crc32_cal_buf[1024];
	u32 file_len = 0;
	u16 k_num = 0;
	u16 last_k_byte_num = 0;
	u16 temp = 0;

	data_buf[0] = 1;

	if(len == 6 + FIRMWARE_BAG_SIZE)
	{
		total_bags = (((u16)(*(buf + 0))) << 8) + (u16)(*(buf + 1));
		current_bags = (((u16)(*(buf + 2))) << 8) + (u16)(*(buf + 3));
		bag_size = (((u16)(*(buf + 4))) << 8) + (u16)(*(buf + 5));

		if(total_bags != FrameWareState.total_bags ||
		   current_bags > FrameWareState.total_bags)		//总包数匹配错误
		{
			return 0;
		}

		msg = buf + 6;

		crc_read = (((u16)(*(msg + bag_size - 2))) << 8) + (u16)(*(msg + bag_size - 1));

		crc_cal = GetCRC16(msg,bag_size - 2);

		if(crc_cal == crc_read)
		{
			if(current_bags == FrameWareState.current_bag_cnt)
			{
				if(current_bags < FrameWareState.total_bags)
				{
					FLASH_Unlock();						//解锁FLASH

					if(bag_size == FIRMWARE_BAG_SIZE)
					{
						for(i = 0; i < (FIRMWARE_BAG_SIZE - 2) / 2; i ++)
						{
							temp = ((u16)(*(msg + i * 2 + 1)) << 8) + (u16)(*(msg + i * 2));

							FLASH_ProgramHalfWord(FIRMWARE_BUCKUP_FLASH_BASE_ADD + (current_bags - 1) * (FIRMWARE_BAG_SIZE - 2) + i * 2,temp);
						}
					}

					FLASH_Lock();						//上锁

					FrameWareState.current_bag_cnt ++;

					FrameWareState.state = FIRMWARE_DOWNLOADING;	//当前包下载完成
				}
				else if(current_bags == FrameWareState.total_bags)
				{
					crc32_read = (((u32)(*(msg + 0))) << 24) +
					             (((u32)(*(msg + 1))) << 16) +
					             (((u32)(*(msg + 2))) << 8) +
					             (((u32)(*(msg + 3))));

					file_len = 128 * (FrameWareState.total_bags - 1);

					k_num = file_len / 1024;
					last_k_byte_num = file_len % 1024;
					if(last_k_byte_num > 0)
					{
						k_num += 1;
					}

					for(i = 0; i < k_num; i ++)
					{
						memset(crc32_cal_buf,0,1024);
						if(i < k_num - 1)
						{
							STMFLASH_ReadBytes(FIRMWARE_BUCKUP_FLASH_BASE_ADD + 1024 * i,crc32_cal_buf,1024);
							crc32_cal = CRC32(crc32_cal_buf,1024,crc32_cal,0);
						}
						if(i == k_num - 1)
						{
							if(last_k_byte_num == 0)
							{
								STMFLASH_ReadBytes(FIRMWARE_BUCKUP_FLASH_BASE_ADD + 1024 * i,crc32_cal_buf,1024);
								crc32_cal = CRC32(crc32_cal_buf,1024,crc32_cal,1);
							}
							else if(last_k_byte_num > 0)
							{
								STMFLASH_ReadBytes(FIRMWARE_BUCKUP_FLASH_BASE_ADD + 1024 * i,crc32_cal_buf,last_k_byte_num);
								crc32_cal = CRC32(crc32_cal_buf,last_k_byte_num,crc32_cal,1);
							}
						}
					}

					if(crc32_read == crc32_cal)
					{
						FrameWareState.state = FIRMWARE_DOWNLOADED;				//全部下载完成

						data_buf[0] = 2;
					}
					else
					{
						FrameWareState.state = FIRMWARE_DOWNLOAD_FAILED;		//全部下载完成
						data_buf[0] = 3;
					}

					WriteFrameWareStateToEeprom();
				}
			}
		}
	}

	data_buf[1] = (u8)(FrameWareState.total_bags >> 8);
	data_buf[2] = (u8)(FrameWareState.total_bags & 0x00FF);
	data_buf[3] = (u8)(FrameWareState.current_bag_cnt >> 8);
	data_buf[4] = (u8)(FrameWareState.current_bag_cnt & 0x00FF);

	if(resp == 1)
	{
		out_len = PackNetData(cmd_code,data_buf,5,outbuf,id_type);
	}

	return out_len;
}

//远程重启
u16 ControlDeviceReset(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	data_buf[0] = cmd_code;

	if(len == 0)
	{
		NeedToReset = 1;
	}
	else
	{
		data_buf[1] = 2;
	}

	if(resp == 1)
	{
		out_len = PackAckPacket(cmd_code,data_buf,outbuf,id_type);
	}

	return out_len;
}

//设置设备数据上传时间间隔
u16 SetDeviceUpLoadINCL(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	u16 incl;

	data_buf[0] = cmd_code;

	if(len == 2)												//数据长度必须是64
	{
		incl = ((u16)(*(buf + 0)) << 8) + (*(buf + 1));

		if(incl <= MAX_UPLOAD_INVL)
		{
			UpLoadINCL = incl;

			memcpy(&HoldReg[UPLOAD_INVL_ADD],buf,2);
			WriteDataFromHoldBufToEeprom(&HoldReg[UPLOAD_INVL_ADD],UPLOAD_INVL_ADD, UPLOAD_INVL_LEN - 2);
		}
		else
		{
			data_buf[1] = 1;
		}
	}
	else
	{
		data_buf[1] = 2;
	}

	if(resp == 1)
	{
		out_len = PackAckPacket(cmd_code,data_buf,outbuf,id_type);
	}

	return out_len;
}

//读取继电器信息
u16 ReadDeviceInfo(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;

	u8 data_buf[3] = {0};

	if(len == 0)
	{
		if(resp == 1)
		{
			data_buf[0] = DeviceWorkMode;
			data_buf[1] = (u8)(((u16)SOFT_WARE_VRESION) >> 8);
			data_buf[2] = (u8)SOFT_WARE_VRESION;

			out_len = PackNetData(cmd_code,data_buf,3,outbuf,id_type);
		}
	}

	return out_len;
}

//从服务器获取时间戳
u16 GetTimeDateFromServer(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	u8 year = 0;
	u8 mon = 0;
	u8 day = 0;
	u8 hour = 0;
	u8 min = 0;
	u8 sec = 0;

	data_buf[0] = cmd_code;

	if(len  == 6)												//数据长度必须是64
	{
		year = *(buf + 0);
		mon  = *(buf + 1);
		day  = *(buf + 2);
		hour = *(buf + 3);
		min  = *(buf + 4);
		sec  = *(buf + 5);

		if(year >= 18 && mon <= 12 && day <= 31 && hour <= 23 && min <= 59 && sec <= 59)
		{
			RTC_Set(year + 2000,mon,day,hour,min,sec);

			GetTimeOK = 1;
		}
		else
		{
			data_buf[1] = 1;
		}
	}
	else
	{
		data_buf[1] = 2;
	}

	if(resp == 1)
	{
		out_len = PackAckPacket(cmd_code,data_buf,outbuf,id_type);
	}

	return out_len;
}

//设置策略时间
u16 SetRegularTimeGroups(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;
	u8 group_num = 0;
	u16 i = 0;
	u16 j = 0;
	u16 k = 0;
	u8 data_buf[2] = {0,0};
	u8 time_group[MAX_GROUP_NUM * TIME_RULE_LEN];
	u16 crc16 = 0;

	data_buf[0] = cmd_code;

	if(len % 10 == 0)							//数据长度必须是10的倍数
	{
		group_num = len / 10;									//计算下发了几组数据

		if(group_num <= MAX_GROUP_NUM)	//组数必须是2的倍数，并且要小于MAX_GROUP_NUM
		{
			RemoveAllStrategy();				//删除所有本地存储策略

			TimeGroupNumber = group_num;

			crc16 = CRC16(&group_num,1);

			AT24CXX_WriteOneByte(TIME_GROUP_NUM_ADD + 0,TimeGroupNumber);
			AT24CXX_WriteOneByte(TIME_GROUP_NUM_ADD + 1,(u8)(crc16 >> 8));
			AT24CXX_WriteOneByte(TIME_GROUP_NUM_ADD + 2,(u8)(crc16 & 0x00FF));

			memset(time_group,0,MAX_GROUP_NUM * TIME_RULE_LEN);

			k = 0;

			for(i = 0; i < group_num; i ++)
			{
				for(j = i * TIME_RULE_LEN; j < i * TIME_RULE_LEN + 10; j ++, k ++)
				{
					time_group[j] = *(buf + k);
				}

				crc16 = CRC16(&time_group[j - 10],10);

				time_group[j + 0] = (u8)(crc16 >> 8);
				time_group[j + 1] = (u8)(crc16 & 0x00FF);
			}

			for(i = 0; i < group_num; i ++)
			{
				pRegularTime tmp_time = NULL;

				tmp_time = (pRegularTime)mymalloc(sizeof(RegularTime_S));

				tmp_time->prev = NULL;
				tmp_time->next = NULL;

				tmp_time->number 		= i;
				tmp_time->type 			= time_group[i * TIME_RULE_LEN + 0];
				tmp_time->year 			= time_group[i * TIME_RULE_LEN + 1];
				tmp_time->month 		= time_group[i * TIME_RULE_LEN + 2];
				tmp_time->date 			= time_group[i * TIME_RULE_LEN + 3];
				tmp_time->hour 			= time_group[i * TIME_RULE_LEN + 4];
				tmp_time->minute 		= time_group[i * TIME_RULE_LEN + 5];
				tmp_time->control_bit	= (((u16)time_group[i * TIME_RULE_LEN + 6]) << 8) + (u16)time_group[i * TIME_RULE_LEN + 7];
				tmp_time->control_state	= (((u16)time_group[i * TIME_RULE_LEN + 8]) << 8) + (u16)time_group[i * TIME_RULE_LEN + 9];

				switch(tmp_time->type)
				{
					case TYPE_WEEKDAY:
						RegularTimeGroupAdd(TYPE_WEEKDAY,tmp_time);
					break;

					case TYPE_HOLIDAY_START:
						RegularTimeGroupAdd(TYPE_HOLIDAY_START,tmp_time);
					break;

					case TYPE_HOLIDAY_END:
						RegularTimeGroupAdd(TYPE_HOLIDAY_END,tmp_time);
					break;

					default:

					break;
				}

			}

			for(i = 0; i < group_num * TIME_RULE_LEN + group_num; i ++)				//每组7个字节+2个字节(CRC16)
			{
				AT24CXX_WriteOneByte(TIME_RULE_ADD + i,time_group[i]);
			}
		}

		RefreshStrategy = 1;	//需要刷新策略状态
	}
	else
	{
		data_buf[1] = 2;
	}

	if(resp == 1)
	{
		out_len = PackAckPacket(cmd_code,data_buf,outbuf,id_type);
	}

	return out_len;
}

//获取调光策略
u16 GetRegularTimeGroups(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u16 i = 0;
	u16 j = 0;
	u16 read_crc = 0;
	u16 cal_crc = 0;
	u8 data_buf[MAX_GROUP_NUM * TIME_RULE_LEN];
	u8 read_success_buf_flag[MAX_GROUP_NUM];
	u16 out_len = 0;

	if(len == 0)
	{
		for(i = 0; i < MAX_GROUP_NUM; i ++)
		{
			for(j = i * TIME_RULE_LEN; j < i * TIME_RULE_LEN + TIME_RULE_LEN; j ++)
			{
				data_buf[j] = AT24CXX_ReadOneByte(TIME_RULE_ADD + j);
			}

			cal_crc = GetCRC16(&data_buf[j - TIME_RULE_LEN],TIME_RULE_LEN - 2);
			read_crc = (((u16)data_buf[j - 2]) << 8) + (u16)data_buf[j - 1];

			if(cal_crc == read_crc)
			{
				read_success_buf_flag[i] = 1;
			}
		}

		for(i = 0; i < MAX_GROUP_NUM; i ++)
		{
			if(read_success_buf_flag[i] == 1)
			{
				memcpy(data_buf + i * (TIME_RULE_LEN - 2),data_buf + i * TIME_RULE_LEN,TIME_RULE_LEN - 2);
			}
			else
			{
				break;
			}
		}

		out_len = i * (TIME_RULE_LEN - 2);
	}

	if(resp == 1)
	{
		out_len = PackNetData(cmd_code,data_buf,out_len,outbuf,id_type);
	}

	return out_len;
}

//控制设备的工作模式
u16 SetDeviceWorkMode(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;
	u8 mode = 0;
	u8 data_buf[2] = {0,0};
	data_buf[0] = cmd_code;

	if(len == 1)
	{
		mode = *(buf + 0);

		if(mode == 0 || mode == 1)
		{
			DeviceWorkMode = mode;			//置工作模式标志
		}
		else
		{
			data_buf[1] = 1;
		}
	}
	else
	{
		data_buf[1] = 2;
	}

	if(resp == 1)
	{
		out_len = PackAckPacket(cmd_code,data_buf,outbuf,id_type);
	}

	return out_len;
}

//设置设备的UUID
u16 SetDeviceUUID(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	u8 uuid_buf[38];

	data_buf[0] = cmd_code;

	if(len == UU_ID_LEN - 2)												//数据长度必须是64
	{
		memset(uuid_buf,0,38);

		memcpy(&HoldReg[UU_ID_ADD],buf,36);

		GetDeviceUUID();

		WriteDataFromHoldBufToEeprom(&HoldReg[UU_ID_ADD],UU_ID_ADD, UU_ID_LEN - 2);
	}
	else
	{
		data_buf[1] = 2;
	}

	if(resp == 1)
	{
		out_len = PackAckPacket(cmd_code,data_buf,outbuf,id_type);
	}

	return out_len;
}

//设置继电器动作间隔
u16 SetRelayActionINCL(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	u16 incl;

	data_buf[0] = cmd_code;

	if(len == 2)												//数据长度必须是64
	{
		incl = ((u16)(*(buf + 0)) << 8) + (*(buf + 1));

		if(incl <= MAX_REALY_ACTION_INVL)
		{
			RelayActionINCL = incl;

			memcpy(&HoldReg[REALY_ACTION_INVL_ADD],buf,2);
			WriteDataFromHoldBufToEeprom(&HoldReg[REALY_ACTION_INVL_ADD],REALY_ACTION_INVL_ADD, REALY_ACTION_INVL_LEN - 2);
		}
		else
		{
			data_buf[1] = 1;
		}
	}
	else
	{
		data_buf[1] = 2;
	}

	if(resp == 1)
	{
		out_len = PackAckPacket(cmd_code,data_buf,outbuf,id_type);
	}

	return out_len;
}

//设置继电器动作间隔
u16 SetRS485BuarRate(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	u32 baud_rate = 9600;

	data_buf[0] = cmd_code;

	if(len == 4)												//数据长度必须是64
	{
		baud_rate = ((u32)(*(buf + 0)) << 24) + ((u32)(*(buf + 1)) << 16) + ((u32)(*(buf + 2)) << 8) + (u32)(*(buf + 3));

		if(baud_rate <= 19200)
		{
			RS485BuadRate = baud_rate;

			memcpy(&HoldReg[RS485_BUAD_RATE_ADD],buf,4);
			WriteDataFromHoldBufToEeprom(&HoldReg[RS485_BUAD_RATE_ADD],RS485_BUAD_RATE_ADD, RS485_BUAD_RATE_LEN - 2);

			USART2_Init(RS485BuadRate);
		}
		else
		{
			data_buf[1] = 1;
		}
	}
	else
	{
		data_buf[1] = 2;
	}

	if(resp == 1)
	{
		out_len = PackAckPacket(cmd_code,data_buf,outbuf,id_type);
	}

	return out_len;
}

//设置逻辑区和物理区
u16 SetAreaID_BoxID(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	data_buf[0] = cmd_code;

	if(len == 2)
	{
		if(*(buf + 0) < 0xFE && *(buf + 1) < 0xFE)
		{
			DeviceAreaID = *(buf + 0);
			memcpy(&HoldReg[AREA_ID_ADD],&DeviceAreaID,AREA_ID_LEN - 2);
			WriteDataFromHoldBufToEeprom(&HoldReg[AREA_ID_ADD],AREA_ID_ADD, AREA_ID_LEN - 2);

			DeviceBoxID = *(buf + 1);
			memcpy(&HoldReg[BOX_ID_ADD],&DeviceBoxID,BOX_ID_LEN - 2);
			WriteDataFromHoldBufToEeprom(&HoldReg[BOX_ID_ADD],BOX_ID_ADD, BOX_ID_LEN - 2);
		}
		else
		{
			data_buf[1] = 1;
		}
	}
	else
	{
		data_buf[1] = 2;
	}

	if(resp == 1)
	{
		out_len = PackAckPacket(cmd_code,data_buf,outbuf,id_type);
	}

	return out_len;
}

//设置位置信息
u16 SetPosition(u8 cmd_code,u8 *buf,u16 len,u8 *outbuf,u8 resp,u8 id_type)
{
	u8 out_len = 0;
	u8 i = 0;
	u8 data_buf[2] = {0,0};
	u8 temp_buf[POSITION_LEN];

	data_buf[0] = cmd_code;

	if(len == POSITION_LEN - 2)
	{
		memset(temp_buf,0,POSITION_LEN);

		memcpy(&HoldReg[POSITION_ADD],buf,POSITION_LEN - 2);

		for(i = 0; i < 8; i ++)
		{
			temp_buf[i] = HoldReg[POSITION_ADD + 7 - i];
		}

		memcpy(&Location.longitude,temp_buf,8);

		for(i = 0; i < 8; i ++)
		{
			temp_buf[i] = HoldReg[POSITION_ADD + 15 - i];
		}

		memcpy(&Location.latitude,temp_buf,8);

		WriteDataFromHoldBufToEeprom(&HoldReg[POSITION_ADD],POSITION_ADD, POSITION_LEN - 2);
	}
	else
	{
		data_buf[1] = 2;
	}

	if(resp == 1)
	{
		out_len = PackAckPacket(cmd_code,data_buf,outbuf,id_type);
	}

	return out_len;
}




































