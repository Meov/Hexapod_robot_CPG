#ifndef __CAN_H
#define __CAN_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F746������
//CAN��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/12/29
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//CAN1����RX0�ж�ʹ��
#define CAN1_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.



u8 CAN1_Mode_Init(u32 tsjw,u32 tbs2,u32 tbs1,u16 brp,u32 mode);//CAN��ʼ��
u8 CAN1_Send_Msg(u8* msg,u8 len);						//��������
u8 CAN1_Receive_Msg(u8 *buf);							//��������
u8 CAN_Transmit(CanTxMsgTypeDef* TxMessage);//�������� ����Ϊ1��
//u8 CAN_Receive(CanRxMsgTypeDef* rxMessage);
#define abs(x) ((x)>0? (x):(-(x)))
#endif
