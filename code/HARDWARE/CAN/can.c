#include "can.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F429������
//CAN��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/12/29
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									////////////////////////////////////////////////
  
////////////////  //////////////////
CAN_HandleTypeDef   CAN1_Handler;   //CAN1���
CanTxMsgTypeDef     TxMessage;      //������Ϣ
CanRxMsgTypeDef     RxMessage;      //������Ϣ

short Real_Current_Value[6];
short Real_Velocity_Value[6];
unsigned long Real_Position_Value[6];


char Real_Online[6] = {0};
char Real_Ctl1_Value[6] = {0};
char Real_Ctl2_Value[6] = {0};

////CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1TQ~CAN_SJW_4TQ
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1TQ~CAN_BS2_8TQ;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1TQ~CAN_BS1_16TQ
//brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+tbs2+1)*brp); ����tbs1��tbs2����ֻ�ù�ע��ʶ���ϱ�־����ţ�����CAN_BS2_1TQ�����Ǿ���Ϊtbs2=1�����㼴�ɡ�
//mode:CAN_MODE_NORMAL,��ͨģʽ;CAN_MODE_LOOPBACK,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ45M,�������CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_8tq,6,CAN_MODE_LOOPBACK);
//������Ϊ:45M/((6+8+1)*6)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��; 
u8 CAN1_Mode_Init(u32 tsjw,u32 tbs2,u32 tbs1,u16 brp,u32 mode)
{
    CAN_FilterConfTypeDef  CAN1_FilerConf;
    
    CAN1_Handler.Instance=CAN1; 
    CAN1_Handler.pTxMsg=&TxMessage;     //������Ϣ
    CAN1_Handler.pRxMsg=&RxMessage;     //������Ϣ
    CAN1_Handler.Init.Prescaler=brp;    //��Ƶϵ��(Fdiv)Ϊbrp+1
    CAN1_Handler.Init.Mode=mode;        //ģʽ���� 
    CAN1_Handler.Init.SJW=tsjw;         //����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1TQ~CAN_SJW_4TQ
    CAN1_Handler.Init.BS1=tbs1;         //tbs1��ΧCAN_BS1_1TQ~CAN_BS1_16TQ
    CAN1_Handler.Init.BS2=tbs2;         //tbs2��ΧCAN_BS2_1TQ~CAN_BS2_8TQ
    CAN1_Handler.Init.TTCM=DISABLE;     //��ʱ�䴥��ͨ��ģʽ 
    CAN1_Handler.Init.ABOM=DISABLE;     //����Զ����߹���
    CAN1_Handler.Init.AWUM=DISABLE;     //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
    CAN1_Handler.Init.NART=ENABLE;      //��ֹ�����Զ����� 
    CAN1_Handler.Init.RFLM=DISABLE;     //���Ĳ�����,�µĸ��Ǿɵ� 
    CAN1_Handler.Init.TXFP=DISABLE;     //���ȼ��ɱ��ı�ʶ������ 
	
    if(HAL_CAN_Init(&CAN1_Handler)!=HAL_OK) return 1;   //��ʼ��
    
    CAN1_FilerConf.FilterIdHigh=0X0000;     //32λID
    CAN1_FilerConf.FilterIdLow=0X0000;
    CAN1_FilerConf.FilterMaskIdHigh=0X0000; //32λMASK
    CAN1_FilerConf.FilterMaskIdLow=0X0000;  
    CAN1_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;//������0������FIFO0
    CAN1_FilerConf.FilterNumber=0;          //������0
    CAN1_FilerConf.FilterMode=CAN_FILTERMODE_IDMASK;
    CAN1_FilerConf.FilterScale=CAN_FILTERSCALE_32BIT;
    CAN1_FilerConf.FilterActivation=ENABLE; //�����˲���0
    CAN1_FilerConf.BankNumber=14;
	
    if(HAL_CAN_ConfigFilter(&CAN1_Handler,&CAN1_FilerConf)!=HAL_OK) return 2;//�˲�����ʼ��
	
    return 0;
}

//CAN�ײ��������������ã�ʱ�����ã��ж�����
//�˺����ᱻHAL_CAN_Init()����
//hcan:CAN���
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_CAN1_CLK_ENABLE();                //ʹ��CAN1ʱ��
    __HAL_RCC_GPIOA_CLK_ENABLE();			    //����GPIOAʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_11|GPIO_PIN_12;   //PA11,12
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //���츴��
    GPIO_Initure.Pull=GPIO_PULLUP;              //����
    GPIO_Initure.Speed=GPIO_SPEED_FAST;         //����
    GPIO_Initure.Alternate=GPIO_AF9_CAN1;       //����ΪCAN1
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);         //��ʼ��
    
#if CAN1_RX0_INT_ENABLE
    __HAL_CAN_ENABLE_IT(&CAN1_Handler,CAN_IT_FMP0);//FIFO0��Ϣ�����ж�����.	  
    //CAN1->IER|=1<<1;		//FIFO0��Ϣ�����ж�����.	
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn,1,2);    //��ռ���ȼ�1�������ȼ�2
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);          //ʹ���ж�
#endif	
}

#if CAN1_RX0_INT_ENABLE                         //ʹ��RX0�ж�
//CAN�жϷ�����
void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&CAN1_Handler);//�˺��������CAN_Receive_IT()��������
}

//CAN�жϴ������
//�˺����ᱻCAN_Receive_IT()����
//hcan:CAN���


//���������ݵĺ�����Ĭ��Ϊ4����������������0�飬���Ϊ1��2��3��4
/*************************************************************************
                          CAN1_RX0_IRQHandler
������CAN1�Ľ����жϺ���
*************************************************************************/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
   
    //CAN_Receive_IT()������ر�FIFO0��Ϣ�Һ��жϣ����������Ҫ���´�
    __HAL_CAN_ENABLE_IT(&CAN1_Handler,CAN_IT_FMP0);//���¿���FIF00��Ϣ�Һ��ж�
	
        if((CAN1_Handler.pRxMsg->IDE == CAN_ID_STD)&&(CAN1_Handler.pRxMsg->IDE == CAN_RTR_DATA)&&(CAN1_Handler.pRxMsg->DLC == 8)) //��׼֡������֡�����ݳ���Ϊ8
        {
            
						//���������ת�١��ٶȡ�λ�ö�ȡ
						if(CAN1_Handler.pRxMsg->StdId == 0x1B) //������һ�������
            {
                Real_Current_Value[0] = (CAN1_Handler.pRxMsg->Data[0]<<8)|(CAN1_Handler.pRxMsg->Data[1]);
                Real_Velocity_Value[0] = (CAN1_Handler.pRxMsg->Data[2]<<8)|(CAN1_Handler.pRxMsg->Data[3]);
                Real_Position_Value[0] = ((CAN1_Handler.pRxMsg->Data[4]<<24)|(CAN1_Handler.pRxMsg->Data[5]<<16)|(CAN1_Handler.pRxMsg->Data[6]<<8)|(CAN1_Handler.pRxMsg->Data[7]));
            }
            else if(CAN1_Handler.pRxMsg->StdId == 0x2B)
            {
                Real_Current_Value[1] = (CAN1_Handler.pRxMsg->Data[0]<<8)|(CAN1_Handler.pRxMsg->Data[1]);
                Real_Velocity_Value[1] = (CAN1_Handler.pRxMsg->Data[2]<<8)|(CAN1_Handler.pRxMsg->Data[3]);
                Real_Position_Value[1] = ((CAN1_Handler.pRxMsg->Data[4]<<24)|(CAN1_Handler.pRxMsg->Data[5]<<16)|(CAN1_Handler.pRxMsg->Data[6]<<8)|(CAN1_Handler.pRxMsg->Data[7]));
            }
            else if(CAN1_Handler.pRxMsg->StdId == 0x3B)
            {
                Real_Current_Value[2] = (CAN1_Handler.pRxMsg->Data[0]<<8)|(CAN1_Handler.pRxMsg->Data[1]);
                Real_Velocity_Value[2] = (CAN1_Handler.pRxMsg->Data[2]<<8)|(CAN1_Handler.pRxMsg->Data[3]);
                Real_Position_Value[2] = ((CAN1_Handler.pRxMsg->Data[4]<<24)|(CAN1_Handler.pRxMsg->Data[5]<<16)|(CAN1_Handler.pRxMsg->Data[6]<<8)|(CAN1_Handler.pRxMsg->Data[7]));
            }
            else if(CAN1_Handler.pRxMsg->StdId == 0x4B)
            {
                Real_Current_Value[3] = (CAN1_Handler.pRxMsg->Data[0]<<8)|(CAN1_Handler.pRxMsg->Data[1]);
                Real_Velocity_Value[3] = (CAN1_Handler.pRxMsg->Data[2]<<8)|(CAN1_Handler.pRxMsg->Data[3]);
                Real_Position_Value[3] = ((CAN1_Handler.pRxMsg->Data[4]<<24)|(CAN1_Handler.pRxMsg->Data[5]<<16)|(CAN1_Handler.pRxMsg->Data[6]<<8)|(CAN1_Handler.pRxMsg->Data[7]));
            }
						else if(CAN1_Handler.pRxMsg->StdId == 0x5B)
            {
                Real_Current_Value[4] = (CAN1_Handler.pRxMsg->Data[0]<<8)|(CAN1_Handler.pRxMsg->Data[1]);
                Real_Velocity_Value[4] = (CAN1_Handler.pRxMsg->Data[2]<<8)|(CAN1_Handler.pRxMsg->Data[3]);
                Real_Position_Value[4] = ((CAN1_Handler.pRxMsg->Data[4]<<24)|(CAN1_Handler.pRxMsg->Data[5]<<16)|(CAN1_Handler.pRxMsg->Data[6]<<8)|(CAN1_Handler.pRxMsg->Data[7]));
            }
						else if(CAN1_Handler.pRxMsg->StdId == 0x6B)
            {
                Real_Current_Value[5] = (CAN1_Handler.pRxMsg->Data[0]<<8)|(CAN1_Handler.pRxMsg->Data[1]);
                Real_Velocity_Value[5] = (CAN1_Handler.pRxMsg->Data[2]<<8)|(CAN1_Handler.pRxMsg->Data[3]);
                Real_Position_Value[5] = ((CAN1_Handler.pRxMsg->Data[4]<<24)|(CAN1_Handler.pRxMsg->Data[5]<<16)|(CAN1_Handler.pRxMsg->Data[6]<<8)|(CAN1_Handler.pRxMsg->Data[7]));
            }
						
						
						
            else if(CAN1_Handler.pRxMsg->StdId == 0x1F)
            {
                Real_Online[0] = 1;
            }
            else if(CAN1_Handler.pRxMsg->StdId == 0x2F)
            {
                Real_Online[1] = 1;
            }
            else if(CAN1_Handler.pRxMsg->StdId == 0x3F)
            {
                Real_Online[2] = 1;
            }
            else if(CAN1_Handler.pRxMsg->StdId == 0x4F)
            {
                Real_Online[3] = 1;
            }
            else if(CAN1_Handler.pRxMsg->StdId == 0x1C)
            {
                Real_Ctl1_Value[0] = CAN1_Handler.pRxMsg->Data[0];
                Real_Ctl2_Value[0] = CAN1_Handler.pRxMsg->Data[1];
            }
            else if(CAN1_Handler.pRxMsg->StdId == 0x2C)
            {
                Real_Ctl1_Value[1] = CAN1_Handler.pRxMsg->Data[0];
                Real_Ctl2_Value[1] = CAN1_Handler.pRxMsg->Data[1];
            }
            else if(CAN1_Handler.pRxMsg->StdId == 0x3C)
            {
                Real_Ctl1_Value[2] = CAN1_Handler.pRxMsg->Data[0];
                Real_Ctl2_Value[2] = CAN1_Handler.pRxMsg->Data[1];
            }
            else if(CAN1_Handler.pRxMsg->StdId == 0x4C)
            {
                Real_Ctl1_Value[3] = CAN1_Handler.pRxMsg->Data[0];
                Real_Ctl2_Value[3] = CAN1_Handler.pRxMsg->Data[1];
            }

        }
	/*
    printf("id:%d\r\n",CAN1_Handler.pRxMsg->StdId);
    printf("ide:%d\r\n",CAN1_Handler.pRxMsg->IDE);
    printf("rtr:%d\r\n",CAN1_Handler.pRxMsg->RTR);
    printf("len:%d\r\n",CAN1_Handler.pRxMsg->DLC);
				
		//printf("---------spd_0:%d\r\n", Real_Velocity_Value[0]);
		printf("---------pos_0:%lu\r\n", Real_Position_Value[0]);
    printf("---------pos_1:%lu\r\n", Real_Position_Value[1]);
		printf("---------pos_2:%lu\r\n", Real_Position_Value[2]);
		printf("---------pos_3:%lu\r\n", Real_Position_Value[3]);
		printf("---------pos_4:%lu\r\n", Real_Position_Value[4]);
		printf("---------pos_5:%lu\r\n", Real_Position_Value[5]);
		//for(i=0;i<8;i++)
    //printf("rxbuf[%d]:%d\r\n",i,CAN1_Handler.pRxMsg->Data[i]);
		*/		
}
#endif	

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
    u16 i=0;
    CAN1_Handler.pTxMsg->StdId=0X12;        //��׼��ʶ��
    CAN1_Handler.pTxMsg->ExtId=0x12;        //��չ��ʶ��(29λ)
    CAN1_Handler.pTxMsg->IDE=CAN_ID_STD;    //ʹ�ñ�׼֡
    CAN1_Handler.pTxMsg->RTR=CAN_RTR_DATA;  //����֡
    CAN1_Handler.pTxMsg->DLC=len;                
    for(i=0;i<len;i++)
    CAN1_Handler.pTxMsg->Data[i]=msg[i];
    if(HAL_CAN_Transmit(&CAN1_Handler,10)!=HAL_OK) return 1;     //����
    return 0;		
}

u8 CAN_Transmit(CanTxMsgTypeDef* TxMessage)
{	
    CAN1_Handler.pTxMsg = TxMessage;        //��׼��ʶ��
    if(HAL_CAN_Transmit(&CAN1_Handler,10)!=HAL_OK) return 1;     //����
    return 0;		
}


//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
    if(HAL_CAN_Receive(&CAN1_Handler,CAN_FIFO0,0)!=HAL_OK) return 0;//�������ݣ���ʱʱ������Ϊ0	
    for(i=0;i<CAN1_Handler.pRxMsg->DLC;i++)
    buf[i]=CAN1_Handler.pRxMsg->Data[i];
	return CAN1_Handler.pRxMsg->DLC;	
}

/*
u8 CAN_Receive(CanRxMsgTypeDef* rxMessage)
{	
	if(HAL_CAN_Receive(&CAN1_Handler,CAN_FIFO0,0)!=HAL_OK)
	{
	return 1;//����
	}
	rxMessage = CAN1_Handler.pRxMsg;
	return CAN1_Handler.pRxMsg->DLC;	
}
*/

