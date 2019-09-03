#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "can.h"
#include "motorcontrol.h"
#include "key.h"
#include "led.h"
#include <stdbool.h>
#include <string.h>
#include "CpgGenerator.h"
#include "timer.h"
#include "gait_update.h"




/****

2019��4��24�� ����ʵ��CPG���� ��Ͼ��� ��ӱ任�ٶ�ϵ��
							
����ֻ���Ϊ ǰ�к� ���� ÿ��֮ǰ������� ����λ��Ϊpi ͨ��������ͬ�ĳ�ֵ ʵ�������λ��ϣ��Ƿ��ͨ��������ͬ�ĳ�ֵ������Ͼ���ʵ�������ϣ���

δ��������ͨ��������Ͼ���� ϵ�� ʵ�ֻ����˲�̬�任�ٶȵĵ��� 


			|			|
leg0 =|			|=  leg3       groupA
			|			|
			|			|
leg1 =|			|=  leg4			 groupB
			|			|
			|			|
leg2 =|			|=  leg5			 groupC
			|     |

******/





u8 key;
unsigned int pos = 0;
__IO int16_t real_current[6];
__IO int16_t real_velocity[6];
__IO int32_t real_position[6];
__IO int16_t real_angle[6];         
long prev_position[6];


//���ǲ�̬��ʼ��
int legs_stance[6] = {33,33,33,33,33,33};
int legs_swing[6] = {333,333,333,333,333,333};



float legs_dutyf[6]={0.42,0.42,0.42,0.42,0.42,0.42};
float legs_phase[6] = {0,0.5,0,0.5,0,0.5};
int walk_period[6] = {2000,2000,2000,2000,2000,2000}; //720;


u8 USART_RX_CMD[USART_REC_LEN];     //����ָ�����

//extern ��ʾ�˱������ڱ𴦶���� ���˴�����
extern short Real_Current_Value[6];
extern short Real_Velocity_Value[6];
extern unsigned long Real_Position_Value[6];
extern unsigned long time_ms;

extern float omega_to_set;//��Ҫ����omega��ֵ
extern float beta_to_set;//��Ҫ��beta��ֵ
extern float omega_set;
extern float beta_set;
extern float theta[6][6];
extern float k_value[6][6];


extern function_parameters xy_value_new[6];
extern function_parameters coupling_xy_value[6];



//20190521 test 
extern cpg_xy value_xy;


//long delta_time = 0;

int TRIANGLE_MODE = 0;
long motor_pos[6] = {0};  //���ʵ�����е�λ��

int turn_dir[2] = {-1,1};
const int TURN_LEFT = 0;
const int TURN_RIGHT = 1;   //turn_dir��һ����ʾ��������飬�����˵����������෴����ת 0��1��2��ǰ��3��4��5�������ת����֮����ת




vals desired_values = {0};



void set_all_position_(vals desired_vals)
{
	int motor_index = 0;
	int direction[6] = {1};

	for(motor_index = 0;motor_index<6;motor_index++)
			{
				motor_pos[motor_index] += desired_vals.global_theta[motor_index]* 32000/360;
				//if((motor_index == 0)||(motor_index == 1))
					//printf("motor_pos[%d]: %lu\r\n",motor_index,motor_pos[motor_index]);												
			}
			//����������
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,4000,desired_vals.global_velocity[0],motor_pos[0]);		
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,2,4000,desired_vals.global_velocity[1],motor_pos[1]);
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,3,4000,desired_vals.global_velocity[2],motor_pos[2]);
		  //�Ҳ��������
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,4,4000,desired_vals.global_velocity[3],-motor_pos[3]);
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,5,4000,desired_vals.global_velocity[4],-motor_pos[4]);
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,6,4000,desired_vals.global_velocity[5],-motor_pos[5]);	
}



void driver_init()
{
		delay_ms(2000);	  
	  CAN1_Mode_Init(CAN_SJW_1TQ,CAN_BS2_5TQ,CAN_BS1_9TQ,3,CAN_MODE_NORMAL); //CAN��ʼ��,������1000Kbps 		
	  delay_us(200);  
	  CAN_RoboModule_DRV_Reset(0,0);
		delay_ms(500);  //����ʱ�ز����٣�
	  CAN_RoboModule_DRV_Config(0,1,100,0);               
    delay_ms(200);                                     
    CAN_RoboModule_DRV_Config(0,2,100,0);               
    delay_ms(200);                                      
    CAN_RoboModule_DRV_Config(0,3,100,0);               
    delay_us(200);                                     
    CAN_RoboModule_DRV_Config(0,4,100,0);               
		delay_us(200);
		CAN_RoboModule_DRV_Config(0,5,100,0);               
    delay_us(200);
		CAN_RoboModule_DRV_Config(0,6,100,0);     
		delay_ms(500);  //����ʱ�ز����٣�
	  
	/*****λ��ģʽ��******/	
		CAN_RoboModule_DRV_Mode_Choice(0,0,Velocity_Position_Mode);  //�ٶ�λ��ģʽ
	
	
		delay_ms(500);                            //����ʱ�ز�����
}




int main(void)
{
	
	
	
	
	
	
	
	

/*
		CPG�������� �������Ǵ��ؽ׶� state_state  �½��������ս׶� state_swing
		
	 |      /\            /\            /
	 |     /  \          /  \          /
	 |		/    \        /    \        /
 --|---/------\------/------\------/--------------
   |  /		     \    /        \    /
	 | /          \  /          \  /
   |/            \/		         \/
*/	  
		
		//ode initialazation
		int16_t i = 0;
		float xt0 = 1;
		float yt0 = 1;
		float xt1 = -1; //����ͬ��������
		float yt1 = -1;
		
		
		
		function_parameters xy_init0;
		function_parameters xy_init1;	
		
		xy_init0.x = xt0;
		xy_init0.y = yt0;  //�������ĳ�ֵ
	
		xy_init1.x = xt1;
		xy_init1.y = yt1;  //�������ĳ�ֵ
		
		
    HAL_Init();                     //��ʼ��HAL��    
    Stm32_Clock_Init(360,25,2,8);   //����ʱ��,180Mhz
    delay_init(180);                //��ʼ����ʱ����
    uart_init(115200);              //��ʼ��USART
	  LED_Init();                     //��ʼ��LED 
 
    KEY_Init();                     //��ʼ������
		//printf("can init!\r\n");	

		driver_init();   //��ʼ������
		TIM3_Init(10-1,9000-1);       //��ʱ��3��ʼ������ʱ��ʱ��Ϊ90M����Ƶϵ��Ϊ9000-1�
																	//���Զ�ʱ��3��Ƶ��Ϊ90M/9000=10K���Զ���װ��Ϊ20-1����ô��ʱ�����ھ���2ms											
		omega_to_set = omega_set;

		for(i = 0; i<6;i++)
		{		
	
			//xy_value_new[i] = xy_init0;
			//coupling_xy_value[i] = xy_init0;
			
			
			value_xy.x[i] = xy_init0.x;
			value_xy.y[i] = xy_init0.y;

			
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,i+1,5000,0,0); //��ʼʱ������	
		}
		
					
		
		while(1)
		{							
			
			
									
		 //if((Real_Position_Value[0]%32000) <= 10) //�����˴�ʱ�㴦����ֱλ�ø���
			//{											
			//	printf("leg %d �����л��� Ŀ��ֵ�Ѿ�����\r\n",0);																		
			//}				
				
			cpg_test();  //����6·��ϵ�CPG�ź�
			get_cmd_from_pc();  //����˶�ģʽָ��
			gait_mapping();     //����ӳ��
	
			//update_gait();														
			set_all_position_(desired_values);//����������˶�ָ��	
	
			delay_ms(50); 							
	}
}










        