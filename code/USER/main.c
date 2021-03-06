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

2019年4月24日 程序实现CPG网络 耦合矩阵 添加变换速度系数
							
将六只足分为 前中后 三组 每组之前互相耦合 其相位差为pi 通过给定不同的初值 实现组间相位耦合（是否可通过给定相同的初值，用耦合矩阵实现组间耦合？）

未来工作：通过调整耦合矩阵的 系数 实现机器人步态变换速度的调整 


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


//三角步态初始化
int legs_stance[6] = {33,33,33,33,33,33};
int legs_swing[6] = {333,333,333,333,333,333};



float legs_dutyf[6]={0.42,0.42,0.42,0.42,0.42,0.42};
float legs_phase[6] = {0,0.5,0,0.5,0,0.5};
int walk_period[6] = {2000,2000,2000,2000,2000,2000}; //720;


u8 USART_RX_CMD[USART_REC_LEN];     //接收指令缓冲区

//extern 表示此变量是在别处定义的 ，此处引用
extern short Real_Current_Value[6];
extern short Real_Velocity_Value[6];
extern unsigned long Real_Position_Value[6];
extern unsigned long time_ms;

extern float omega_to_set;//将要赋给omega的值
extern float beta_to_set;//将要给beta赋值
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
long motor_pos[6] = {0};  //电机实际运行的位置

int turn_dir[2] = {-1,1};
const int TURN_LEFT = 0;
const int TURN_RIGHT = 1;   //turn_dir是一个表示方向的数组，机器人单两侧轮向相反的旋转 0，1，2向前，3，4，5向后向左转，反之向右转




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
			//左侧三个电机
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,4000,desired_vals.global_velocity[0],motor_pos[0]);		
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,2,4000,desired_vals.global_velocity[1],motor_pos[1]);
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,3,4000,desired_vals.global_velocity[2],motor_pos[2]);
		  //右侧三个电机
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,4,4000,desired_vals.global_velocity[3],-motor_pos[3]);
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,5,4000,desired_vals.global_velocity[4],-motor_pos[4]);
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,6,4000,desired_vals.global_velocity[5],-motor_pos[5]);	
}



void driver_init()
{
		delay_ms(2000);	  
	  CAN1_Mode_Init(CAN_SJW_1TQ,CAN_BS2_5TQ,CAN_BS1_9TQ,3,CAN_MODE_NORMAL); //CAN初始化,波特率1000Kbps 		
	  delay_us(200);  
	  CAN_RoboModule_DRV_Reset(0,0);
		delay_ms(500);  //此延时必不可少！
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
		delay_ms(500);  //此延时必不可少！
	  
	/*****位置模式下******/	
		CAN_RoboModule_DRV_Mode_Choice(0,0,Velocity_Position_Mode);  //速度位置模式
	
	
		delay_ms(500);                            //此延时必不可少
}




int main(void)
{
	
	
	
	
	
	
	
	

/*
		CPG波形如下 上升沿是触地阶段 state_state  下降沿是悬空阶段 state_swing
		
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
		float xt1 = -1; //控制同侧相邻足
		float yt1 = -1;
		
		
		
		function_parameters xy_init0;
		function_parameters xy_init1;	
		
		xy_init0.x = xt0;
		xy_init0.y = yt0;  //带耦合项的初值
	
		xy_init1.x = xt1;
		xy_init1.y = yt1;  //带耦合项的初值
		
		
    HAL_Init();                     //初始化HAL库    
    Stm32_Clock_Init(360,25,2,8);   //设置时钟,180Mhz
    delay_init(180);                //初始化延时函数
    uart_init(115200);              //初始化USART
	  LED_Init();                     //初始化LED 
 
    KEY_Init();                     //初始化按键
		//printf("can init!\r\n");	

		driver_init();   //初始化驱动
		TIM3_Init(10-1,9000-1);       //定时器3初始化，定时器时钟为90M，分频系数为9000-1�
																	//所以定时器3的频率为90M/9000=10K，自动重装载为20-1，那么定时器周期就是2ms											
		omega_to_set = omega_set;

		for(i = 0; i<6;i++)
		{		
	
			//xy_value_new[i] = xy_init0;
			//coupling_xy_value[i] = xy_init0;
			
			
			value_xy.x[i] = xy_init0.x;
			value_xy.y[i] = xy_init0.y;

			
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,i+1,5000,0,0); //初始时刻锁定	
		}
		
					
		
		while(1)
		{							
			
			
									
		 //if((Real_Position_Value[0]%32000) <= 10) //机器人此时足处于竖直位置附近
			//{											
			//	printf("leg %d 到达切换点 目标值已经置零\r\n",0);																		
			//}				
				
			cpg_test();  //产生6路耦合的CPG信号
			get_cmd_from_pc();  //获得运动模式指令
			gait_mapping();     //动作映射
	
			//update_gait();														
			set_all_position_(desired_values);//给电机发送运动指令	
	
			delay_ms(50); 							
	}
}










        