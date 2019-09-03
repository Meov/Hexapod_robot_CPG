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

2019Äê4ÔÂ24ÈÕ ³ÌĞòÊµÏÖCPGÍøÂç ñîºÏ¾ØÕó Ìí¼Ó±ä»»ËÙ¶ÈÏµÊı
							
½«ÁùÖ»×ã·ÖÎª Ç°ÖĞºó Èı×é Ã¿×éÖ®Ç°»¥ÏàñîºÏ ÆäÏàÎ»²îÎªpi Í¨¹ı¸ø¶¨²»Í¬µÄ³õÖµ ÊµÏÖ×é¼äÏàÎ»ñîºÏ£¨ÊÇ·ñ¿ÉÍ¨¹ı¸ø¶¨ÏàÍ¬µÄ³õÖµ£¬ÓÃñîºÏ¾ØÕóÊµÏÖ×é¼äñîºÏ£¿£©

Î´À´¹¤×÷£ºÍ¨¹ıµ÷ÕûñîºÏ¾ØÕóµÄ ÏµÊı ÊµÏÖ»úÆ÷ÈË²½Ì¬±ä»»ËÙ¶ÈµÄµ÷Õû 


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


//Èı½Ç²½Ì¬³õÊ¼»¯
int legs_stance[6] = {33,33,33,33,33,33};
int legs_swing[6] = {333,333,333,333,333,333};



float legs_dutyf[6]={0.42,0.42,0.42,0.42,0.42,0.42};
float legs_phase[6] = {0,0.5,0,0.5,0,0.5};
int walk_period[6] = {2000,2000,2000,2000,2000,2000}; //720;


u8 USART_RX_CMD[USART_REC_LEN];     //½ÓÊÕÖ¸Áî»º³åÇø

//extern ±íÊ¾´Ë±äÁ¿ÊÇÔÚ±ğ´¦¶¨ÒåµÄ £¬´Ë´¦ÒıÓÃ
extern short Real_Current_Value[6];
extern short Real_Velocity_Value[6];
extern unsigned long Real_Position_Value[6];
extern unsigned long time_ms;

extern float omega_to_set;//½«Òª¸³¸øomegaµÄÖµ
extern float beta_to_set;//½«Òª¸øbeta¸³Öµ
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
long motor_pos[6] = {0};  //µç»úÊµ¼ÊÔËĞĞµÄÎ»ÖÃ

int turn_dir[2] = {-1,1};
const int TURN_LEFT = 0;
const int TURN_RIGHT = 1;   //turn_dirÊÇÒ»¸ö±íÊ¾·½ÏòµÄÊı×é£¬»úÆ÷ÈËµ¥Á½²àÂÖÏòÏà·´µÄĞı×ª 0£¬1£¬2ÏòÇ°£¬3£¬4£¬5ÏòºóÏò×ó×ª£¬·´Ö®ÏòÓÒ×ª




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
			//×ó²àÈı¸öµç»ú
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,4000,desired_vals.global_velocity[0],motor_pos[0]);		
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,2,4000,desired_vals.global_velocity[1],motor_pos[1]);
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,3,4000,desired_vals.global_velocity[2],motor_pos[2]);
		  //ÓÒ²àÈı¸öµç»ú
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,4,4000,desired_vals.global_velocity[3],-motor_pos[3]);
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,5,4000,desired_vals.global_velocity[4],-motor_pos[4]);
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,6,4000,desired_vals.global_velocity[5],-motor_pos[5]);	
}



void driver_init()
{
		delay_ms(2000);	  
	  CAN1_Mode_Init(CAN_SJW_1TQ,CAN_BS2_5TQ,CAN_BS1_9TQ,3,CAN_MODE_NORMAL); //CAN³õÊ¼»¯,²¨ÌØÂÊ1000Kbps 		
	  delay_us(200);  
	  CAN_RoboModule_DRV_Reset(0,0);
		delay_ms(500);  //´ËÑÓÊ±±Ø²»¿ÉÉÙ£¡
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
		delay_ms(500);  //´ËÑÓÊ±±Ø²»¿ÉÉÙ£¡
	  
	/*****Î»ÖÃÄ£Ê½ÏÂ******/	
		CAN_RoboModule_DRV_Mode_Choice(0,0,Velocity_Position_Mode);  //ËÙ¶ÈÎ»ÖÃÄ£Ê½
	
	
		delay_ms(500);                            //´ËÑÓÊ±±Ø²»¿ÉÉÙ
}




int main(void)
{
	
	
	
	
	
	
	
	

/*
		CPG²¨ĞÎÈçÏÂ ÉÏÉıÑØÊÇ´¥µØ½×¶Î state_state  ÏÂ½µÑØÊÇĞü¿Õ½×¶Î state_swing
		
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
		float xt1 = -1; //¿ØÖÆÍ¬²àÏàÁÚ×ã
		float yt1 = -1;
		
		
		
		function_parameters xy_init0;
		function_parameters xy_init1;	
		
		xy_init0.x = xt0;
		xy_init0.y = yt0;  //´øñîºÏÏîµÄ³õÖµ
	
		xy_init1.x = xt1;
		xy_init1.y = yt1;  //´øñîºÏÏîµÄ³õÖµ
		
		
    HAL_Init();                     //³õÊ¼»¯HAL¿â    
    Stm32_Clock_Init(360,25,2,8);   //ÉèÖÃÊ±ÖÓ,180Mhz
    delay_init(180);                //³õÊ¼»¯ÑÓÊ±º¯Êı
    uart_init(115200);              //³õÊ¼»¯USART
	  LED_Init();                     //³õÊ¼»¯LED 
 
    KEY_Init();                     //³õÊ¼»¯°´¼ü
		//printf("can init!\r\n");	

		driver_init();   //³õÊ¼»¯Çı¶¯
		TIM3_Init(10-1,9000-1);       //¶¨Ê±Æ÷3³õÊ¼»¯£¬¶¨Ê±Æ÷Ê±ÖÓÎª90M£¬·ÖÆµÏµÊıÎª9000-1£
																	//ËùÒÔ¶¨Ê±Æ÷3µÄÆµÂÊÎª90M/9000=10K£¬×Ô¶¯ÖØ×°ÔØÎª20-1£¬ÄÇÃ´¶¨Ê±Æ÷ÖÜÆÚ¾ÍÊÇ2ms											
		omega_to_set = omega_set;

		for(i = 0; i<6;i++)
		{		
	
			//xy_value_new[i] = xy_init0;
			//coupling_xy_value[i] = xy_init0;
			
			
			value_xy.x[i] = xy_init0.x;
			value_xy.y[i] = xy_init0.y;

			
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,i+1,5000,0,0); //³õÊ¼Ê±¿ÌËø¶¨	
		}
		
					
		
		while(1)
		{							
			
			
									
		 //if((Real_Position_Value[0]%32000) <= 10) //»úÆ÷ÈË´ËÊ±×ã´¦ÓÚÊúÖ±Î»ÖÃ¸½½ü
			//{											
			//	printf("leg %d µ½´ïÇĞ»»µã Ä¿±êÖµÒÑ¾­ÖÃÁã\r\n",0);																		
			//}				
				
			cpg_test();  //²úÉú6Â·ñîºÏµÄCPGĞÅºÅ
			get_cmd_from_pc();  //»ñµÃÔË¶¯Ä£Ê½Ö¸Áî
			gait_mapping();     //¶¯×÷Ó³Éä
	
			//update_gait();														
			set_all_position_(desired_values);//¸øµç»ú·¢ËÍÔË¶¯Ö¸Áî	
	
			delay_ms(50); 							
	}
}










        