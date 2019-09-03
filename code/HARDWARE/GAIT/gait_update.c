#include "gait_update.h"
extern float omega_set;
extern float beta_set;
extern vals desired_values;
extern unsigned long Real_Position_Value[6];
int i = 0;


const int FORWARD = 1;  //����
const int REVERSE = -1;	//��ת
const int GAIT_INIT =  0;  //ûʲô���� ����ռ��ʼʱ�̵�λ����
const int TRIANGLE_FORWARD = 2;
const int TRIANGLE_BACKWARD = 3;
const int LEFT = 4;
const int RIGHT = 5;
const int STAIR = 6;
const int CLIMB = 7;

	
int turn_left[6] = {FORWARD,FORWARD,FORWARD,REVERSE,REVERSE,REVERSE};
int turn_right[6] = {REVERSE,REVERSE,REVERSE,FORWARD,FORWARD,FORWARD};		
int direction_reverse[6] = {REVERSE,REVERSE,REVERSE,REVERSE,REVERSE,REVERSE};
int direction_forward[6] = {FORWARD,FORWARD,FORWARD,FORWARD,FORWARD,FORWARD};  //���ǲ�̬	
unsigned int gait_mode[6] = {GAIT_INIT,GAIT_INIT,GAIT_INIT,GAIT_INIT,GAIT_INIT,GAIT_INIT};			  //��ʼʱ��ʱ���ǲ�̬ģʽ
unsigned int gait_mode_last[6] = {GAIT_INIT,GAIT_INIT,GAIT_INIT,GAIT_INIT,GAIT_INIT,GAIT_INIT};			

leg_info leg_info_set[6] = {0,0,0,0,0,0};

int direction_to_set[6] = {0,0,0,0,0,0};

//20190521 test
cpg_xy value_xy;

int direction_set[6] = {0,0,0,0,0,0};
		
bool turn_flag[6] = {false};	


//��̬������ر���
function_parameters xy_value_new[6] = {0,0,0,0,0,0};
function_parameters coupling_xy_value[6] = {0,0,0,0,0,0};

float position[6] = {0,0,0,0,0,0};
float position_last[6] = {0,0,0,0,0,0};
short speed[6] = {0,0,0,0,0,0};
float position_delta[6] =  {0,0,0,0,0,0};
static unsigned long running_theta[6] = {0,0,0,0,0,0};
long theta_change[6] = {0,0,0,0,0,0};//���м���ĵ��ʵ��λ����
unsigned long running_theta_last[6] = {0,0,0,0,0,0};
static float omega_last = 0;
float velecity_change_pro = 1;   //���Ƚ׶��ٶȱ仯����
		
static unsigned long time_ms_last = 0;
extern unsigned long time_ms;

extern float omega_to_set;//��Ҫ����omega��ֵ
extern float beta_to_set;//��Ҫ��beta��ֵ

long delta_time = 0;

int16_t speed_swing = 0;
int16_t speed_stance = 0;
bool state_swing[6] = {false,false,false,false,false,false};		
bool state_stance[6] = {false,false,false,false,false,false};		
unsigned int T_swing[6] = {0,0,0,0,0,0}; 
unsigned int T_stance[6] = {0,0,0,0,0,0}; 

static unsigned int T_swing_last[6] = {0,0,0,0,0,0};
static unsigned int T_stance_last[6] = {0,0,0,0,0,0};


float amplitude_up = 0;  //�����ط�ֵ
float amplitude_down = 0; //�½��ط�ֵ
//ת���־λ 
bool is_change[6] = {false,false,false,false,false,false};		

bool gait_stair[6] = {false,false,false,false,false,false};	 //�Ƿ�Ϊ��̨�ײ�̬
int wait_flag[6] = {1,1,1,1,1,1};  //����ֱλ���ϵȴ���̬�任
unsigned long gait_is_change_time[6] = {0,0,0,0,0,0};  //�任��̬������еĴ���  ��Ϊ�˻�õ�һ�β�̬�任

unsigned int time_swing[6] = {0,0,0,0,0,0}; 
unsigned int time_stance[6] = {0,0,0,0,0,0};



//��̬����
//���ǲ�̬��ʼ��
int legs_triangle_stance[6] = {45,45,45,45,45,45};
int legs_triangle_swing[6] = {315,315,315,315,315,315};

//���²�̬��ʼ��
int legs_climb_stance[6] = {0,0,0,0,0,0};
int legs_climb_swing[6] = {0,0,0,0,0,0};



int legs_stair_state_1[6] = {180,45,315,180,45,315};
int legs_stair_state_2[6] = {315,180,45,315,180,45};
int legs_stair_state_3[6] = {315,315,180,315,315,180};
int legs_stair_state_4[6] = {45,45,315,45,45,315};




int legs_climb_state_1[6] = {280,280,280,280,280,280};
int legs_climb_state_2[6] = {80,80,80,80,80,80};
int legs_climb_state_3[6] = {280,80,80,280,80,80};
int legs_climb_state_4[6] = {280,280,80,280,280,80};



int legs_stairx_state_1[6] = {60, 60, 300, 60,  60, 300};
int legs_stairx_state_2[6] = {300,180,300, 300, 180,300};
int legs_stairx_state_3[6] = {60, 225,0,   60,  225,0};
int legs_stairx_state_4[6] = {180,300,0,   180, 300,0};
int legs_stairx_state_5[6] = {270,270,45,  270, 300,45};
int legs_stairx_state_6[6] = {300,315,180, 300, 315,180};
int legs_stairx_state_7[6] = {45, 45 ,270, 300, 45, 270};
int legs_stairx_state_8[6] = {300,60, 300, 300, 60, 300};




int stair_speed[3] = {0,0,0}; //��̨�ײ�̬�ٶ�ƥ��  ǰ�к�����



int legs_zero[6] = { 0,0,0,0,0,0};
//int legs_stair_swing[6] = {270,270,270,270,270,270 };
static int gait_set_last = 0;






void cpg_test(void)  //����6·��ϵ�CPG�ź�
{
	static unsigned long run_time = 0;	
	delta_time = time_ms - time_ms_last;
	time_ms_last = time_ms;	
	run_time++;
	value_xy = function_coupling(value_xy,delta_time/20000.1);
	printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f\r\n",run_time*0.114,value_xy.y[0],value_xy.y[1],value_xy.y[2],value_xy.y[3],value_xy.y[4],value_xy.y[5]);
	
}


void gait_mapping(void)
{	
	int16_t j = 0;
	static unsigned int T_2_up = 0;  //���������ڵ�һ��
	static unsigned int T_2_down = 0;//�½������ڵ�һ��
	
	
	static unsigned int T_4_up = 0;  //���������ڵ�һ��
	static unsigned int T_4_down = 0;//�½������ڵ�һ��
	
	
	
	
	
	
	
	bool gait_mode_change_location[6] = {false,false,false,false,false,false};  //��ʾ��̬�л�λ��
	for(i=0;i<6;i++)
	{
		
		j = i + 3;					
		if(i>2)
		{
				j = i - 3;											
		}	
		
		//if(gait_mode[i]==STAIR)
					//printf("stir %d\r\n",i);
		//printf("gait is %d\r\n",gait_mode[i]);
		
		position[i] = value_xy.y[i];
		position_delta[i] = position[i] - position_last[i];  //λ�ò�  ��ʼλ��position��������  ����һ��ʼ position_delta = 179 - 0 = 179
		position_last[i] = position[i];		
		if(position_delta[i]<0)   //�½���  ���ؽ׶�
		{			

				//����
				if(i == 0)   //�Ե�0������Ϊ�ο�������0���ȴ���ʱ����omega��ֵ��ͬʱ���ٶȳ����ٶȱ任�ı���ϵ��
				{
						omega_last = omega_set;
						omega_set = omega_to_set; //��ֵ��omega�µ�ֵ														
						velecity_change_pro = omega_set/omega_last;
					//		printf("T_swing%d  is %d   amplitude_down:  %.2f\r\n",i,T_swing[i],amplitude_down);													
				}	

			
					//�л�
					if((gait_mode_last[i] != gait_mode[i])||(is_change[i]))  //�л���
					{
						is_change[i] = true; //��̬��ʼת��  ʲôʱ�����ת����
						
						//printf("gait changing!\r\n");
						
						if((turn_flag[i])&&(abs(value_xy.y[i] - value_xy.y[j]) <= 0.02)) 		//һ��������ϵ����λ�ò� ��������λ�Ƿ�任����	ת�� 
						{
							leg_info_set[i].turn_dirction = direction_to_set[i];//�л������������ ����ʱ�̵��㷴������  ����ʱ�̵��㷴�򴥵� �Ϳ�ʼת������
							is_change[i] = false; //ת�����
						}
						else if(!(turn_flag[i])&&(abs(value_xy.y[i]- value_xy.y[j]) >= 0.8)) //ֱ��
						{
							leg_info_set[i].turn_dirction =  direction_to_set[i];
							is_change[i] = false;	
						}	
						else if((gait_mode[i]==STAIR)&&(abs(value_xy.y[i]- value_xy.y[j]) <= 0.02))  //�������̨�ײ�̬ �Բ���ͬ��
						{ 
							leg_info_set[i].turn_dirction =  direction_to_set[i];
							is_change[i] = false;	
						}
	
					}
					
				gait_mode_last[i] = gait_mode[i];
	
				//running_theta[i] = -30*value_xy.y[i]+30;  //��������λ��  0----60��
				
				running_theta[i] = leg_info_set[i].stance_theta;	//40			
				
				if((Real_Position_Value[i]%32000) <= 10) //�����˴�ʱ�㴦����ֱλ�ø���
				{			
						//if(i==0)
							//printf("leg %d �����л��� Ŀ��ֵ�Ѿ�����\r\n",i);	
						gait_mode_change_location[i] = true;									
				}
	
		/*
				
				if(time_stance[i]<T_4_up)
				{
					if(gait_mode[i]==STAIR)
						running_theta[i] = legs_stairx_state_1[i];      //��̨��	
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_1[i];       //���²�̬ģʽ
					else 
						running_theta[i] = leg_info_set[i].stance_theta;//���ǲ�̬ģʽ							
				}
				else if(T_4_up<time_stance[i]<T_2_up)
				{
					if(gait_mode[i]==STAIR)
						running_theta[i] = legs_stairx_state_2[i];      //��̨��
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_1[i];      //���²�̬ģʽ
					else 
						running_theta[i] = leg_info_set[i].stance_theta;//���ǲ�̬ģʽ								 									
				
				
				}
				else if(T_2_up<time_stance[i]<T_2_up+T_4_up)
				{
					if(gait_mode[i]==STAIR)
						running_theta[i] = legs_stairx_state_3[i];      //��̨��	
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_2[i];      //���²�̬ģʽ
					else
						running_theta[i] = leg_info_set[i].stance_theta;//���ǲ�̬ģʽ						
				}
				else if(T_2_up+T_4_up<time_stance[i]<T_swing[i])
				{
					if(gait_mode[i]==STAIR)
						running_theta[i] = legs_stairx_state_4[i];      //��̨��	
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_2[i];      //���²�̬ģʽ
					else
						running_theta[i] = leg_info_set[i].stance_theta;//���ǲ�̬ģʽ											
				}
				
	*/
				
				if(time_stance[i] < T_2_up)
				{
					
					if((gait_mode[i]==STAIR)&&(time_stance[i]<T_4_up))
						//running_theta[i] = legs_stair_state_1[i];      //��̫����̬ģʽ
						running_theta[i] = legs_stairx_state_1[i];      //��̨��	
					else if((gait_mode[i]==STAIR)&&(T_4_up<time_stance[i]<T_2_up))
						running_theta[i] = legs_stairx_state_2[i];      //��̨��	
					
					
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_1[i];      //���²�̬ģʽ
					else 
						running_theta[i] = leg_info_set[i].stance_theta;//���ǲ�̬ģʽ								 									
				
				}
				else if(T_2_up<time_swing[i]<T_swing[i])
				{
					
					if((gait_mode[i]==STAIR)&&(T_2_up<time_stance[i]<T_2_up+T_4_up))
						running_theta[i] = legs_stairx_state_3[i];
					else if((gait_mode[i]==STAIR)&&(T_2_up+T_4_up<time_stance[i]<T_swing[i]))
						running_theta[i] = legs_stairx_state_4[i];      //��̨��	
					
					
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_2[i];      //���²�̬ģʽ
					else
						running_theta[i] = leg_info_set[i].stance_theta;//���ǲ�̬ģʽ								 									
				}

				
				
				if(state_swing[i])  //�ɼ�swing��ʱ��
				{			
					T_swing[i] = time_swing[i];//����½��ص�ʱ������
					T_2_down = T_swing[i]/2;		//	����½���ʱ���һ��
					T_4_down = T_swing[i]/4;
				}
						
				if(T_stance[i]>10)
				{
					//speed_stance = 60/T_stance[i];  //���ص��ٶ�
					speed_stance = 95*running_theta[i]/T_stance[i];
					speed_stance  = speed_stance*velecity_change_pro;
				
				}
					
		
				state_swing[i] = false;
				state_stance[i] = true;										
				time_swing[i] = 0;  //�½�ʱ����0 ����ʱ������				
				time_stance[i]++;				
				
			
		}
		else if(position_delta[i]>0)  //������  ���ս׶�
		{
		
				//running_theta[i] = 150*value_xy.y[i]+210; //��������λ��  60----360��
				
				running_theta[i] = leg_info_set[i].swing_theta;//										 							
			

				
				/*
				if(time_swing[i]<T_4_down)
				{
					if(gait_mode[i]==STAIR)
							running_theta[i] = legs_stairx_state_5[i];     //��̫����̬ģʽ
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_3[i];        //���²�̬ģʽ
					else  
						running_theta[i] = leg_info_set[i].swing_theta;  //���ǲ�̬ģʽ									
				}
				else if(T_4_down<time_swing[i]<T_2_down)
				{
					if(gait_mode[i]==STAIR)
							running_theta[i] = legs_stairx_state_6[i];      //��̫����̬ģʽ
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_3[i];         //���²�̬ģʽ
					else 
						running_theta[i] = leg_info_set[i].swing_theta;   //���ǲ�̬ģʽ					
				}
				else if(T_2_down<time_swing[i]<T_2_down+T_4_down)
				{
					if(gait_mode[i]==STAIR)
						running_theta[i] = legs_stairx_state_7[i];      //��̫����̬ģʽ	
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_4[i];      //���²�̬ģʽ
					else
						running_theta[i] = leg_info_set[i].swing_theta;//���ǲ�̬ģʽ						
				}else if(T_2_down+T_4_down<time_swing[i]<T_swing[i])
				{
					if(gait_mode[i]==STAIR)
							running_theta[i] = legs_stairx_state_8[i];     //��̫����̬ģʽ
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_4[i];        //���²�̬ģʽ
					else
						running_theta[i] = leg_info_set[i].swing_theta;  //���ǲ�̬ģʽ						
				}
							
				*/
				if(time_swing[i]<T_2_down)  //�½���ǰ�������
				{
					if((gait_mode[i]==STAIR)&&(time_swing[i]<T_4_down))
						running_theta[i] = legs_stairx_state_5[i];      //��̫����̬ģʽ
					else if((gait_mode[i]==STAIR)&&(T_4_down<time_swing[i]<T_2_down))
						running_theta[i] = legs_stairx_state_6[i];      //��̫����̬ģʽ
					
					
					
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_3[i];      //���²�̬ģʽ
					else 
						running_theta[i] = leg_info_set[i].swing_theta;//���ǲ�̬ģʽ								 									
				
				}
				else if(T_2_down<time_swing[i]<T_swing[i]) //�½��غ�������
				{
					if((gait_mode[i]==STAIR)&&(T_2_down<time_swing[i]<T_2_down+T_4_down))
						running_theta[i] = legs_stairx_state_7[i];      //��̫����̬ģʽ
					if((gait_mode[i]==STAIR)&&(T_2_down+T_4_down<time_swing[i]<T_swing[i]))
						running_theta[i] = legs_stairx_state_8[i];      //��̫����̬ģʽ
					
					
					
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_4[i];      //���²�̬ģʽ
					else
						running_theta[i] = leg_info_set[i].swing_theta;//���ǲ�̬ģʽ								 									
				}
				


				if(state_stance[i])  //�ɼ�stance��ʱ��
				{
					T_stance[i] = time_stance[i];	
					T_2_up = T_stance[i]/2;  //��������ص����ڵ�һ��	
					T_4_up = T_stance[i]/4;
				}			
			
				if(T_swing[i]>10)
				{
					//speed_swing = 300/T_swing[i];  //���յ��ٶ�
					speed_swing  = 70*running_theta[i]/T_swing[i]*1.0;  //360
					speed_swing  = speed_swing*velecity_change_pro;										
				}
								
				//if(i==0)
					//printf("T_swing is %d  speed swing is  %d\r\n",T_swing[i],speed_swing);
				
			
				state_swing[i] = true;
				state_stance[i] = false;	
				time_stance[i] = 0;	//�½�ʱ����0 ����ʱ������
				time_swing[i]++;		
		}
		
		
		
		theta_change[i]  = running_theta[i] - running_theta_last[i];  //ɲͣ
		running_theta_last[i] = running_theta[i];  //���λ�õ�����
	

	

		if(leg_info_set[i].turn_dirction == FORWARD) //��תʱ
		{
			
			
			//printf("forward state\r\n");	
		
			if(theta_change[i] < 0)          //���봥�ؽ׶�
			{							
				theta_change[i] += THETA_CIRCLE; //�����ֵΪ���� �ͽ����Ϊ���ĽǶ�	
				speed[i] = speed_stance;	
				
				//if((i == 0 )||(i == 3))
						//printf("forward state speed[%d] speed_stance is %d theta_change[%d]:%lu\r\n",j,speed_swing,j,theta_change[j]);								
				
			}					
			else if(theta_change[i] > 0)		 //�������ս׶�
			{									
				
					theta_change[i] = theta_change[i];						
					speed[i] = speed_swing;								
					
				//if((i == 0 )||(i == 3))
						//printf("forward state speed[%d] speed_swing is %d theta_change[%d]:%lu\r\n",j,speed_swing,j,theta_change[j]);															
			}		
		
		}
																
		else if(leg_info_set[i].turn_dirction == REVERSE)  //���ת  
		{		
			//printf("reverse state\r\n");	
			
			if(theta_change[i] > 0)          //���ؽ׶�  �˴����ܵ���0 
			{								
					theta_change[i] -= THETA_CIRCLE; 							
					speed[i] = speed_stance;
																
					
				//	if((i == 0 )||(i == 3))
					//		printf("reverse state speed[%d] speed_stance is %d theta_change[%d]:%lu\r\n",i,speed_stance,i,theta_change[i]);															
			}					
			else if(theta_change[i] < 0)   	//���ս׶�
			{																
					theta_change[i] = theta_change[i];														
					speed[i] = speed_swing;																																	
					
																	
			}		
		}					
		
		
		if((gait_mode[i]==STAIR)||gait_mode[i]==CLIMB)
			speed[i] = 500;  //̨�ײ�̬���ǿ�
		
		desired_values.global_theta[i] = theta_change[i];
		desired_values.global_velocity[i] = speed[i];					
	}	
}


void gait_choose(int gait_to_set)
{
	gait_set_last = gait_to_set;
	
	
	if(gait_to_set == STAIR)
	{
			
		
			theta[0][3] = 0;
			theta[3][0] = 0; //�ı�leg0��leg3��������										
				
			theta[1][4] = 0;
			theta[4][1] = 0; //�ı�leg0��leg3��������		
				
			theta[2][5] = 0;
			theta[5][2] = 0; //�ı�leg0��leg3��������		
			
			theta[0][2] = 0.7*pi;
			theta[2][0] = -0.7*pi;
			
				
			//ͬ������λ��ϵ
			theta[0][1] = 0.7*pi;
			theta[1][0] = -0.7*pi;
			theta[0][0] = 0; //�ı�leg0��leg3��������			
			
			theta[1][2] = 0.7*pi;
			theta[2][1] = -0.7*pi;
			theta[1][1] = 0; //�ı�leg0��leg3��������			
			
			theta[3][4] = 0.7*pi;
			theta[4][3] = -0.7*pi;
			
			theta[3][5] = 0.7*pi;
			theta[5][3] = -0.7*pi;             
										
										
			theta[4][5] = 0.7*pi;
			theta[5][4] = -0.7*pi;
			
			
			theta[0][4] = 0.7*pi;    
			theta[4][0] = -0.7*pi;   
			theta[0][5] = 0.7*pi;    
			theta[5][0] = -0.7*pi;  				
			
			
			theta[1][3] = -0.7*pi;    
			theta[3][1] = 0.7*pi;   
			theta[1][5] = 0.7*pi;    
			theta[5][1] = -0.7*pi;  				
			
			
			theta[2][3] = -0.7*pi;    
			theta[3][2] = 0.7*pi;   
			theta[2][4] = 0.7*pi;    
			theta[4][2] = -0.7*pi;  	
		
		
			for(i =0; i<6;i++)
			{	
				gait_mode[i] = gait_to_set;
			
				direction_to_set[i] = direction_forward[i];
				turn_flag[i] = false;		
				gait_stair[i] = true;	
			
			}
	
	}
	
	if(gait_to_set == CLIMB)
	{
		
		for(i =0; i<6;i++)
				{	
					gait_mode[i] = gait_to_set;
					theta[0][3] = 0;
					theta[3][0] = 0; //�ı�leg0��leg3��������										
						
					theta[1][4] = 0;
					theta[4][1] = 0; //�ı�leg0��leg3��������		
						
					theta[2][5] = 0;
					theta[5][2] = 0; //�ı�leg0��leg3��������		
					
					theta[0][2] = 0;
					theta[1][2] = 0;
					
								
					//ͬ������λ��ϵ
					theta[0][1] = 0;
					theta[1][0] = 0;
					theta[0][0] = 0; //�ı�leg0��leg3��������			
					
					theta[1][2] = 0;
					theta[2][1] = 0;
					theta[1][1] = 0; //�ı�leg0��leg3��������			
					
					theta[3][4] = 0;
					theta[4][3] = 0;
					
					theta[4][5] = 0;
					theta[5][4] = 0;
							
					
					
					leg_info_set[i].stance_theta = legs_climb_stance[i];
					leg_info_set[i].swing_theta = legs_climb_swing[i];
					
					direction_to_set[i] = direction_forward[i];
					turn_flag[i] = false;		
					gait_stair[i] = false;				
				}
	}
	
	else  //���ǲ�̬
	{
			//�����������ӵ�
			theta[0][2] = 0;
			theta[0][4] = 0;
			theta[0][5] = 0;
			theta[1][3] = 0;
			theta[1][5] = 0;
			theta[2][0] = 0;
			theta[2][2] = 0;
			theta[2][3] = 0;
			theta[2][4] = 0;
			theta[3][1] = 0;
			theta[3][2] = 0;
			theta[3][3] = 0;
			theta[3][5] = 0;
			theta[4][0] = 0;
			theta[4][2] = 0;
			theta[4][4] = 0;
			theta[5][0] = 0;
			theta[5][1] = 0;
			theta[5][3] = 0;
			theta[5][5] = 0;
	
			
			for(i =0; i<6;i++)
			{			
				leg_info_set[i].stance_theta = legs_triangle_stance[i];
				leg_info_set[i].swing_theta = legs_triangle_swing[i];			
				gait_mode[i] = gait_to_set;
				gait_stair[i] = false;		
				
				if(gait_to_set == LEFT)
				{
					
					theta[0][3] = 0;
					theta[3][0] = 0; //�ı�leg0��leg3��������		
	
					theta[1][4] = 0;
					theta[4][1] = 0; //�ı�leg1��leg4��������
						
					theta[2][5] = 0;
					theta[5][2] = 0; //�ı�leg2��leg5��������									
					

						
					//ͬ������λ��ϵ
					theta[0][1] = pi;
					theta[1][0] = -pi;
					theta[0][0] = 0; //�ı�leg0��leg3��������			
					
					theta[1][2] = pi;
					theta[2][1] = -pi;
					theta[1][1] = 0; //�ı�leg0��leg3��������			
					
					theta[3][4] = pi;
					theta[4][3] = -pi;
					
					theta[4][5] = pi;
					theta[5][4] = -pi;
					
					
					//20190604����
					theta[0][4] = pi;
					theta[4][0] = -pi;
					theta[1][3] = pi;
					theta[3][1] = -pi;
					theta[3][5] = pi;
					theta[5][3] = -pi;
					theta[2][4] = pi;
					theta[4][2] = -pi;
																								


					direction_to_set[i] = turn_left[i];
					turn_flag[i] = true;
					gait_stair[i] = false;							
				}
				else if(gait_to_set == RIGHT)
				{
					//ת��ʱ���轫��Ǳ�Ϊ��ͬ
						
					theta[0][3] = 0;
					theta[3][0] = 0; //�ı�leg0��leg3��������		
	
					theta[1][4] = 0;
					theta[4][1] = 0; //�ı�leg1��leg4��������		
					
					theta[2][5] = 0;
					theta[5][2] = 0; //�ı�leg2��leg5��������		
					
					
						
					
					//ͬ������λ��ϵ
					theta[0][1] = pi;
					theta[1][0] = -pi;
					theta[0][0] = 0; //�ı�leg0��leg3��������			
					
					theta[1][2] = pi;
					theta[2][1] = -pi;
					theta[1][1] = 0; //�ı�leg0��leg3��������			
					
					theta[3][4] = pi;
					theta[4][3] = -pi;
					
					theta[4][5] = pi;
					theta[5][4] = -pi;
					
					//20190604����															
					theta[0][4] = pi;
					theta[4][0] = -pi;
					theta[1][3] = pi;
					theta[3][1] = -pi;
					theta[3][5] = pi;
					theta[5][3] = -pi;
					theta[2][4] = pi;
					theta[4][2] = -pi;
					
					direction_to_set[i] = turn_right[i];
					turn_flag[i] = true;
					gait_stair[i] = false;			
				}
				else if(gait_to_set == TRIANGLE_FORWARD)
				{			
					
					theta[0][3] = pi;
					theta[3][0] = -pi; //�ı�leg0��leg3��������					
					
					
					theta[1][4] = pi;
					theta[4][1] = -pi; //�ı�leg0��leg3��������		
					
					theta[2][5] = pi;
					theta[5][2] = -pi; //�ı�leg0��leg3��������		
					
					
					
					//ͬ������λ��ϵ
					theta[0][1] = pi;
					theta[1][0] = -pi;
					theta[0][0] = 0; //�ı�leg0��leg3��������			
					
					theta[1][2] = pi;
					theta[2][1] = -pi;
					theta[1][1] = 0; //�ı�leg0��leg3��������			
					
					theta[3][4] = pi;
					theta[4][3] = -pi;
					
					theta[4][5] = pi;
					theta[5][4] = -pi;
																										
					direction_to_set[i] = direction_forward[i];
					
					turn_flag[i] = false;
					gait_stair[i] = false;			
				}
				else if(gait_to_set == TRIANGLE_BACKWARD)
				{
									
					theta[0][3] = pi;
					theta[3][0] = -pi; //�ı�leg0��leg3��������										
					
					theta[1][4] = pi;
					theta[4][1] = -pi; //�ı�leg0��leg3��������		
					
					theta[2][5] = pi;
					theta[5][2] = -pi; //�ı�leg0��leg3��������		
						
					
					
					
					//ͬ������λ��ϵ
					theta[0][1] = pi;
					theta[1][0] = -pi;
					theta[0][0] = 0; //�ı�leg0��leg3��������			
					
					theta[1][2] = pi;
					theta[2][1] = -pi;
					theta[1][1] = 0; //�ı�leg0��leg3��������			
					
					theta[3][4] = pi;
					theta[4][3] = -pi;
					
					theta[4][5] = pi;
					theta[5][4] = -pi;
	
					
					direction_to_set[i] = direction_reverse[i];
					turn_flag[i] = false;
					gait_stair[i] = false;								
				}						
			}
		}
}

float cpg_to_angle(float cpg_position)
{
	int16_t position = 0;
	
	position = 360/2 * cpg_position;
	return position;
}



void get_cmd_from_pc()
{			
		u8 len;	
		int i = 0;
		int j = 0;
		char speedstr[6] = {0}; 				
		//float omega_to_set = omega_set;//��Ҫ��beta��ֵ
		if(USART_RX_STA&0x8000)
		{			
				len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
				//printf("\r\n�����͵���ϢΪ:\r\n");	
				//HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART_RX_BUF,len,1000);	//���ͽ��յ�������
				//while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//�ȴ����ͽ���
				//printf("\r\n\r\n");//���뻻��
				USART_RX_STA=0;	
				
				if(!strncmp((char*)USART_RX_BUF,"betaV",5))  //���ǲ�̬
				{																					
						strncpy(speedstr,(char*)USART_RX_BUF+5,1); //���ٶ����ݸ�ֵ��speedstr															
						beta_set = atoi(speedstr)/10.0;								//betaV6
					//	printf("bate is %.2f\r\n",beta_set);					
				}					
		
				if(!strncmp((char*)USART_RX_BUF,"omgaV",5))  //�ƶ�
				{											
						strncpy(speedstr,(char*)USART_RX_BUF+5,1); //���ٶ����ݸ�ֵ��speedstr
						omega_to_set = atoi(speedstr);						 		 //betaV6					
					//	printf("omega to set is %.2f\r\n",omega_to_set);
				}	
				
				
				if(!strncmp((char*)USART_RX_BUF,"k",1))  //�ƶ�
				{											
						strncpy(speedstr,(char*)USART_RX_BUF+1,1); //������Ͼ���ı任����
						
											
	
					//test 20190521
					
					/*
					k_value[0][3]=k_value[3][0] 
					 =k_value[0][1]=k_value[1][0] 
					 =k_value[1][2]=k_value[2][1]
 					 =k_value[1][4]=k_value[4][1]																		
					 =k_value[2][5]=k_value[5][2]
					
					 =atoi(speedstr)*3.8;				
					*/
					for(i=0;i<6;i++)
					{
						for(j=0;j<6;j++)
						{
							k_value[i][j] = atoi(speedstr)*2.8;		
						}
					
					}
					
						//����k7�ƺ�ͦ���ʵ� 20190425
						//k_value[0][0]	=	k_value[1][1] = k_value[2][2] 
					 
					 //=k_value[5][5] = k_value[4][4] = k_value[3][3]  
					 // = atoi(speedstr)*1.1;	
					
				}	
	
				if(!strncmp((char*)USART_RX_BUF,"left",4))  //��ת
				{																															
					
					gait_choose(LEFT);	
					
					/*
					for(i=0;i<6;i++)
						{
							gait_mode[i] = LEFT;
							direction_to_set[i] = turn_left[i];
							turn_flag[i] = true;								
						}	
					
						theta[0][3] = 0;
						theta[3][0] = 0; //�ı�leg0��leg3��������		

						theta[1][4] = 0;
						theta[4][1] = 0; //�ı�leg1��leg4��������
					
						theta[2][5] = 0;
						theta[5][2] = 0; //�ı�leg2��leg5��������
					*/
		
						//printf("turn left \r\n");
				}		
				if(!strncmp((char*)USART_RX_BUF,"rigt",4))  //��ת
				{																														
						
					
					gait_choose(RIGHT);	
						//ת��ʱ���轫��Ǳ�Ϊ��ͬ
						/*
						theta[0][3] = 0;
						theta[3][0] = 0; //�ı�leg0��leg3��������		

						theta[1][4] = 0;
						theta[4][1] = 0; //�ı�leg1��leg4��������		
					
						theta[2][5] = 0;
						theta[5][2] = 0; //�ı�leg2��leg5��������		
							

					
					
						for(i=0;i<6;i++)
							{
								gait_mode[i] = RIGHT;
								direction_to_set[i] = turn_right[i];
								turn_flag[i] = true;					
							}		
					*/
						//printf("turn right \r\n");
				}	
				if(!strncmp((char*)USART_RX_BUF,"fwrd",4))  //ǰ��
				{																							
					/*	
					theta[0][3] = pi;
					theta[3][0] = pi; //�ı�leg0��leg3��������					
					
					
					theta[1][4] = pi;
					theta[4][1] = pi; //�ı�leg0��leg3��������		
					
					theta[2][5] = pi;
					theta[5][2] = pi; //�ı�leg0��leg3��������		
					
			
					//��ϵ��ֻ���趨һ�� k_value[i][j] ��ʾ��i���� �� ��j�������ʱ �任���ٶ�
					
					k_value[0][3] = 9.8;
					k_value[3][0] = 9.8;						
					k_value[0][0] = 1.8;
					k_value[3][3] = 1.8;
					
					
					k_value[0][1] = 6.8;
					k_value[1][0] = 6.8;						
					k_value[1][1] = 1.8;
					
					
					k_value[1][2] = 6.8;
					k_value[2][1] = 6.8;						
					k_value[2][2] = 1.8;
					
					
					k_value[1][4] = 6.8;
					k_value[4][1] = 6.8;						
					k_value[4][4] = 1.8;
					
					k_value[2][5] = 6.8;
					k_value[5][2] = 6.8;						
					k_value[5][5] = 1.8;
					*/
										
					
					
					gait_choose(TRIANGLE_FORWARD);	
					/*
					for(i=0;i<6;i++)
						{
							gait_mode[i] = TRIANGLE_FORWARD;
							direction_to_set[i] = direction_forward[i];
							turn_flag[i] = false;								
						}			
					*/
						//printf("forward \r\n");
				}	

				if(!strncmp((char*)USART_RX_BUF,"back",4))  //����
				{																																
					gait_choose(TRIANGLE_BACKWARD);		
				}		
												
				if(!strncmp((char*)USART_RX_BUF,"stir",4))  //����
				{	
					gait_choose(STAIR);
				}
				if(!strncmp((char*)USART_RX_BUF,"clim",4))  //����
				{	
					gait_choose(CLIMB);
				}
	

				if(!strncmp((char*)USART_RX_BUF,"stop",4))
					HAL_TIM_Base_Stop_IT(&TIM3_Handler); //ֹͣ��ʱ���жϣ�TIM_IT_UPDATE   
				if(!strncmp((char*)USART_RX_BUF,"start",5))
					HAL_TIM_Base_Start_IT(&TIM3_Handler); //��ʼ��ʱ������
			}
}


