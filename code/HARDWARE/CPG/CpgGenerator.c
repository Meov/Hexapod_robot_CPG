#include "CpgGenerator.h"
#include "timer.h"
#include "math.h"
/******************
2019��4��19��  ��ֱλ�õ�������
2019��4��19��17:04 ����ת�򡢵���  ���ڽǶ�ƥ������  ����ʱ�ᷢ�����  �����ǲ������ǲ�̬ʱ ���ɸ��Ƕ�ʱ����ƫ�� ��û�����ô��� ==
*********************/




float omega_set = 2;
float beta_set = 0.5;

float omega_to_set = 0;//��Ҫ����omega��ֵ
float beta_to_set = 0;//��Ҫ��beta��ֵ



//��Ͼ�����ǹ�ϵ
float theta[6][6] = {
											0,0,0,0,0,0,
											0,0,0,0,0,0,
											0,0,0,0,0,0,
											0,0,0,0,0,0,
											0,0,0,0,0,0,
										};

float k_value[6][6] = 
										{
											0,0,0,0,0,0,
											0,0,0,0,0,0,
											0,0,0,0,0,0,
											0,0,0,0,0,0,
											0,0,0,0,0,0,
											0,0,0,0,0,0,				
										};
									

/*

			|			|
leg0 =|			|=  leg3
			|			|
			|			|
leg1 =|			|=  leg4
			|			|
			|			|
leg2 =|			|=  leg5
			|     |

*/






function function_return(float x,float y)
{
		//return (2 *t*t*t*t*exp(t)); //???? y = 2x
		function function_xy;
		float w_swing = 30 * pi, w_sweep =   2*pi;
		
		
		
		float beta = 0.7;
		float u = 1;
		float w = 0;
		int b = 10;
		float rho = 100;
		float sigma = 100;
		
		w_swing = omega_set*3*pi;
		beta = beta_set;
		//printf("omega_set is %.2f\r\n",omega_set);
		w = (((1 - beta) / beta)*w_swing) / (exp(-b*y) + 1) + w_swing / (exp(b*y) + 1);
		function_xy.x = rho*(u - x*x - y*y)*x - w*y;
		function_xy.y = sigma*(u - x*x - y*y)*y + w*x;	
		return function_xy;
}


int Eular(float a, float b, float h,function xy_value,float *result_x,float *result_y)
{
	
		static int k = 0;
		if (a > b)
		{
			return -1; 
		}
	
			*result_x  = xy_value.x + function_return(xy_value.x, xy_value.y).x *h;
			*result_y  = xy_value.y + function_return(xy_value.x, xy_value.y).y *h;								
						
			k++;
			//printf("%d   %f   %f\r\n",k,*result_x,*result_y);
		
		return 0;
}

/******************
2019��4��23��  ΢�ַ��� �����Ͼ���
2019��4��19��17:04 ����ת�򡢵���  ���ڽǶ�ƥ������  ����ʱ�ᷢ�����  �����ǲ������ǲ�̬ʱ ���ɸ��Ƕ�ʱ����ƫ�� ��û�����ô��� ==
										����취  ʹ����Ͼ��� �ı��������λ�� �Ӷ��ﵽ�ı䲽̬��Ч��
*********************/
function_parameters hopf_function_return(function_parameters xy_value,function_parameters xy_coupling_value,float h,int i,int j)
{
		function_parameters paramters_out;  //�����Ĳ��� ��ֵ��������
		float w_swing = 0;

		float beta = 0.5;
		float u = 1;
		float omega[6] = {0};
		int b = 10;
		float rho = 100;
		float sigma = 100;
	
		w_swing = omega_set*3*pi;
		beta = beta_set;
		
		omega[0] = (((1 - beta) / beta)*w_swing) / (exp(-b*xy_value.y) + 1) + w_swing / (exp(b*xy_value.y) + 1);

		
		paramters_out.x = rho*(u - xy_value.x*xy_value.x - xy_value.y*xy_value.y)*xy_value.x - omega[0]*xy_value.y
												+ k_value[i][i]*(cos(theta[i][i])*xy_value.x - sin(theta[i][i])*xy_value.y)
												+ k_value[i][j]*(cos(theta[i][j])*xy_coupling_value.x - sin(theta[i][j])*xy_coupling_value.y); //x�������xy_coupling_value.x  y�������xy_coupling_value.y
											

		paramters_out.y = sigma*(u - xy_value.x*xy_value.x - xy_value.y*xy_value.y)*xy_value.y + omega[0]*xy_value.x
												+	k_value[i][i]*(sin(theta[i][i])*xy_value.x + cos(theta[i][i])*xy_value.y)
												+	k_value[i][j]*(sin(theta[i][j])*xy_coupling_value.x + cos(theta[i][j])*xy_coupling_value.y);  //x�������x_coupling  y�������y_coupling
						
	
		
		paramters_out.x = xy_value.x + paramters_out.x*h;   // �൱��f(x)+ dx*step
		paramters_out.y = xy_value.y + paramters_out.y*h;
					
		return paramters_out;
}

/*************
2019��5��21�� �������ȫ�������

**********/

cpg_xy function_coupling(cpg_xy values,float step)
{
		float dx[6] = {0,0,0,0,0,0};
		float dy[6] = {0,0,0,0,0,0};
		float w_swing = 0;
		float beta = 0.5;
		float u = 1;
		float omega[6] = {0};
		int b = 10;
		float rho = 100;
		float sigma = 100;
		int i = 0;
		w_swing = omega_set*3*pi;
		beta = beta_set;
		
		omega[0] = (((1 - beta) / beta)*w_swing) / (exp(-b*values.y[0]) + 1) + w_swing / (exp(b*values.y[0]) + 1);
		omega[1] = (((1 - beta) / beta)*w_swing) / (exp(-b*values.y[1]) + 1) + w_swing / (exp(b*values.y[1]) + 1);
		omega[2] = (((1 - beta) / beta)*w_swing) / (exp(-b*values.y[2]) + 1) + w_swing / (exp(b*values.y[2]) + 1);
		omega[3] = (((1 - beta) / beta)*w_swing) / (exp(-b*values.y[3]) + 1) + w_swing / (exp(b*values.y[3]) + 1);
		omega[4] = (((1 - beta) / beta)*w_swing) / (exp(-b*values.y[4]) + 1) + w_swing / (exp(b*values.y[4]) + 1);
		omega[5] = (((1 - beta) / beta)*w_swing) / (exp(-b*values.y[5]) + 1) + w_swing / (exp(b*values.y[5]) + 1);
		
		for(i=0;i<6;i++)
		{
		
			dx[i] = rho*(u - values.x[i]*values.x[i] - values.y[i]*values.y[i])*values.x[i] - omega[i]*values.y[i] //��������
												+ k_value[i][0]*(cos(theta[i][0])*values.x[0] - sin(theta[i][0])*values.y[0])
												+ k_value[i][1]*(cos(theta[i][1])*values.x[1] - sin(theta[i][1])*values.y[1])
												+ k_value[i][2]*(cos(theta[i][2])*values.x[2] - sin(theta[i][2])*values.y[2])
												+ k_value[i][3]*(cos(theta[i][3])*values.x[3] - sin(theta[i][3])*values.y[3])
												+ k_value[i][4]*(cos(theta[i][4])*values.x[4] - sin(theta[i][4])*values.y[4])
												+ k_value[i][5]*(cos(theta[i][5])*values.x[5] - sin(theta[i][5])*values.y[5]);
			
			dy[i] = sigma*(u - values.x[i]*values.x[i] - values.y[i]*values.y[i])*values.y[i] + omega[i]*values.x[i]//��������
												+	k_value[i][0]*(sin(theta[i][0])*values.x[0] + cos(theta[i][0])*values.y[0])
												+	k_value[i][1]*(sin(theta[i][1])*values.x[1] + cos(theta[i][1])*values.y[1])
												+	k_value[i][2]*(sin(theta[i][2])*values.x[2] + cos(theta[i][2])*values.y[2])
												+	k_value[i][3]*(sin(theta[i][3])*values.x[3] + cos(theta[i][3])*values.y[3])
												+	k_value[i][4]*(sin(theta[i][4])*values.x[4] + cos(theta[i][4])*values.y[4])
												+	k_value[i][5]*(sin(theta[i][5])*values.x[5] + cos(theta[i][5])*values.y[5]); 
		
			values.x[i] = values.x[i] + dx[i] * step;
			values.y[i] = values.y[i] + dy[i] * step;
		}
		
		return values;
}
