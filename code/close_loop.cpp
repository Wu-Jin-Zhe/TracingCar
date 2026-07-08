#include "tuoluo.hpp"
#include "motor.hpp"
#include "close_loop.hpp"
#include "encoder.hpp"

//速度环
float err_speed = 0;
float out_L = 0,out_R = 0;
float err_sum = 0;

//方向环
float expect_gyro = 0;
float correct_L = 0;
float Out_angle =0 ;
float gyro_out = 0;
float speed_target = 100;
float target_angle = 0;      // 目标绝对角度
bool  angle_ctrl_enable = false;

/* 电压11.2～11.4*/

//1-速度环
struct PID pid_left_speed = {3,9,0,0,0,0};
struct PID pid_right_speed = {3,9,0,0,0,0};

//2-循迹
struct PID pid_motor_run = {3,0,0,0.2,0,1};

struct PID pid_gyro = {0.18,0,0,0,0,0};

struct PID pid_angle = {6,0,0,0,0,0};

//角速度环
int out_gyro = 0;

float d1 = 0.2;

//摄像头采集位置->方向环->电机差速
void direction_return(float err_position)
{
	static float direction_err1[4]={0};
        
	//1-摄像头采集位置->误差环->角速度期望
	direction_err1[1] = direction_err1[0];
	direction_err1[0] = err_position;
	err_sum += err_position;

	//P D
	expect_gyro =  pid_motor_run.Kp *  direction_err1[0]	
		         + pid_motor_run.Kd * (direction_err1[0] - direction_err1[1]);    

	if(expect_gyro >= 1500){expect_gyro = 1500;}
	if(expect_gyro <= -1500){expect_gyro = -1500;}
        
}

void direction_loop(float err_position)
{
	static float direction_err1[4]={0};
	static float err_sum = 0;
        
	//1-摄像头采集位置->误差环->角速度期望
	direction_err1[1] = direction_err1[0];
	direction_err1[0] = err_position;

	//P D
	correct_L =  pid_motor_run.Kp *  direction_err1[0]	
		         + pid_motor_run.Kd * (direction_err1[0] - direction_err1[1])
				 - d1 * avl_gyro_z;    

	if(correct_L >= 200){correct_L = 200;}
	if(correct_L <= -200){correct_L = -200;}
        
}

//角速度环
void gyro_loop(void)
{
    static float  err_gyro;//dec=0;
    static float  direction_err2[2]={0};

	err_gyro = expect_gyro - avl_gyro_z;
	direction_err2[1] = direction_err2[0];
	direction_err2[0] = err_gyro;
	
	correct_L=  pid_motor_run.Kp_gyro *  direction_err2[0]
		      + pid_motor_run.Kd_gyro * (direction_err2[0] - direction_err2[1]); 	

	if(correct_L >= 200) {correct_L = 200;}
	if(correct_L <= -200){correct_L = -200;}
   
}

void loop_speed_LR(int16 speed_L_t,int16 speed_R_t) 
{
	static int16 err_speed_L_last = 0,err_speed_L = 0;
	static float dec_speed_loop_L = 0;
	static int16 err_speed_R_last = 0,err_speed_R = 0;
	static float dec_speed_loop_R = 0;
	
	
	err_speed_L_last = err_speed_L;
	err_speed_L = speed_L_t - speed_L;
	
	
	dec_speed_loop_L =  pid_left_speed.Kp * (err_speed_L - err_speed_L_last)
									  + pid_left_speed.Ki *  err_speed_L;

	
	if(dec_speed_loop_L > 1000 ){dec_speed_loop_L = 1000; }
	if(dec_speed_loop_L < -1000){dec_speed_loop_L = -1000;}
	
	
	out_L += dec_speed_loop_L;

	
	err_speed_R_last = err_speed_R;
	err_speed_R = speed_R_t - speed_R;
	
	
	dec_speed_loop_R =  pid_right_speed.Kp * (err_speed_R - err_speed_R_last)
									  + pid_right_speed.Ki *  err_speed_R;

	
	if(dec_speed_loop_R > 1000 ){dec_speed_loop_R = 1000; }
	if(dec_speed_loop_R < -1000){dec_speed_loop_R = -1000;}
	
	
	out_R += dec_speed_loop_R;
	
    if(out_L > 8000 ){out_L = 8000; }
	if(out_L < -8000){out_L = -8000;}
	if(out_R > 8000 ){out_R = 8000; }
	if(out_R < -8000){out_R = -8000;}
}


// 调用一次 -> 从当前位置转 delta_deg 度
void set_relative_angle(float delta_deg)
{
    target_angle = angle + delta_deg;   // 当前角度 + 增量，不清零
    angle_ctrl_enable = true;
}

void angle_loop(void)
{
	static float err_angle = 0;
    static float  direction_err[2] = {0};

	err_angle = target_angle - angle;
	direction_err[1] = direction_err[0];
	direction_err[0] = err_angle;

	Out_angle = pid_angle.Kp * direction_err[0] 
	            + pid_angle.Kd * (direction_err[0] - direction_err[1]);

	if(Out_angle >= 9000){Out_angle = 9000;}
	if(Out_angle <= -9000){Out_angle = -9000;}

}