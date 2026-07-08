#ifndef __CLOSE_LOOP_HPP__
#define __CLOSE_LOOP_HPP__

#ifdef __cplusplus
extern "C" {
#endif

#include "zf_common_headfile.hpp"


struct PID
{
	float Kp;
	float Ki;
	float Kd;
	float Kp_gyro;
	float Ki_gyro;
	float Kd_gyro;
};

//速度环

extern float err_speed;				
extern float out_L,out_R;
extern float correct_L;
extern float expect_gyro;
extern float gyro_out;
extern float speed_target;
extern bool  angle_ctrl_enable ;
extern float target_angle;
extern float Out_angle;

extern struct PID pid_left_speed;
extern struct PID pid_right_speed;
//2-循迹
extern struct PID pid_motor_run;

extern struct PID pid_gyro;

extern struct PID pid_angle;

void loop_speed(void);
void direction_return(float err_position);
void gyro_loop(void);
void loop_speed_LR(int16 speed_L_t,int16 speed_R_t) ;
void Gyro_loop_out(float expect);
void direction_loop(float err_position);
void angle_loop(void);
void set_relative_angle(float delta_deg);

#ifdef __cplusplus
}
#endif

#endif