#ifndef __MOTOR_HPP__
#define __MOTOR_HPP__

#include "zf_common_headfile.hpp"

#define PWM_1_PATH ZF_PWM_MOTOR_1
#define DIR_1_PATH ZF_GPIO_MOTOR_1
#define PWM_2_PATH ZF_PWM_MOTOR_2
#define DIR_2_PATH ZF_GPIO_MOTOR_2

#define PWM_MAX 10000
#define PWM_MIN -10000
#define MOTOR1_PWM_DUTY_MAX (drv8701e_pwm_1_info.duty_max)
#define MOTOR2_PWM_DUTY_MAX (drv8701e_pwm_2_info.duty_max)

// 声明全局对象（不要在头文件定义）
extern zf_driver_gpio  drv8701e_dir_1;
extern zf_driver_gpio  drv8701e_dir_2;
extern zf_driver_pwm   drv8701e_pwm_1;
extern zf_driver_pwm   drv8701e_pwm_2;

extern struct pwm_info drv8701e_pwm_1_info;
extern struct pwm_info drv8701e_pwm_2_info;

// 函数可以用 extern "C" 包裹
#ifdef __cplusplus
extern "C" {
#endif

void motor_init(void);
void motor_control(int motor1, int motor2);

#ifdef __cplusplus
}
#endif

#endif