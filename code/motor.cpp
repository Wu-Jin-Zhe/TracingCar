#include "motor.hpp"

// 定义全局对象
zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR);
zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR);
zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);
zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);

struct pwm_info drv8701e_pwm_1_info;
struct pwm_info drv8701e_pwm_2_info;

// 内联函数，不要放 extern "C"
static inline int pwm_limit(int v)
{
    if(v > PWM_MAX) return PWM_MAX;
    if(v < PWM_MIN) return PWM_MIN;
    return v;
}

void motor_init(void)
{
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);
}

void motor_control(int motor1, int motor2)
{
    uint32 duty;

    motor1 = pwm_limit(motor1);
    if(motor1 >= 0) 
    { 
        drv8701e_dir_1.set_level(1); 
        duty = motor1; 
    }
    else 
    { 
        drv8701e_dir_1.set_level(0);
        duty = (-motor1); 
    }

    if(duty > MOTOR1_PWM_DUTY_MAX) 
       duty = MOTOR1_PWM_DUTY_MAX;

    drv8701e_pwm_1.set_duty(duty);

    motor2 = pwm_limit(motor2);

    if(motor2 >= 0) 
    { 
        drv8701e_dir_2.set_level(1); 
        duty = motor2; 
    }
    else 
    { 
        drv8701e_dir_2.set_level(0);
        duty = (-motor2); 
    }

    if(duty > MOTOR2_PWM_DUTY_MAX) 
       duty = MOTOR2_PWM_DUTY_MAX;
       
    drv8701e_pwm_2.set_duty(duty);
}