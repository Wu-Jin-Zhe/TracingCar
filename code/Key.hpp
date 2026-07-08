#ifndef __KEY_HPP__
#define __KEY_HPP__

#include "zf_common_headfile.hpp"

#define KEY0 0
#define KEY1 1
#define KEY2 2
#define KEY3 3

#define KEY_1_PATH ZF_GPIO_KEY_1
#define KEY_2_PATH ZF_GPIO_KEY_2
#define KEY_3_PATH ZF_GPIO_KEY_3
#define KEY_4_PATH ZF_GPIO_KEY_4

#define KEY_HOLD   0x01
#define KEY_DOWN   0x02
#define KEY_UP     0x04
#define KEY_SINGLE 0x08
#define KEY_DOUBLE 0x10
#define KEY_LONG   0x20
#define KEY_REPEAT 0x40

#ifdef __cplusplus
extern "C" {
#endif

int Key_GetState(int n);
void Key_Tick(void);
int Key_Check(int n, int Flag);

#ifdef __cplusplus
}
#endif

// 全局对象声明（不要在头文件中定义）
extern zf_driver_gpio key_1;
extern zf_driver_gpio key_2;
extern zf_driver_gpio key_3;
extern zf_driver_gpio key_4;

#endif