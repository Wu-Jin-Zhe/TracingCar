#ifndef __ENCODER_HPP__
#define __ENCODER_HPP__

#pragma once

#include "zf_common_headfile.hpp"

#define ENCODER_DIR_1_PATH ZF_ENCODER_DIR_1
#define ENCODER_DIR_2_PATH ZF_ENCODER_DIR_2

// ❗这里只能声明
extern zf_driver_encoder encoder_dir_1;
extern zf_driver_encoder encoder_dir_2;

// 全局变量
extern int16 speed_L, speed_R, speed_avl;
extern int16 lastspeed_L, lastspeed_R;
extern int16 distance;

#ifdef __cplusplus
extern "C" {
#endif

void encoder_update(void);
void encoder_clear(void);

#ifdef __cplusplus
}
#endif

#endif