#ifndef __CROSS_HPP__
#define __CROSS_HPP__

#pragma once

#include "zf_common_headfile.hpp"
#include <cstdint>

#ifdef __cplusplus
#include <mutex>   // C++的东西放外面
#endif

#define ROAD_WIDTH_MIN   40
#define ROAD_WIDTH_MAX   120
#define CROSS_POINT_NUM  5

typedef enum
{
    CROSS_NONE = 0,
    CROSS_IN,
} cross_state_t;

// 全局变量
extern cross_state_t cross_state;

// 函数声明（C接口）
#ifdef __cplusplus
extern "C" {
#endif

int cross_column_check(uint16 pts[][2], int corner_idx[], int corner_num);
void check_cross(void);
void run_cross(void);

#ifdef __cplusplus
}
#endif

// C++专用
#ifdef __cplusplus
extern std::mutex frame_mutex;
#endif

#endif