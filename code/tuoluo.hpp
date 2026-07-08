// tuoluo.hpp
#ifndef __TUOLUO_H__
#define __TUOLUO_H__

#include "zf_common_headfile.hpp"

// C++ 对象声明（不要放在 extern "C" 块里）
extern zf_device_imu imu_dev;

// 普通 C 全局变量可以用 extern "C"
#ifdef __cplusplus
extern "C" {
#endif

extern float angle;
extern float avl_gyro_z;
extern int last_gyro_y;
extern float start_angle;

void imu_init(void);
void gyroscope_get_gyro(void);
void angle_clear(void);
void null_drift_calculate(void);

#ifdef __cplusplus
}
#endif

#endif
