// tuoluo.cpp
#include "tuoluo.hpp"

// C++ 对象定义
zf_device_imu imu_dev;

// 其他全局变量定义
float angle;
float avl_gyro_z;
float null_drift_z = 0;
float gyro_z[2] = {0,0};
float next_gyro_z = 0;
int last_gyro_y;
int gyro_y[2] = {0};
float start_angle = 0;

int16 imu_acc_x, imu_acc_y, imu_acc_z;
int16 imu_gyro_x, imu_gyro_y, imu_gyro_z;
int16 imu_mag_x, imu_mag_y, imu_mag_z;

void imu_init(void)
{
    imu_dev.init();
}

void gyroscope_get_gyro(void)
{
    imu_acc_x = imu_dev.get_acc_x();
    imu_acc_y = imu_dev.get_acc_y();
    imu_acc_z = imu_dev.get_acc_z();

    imu_gyro_x = imu_dev.get_gyro_x();
    imu_gyro_y = imu_dev.get_gyro_y();
    imu_gyro_z = imu_dev.get_gyro_z();

    gyro_z[0] = 0.9 * (imu_gyro_z) + 0.1 * gyro_z[1];
    gyro_z[1] = gyro_z[0];

    avl_gyro_z = gyro_z[0] / 16.4f;


    angle += avl_gyro_z * 0.005f;
    // angle += avl_gyro_z * 0.006f;
}

void angle_clear(void)
{
    angle = 0;
}

void null_drift_calculate(void)
{
    static int16 cnt_null = 0;
    static float temp = 0;
    static int8 ret = 0;

    temp += imu_gyro_z;
    cnt_null++;

    if (cnt_null >= 200)
    {
        null_drift_z = temp / 200;
        ret = 1;
        angle_clear();
    }
}