#include "encoder.hpp"


zf_driver_encoder encoder_dir_1(ENCODER_DIR_1_PATH);
zf_driver_encoder encoder_dir_2(ENCODER_DIR_2_PATH);

int16 speed_L = 0, speed_R = 0, speed_avl = 0;
int16 lastspeed_L = 0, lastspeed_R = 0;
int16 distance = 0;

void encoder_update(void)
{
    speed_L = encoder_dir_1.get_count();
    speed_R = -encoder_dir_2.get_count();

    // 一阶低通滤波
    speed_L = (0.9 * speed_L + 0.1 * lastspeed_L);
    speed_R = (0.9 * speed_R + 0.1 * lastspeed_R);
    lastspeed_L = speed_L;
    lastspeed_R = speed_R;

    speed_avl = (speed_L + speed_R)/2;
    distance += (int)speed_avl;

    encoder_dir_1.clear_count();
    encoder_dir_2.clear_count();
}

void encoder_clear(void)
{
     distance = 0;
}