#include "zf_common_headfile.hpp"
#include "tuoluo.hpp"
#include "encoder.hpp"
#include "deal_img.hpp"
#include "close_loop.hpp"
#include "Key.hpp"
#include "vofa.hpp"
#include "Ring.hpp"
#include "cross.hpp"
#include "menu.hpp"
#include "motor.hpp"

#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>

// ========================= 网络可视化配置 =========================
#define ENABLE_VISUALIZATION    0

#if ENABLE_VISUALIZATION
#define SERVER_IP    "10.210.2.132"
#define PORT         8086
#define BOUNDARY_NUM 800

#define IMG_W   160
#define IMG_H   60

zf_driver_tcp_client tcp_client_dev;

// 灰度缓冲区，1字节/像素
uint8 image_copy_gray[IMG_H][IMG_W];

uint8   xy_x1_boundary[BOUNDARY_NUM];   // 左边界 X
uint8   xy_y1_boundary[BOUNDARY_NUM];   // 左边界 Y
uint8   xy_x2_boundary[BOUNDARY_NUM];   // 右边界 X
uint8   xy_y2_boundary[BOUNDARY_NUM];   // 右边界 Y
uint8   xy_x3_boundary[BOUNDARY_NUM];   // 中线   X
uint8   xy_y3_boundary[BOUNDARY_NUM];   // 中线   Y

extern zf_device_uvc uvc_dev;
extern uint8  imgotsu[60][160];
extern uint16 points_l[800][2];
extern uint16 points_r[800][2];
extern uint16 guide_line[800][2];
extern int    l_data_statics;
extern int    r_data_statics;
extern uint16 mid_count;

uint32 tcp_send_wrap(const uint8 *buf, uint32 len)
{
    return tcp_client_dev.send_data(buf, len);
}

uint32 tcp_read_wrap(uint8 *buf, uint32 len)
{
    return tcp_client_dev.read_data(buf, len);
}

static int copy_pts_to_boundary(uint16 pts[][2], int num,
                                uint8 *x_buf,    uint8 *y_buf)
{
    int n = (num > BOUNDARY_NUM) ? BOUNDARY_NUM : num;
    for (int i = 0; i < n; i++)
    {
        int x = pts[i][0];
        int y = pts[i][1];
        if (x < 0)      x = 0;
        if (x >= IMG_W) x = IMG_W - 1;
        if (y < 0)      y = 0;
        if (y >= IMG_H) y = IMG_H - 1;
        x_buf[i] = (uint8)x;
        y_buf[i] = (uint8)y;
    }
    return n;
}

static int visualization_init(void)
{
    if (tcp_client_dev.init(SERVER_IP, PORT) != 0)
    {
        printf("[VIS] tcp_client error, visualization disabled\r\n");
        return -1;
    }
    printf("[VIS] tcp_client ok\r\n");

    seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        image_copy_gray[0],
        IMG_W,
        IMG_H
    );

    seekfree_assistant_camera_boundary_config(
        XY_BOUNDARY,
        0,
        xy_x1_boundary, xy_x2_boundary, xy_x3_boundary,
        xy_y1_boundary, xy_y2_boundary, xy_y3_boundary
    );

    return 0;
}

// imgotsu → 灰度缓冲区，并在 x=80 处绘制中心参考线
static void imgotsu_to_gray(void)
{
    // 先 memcpy imgotsu
    memcpy(image_copy_gray, imgotsu, IMG_H * IMG_W * sizeof(uint8));

    // ★ 在 x=80 处画竖线，灰度值 128（在黑白图中显示为中灰，清晰可辨）
    for (int y = 0; y < IMG_H; y++)
    {
        image_copy_gray[y][80] = 128;
    }
}

static void visualization_send(void)
{
    // ★ 限速：每 3 帧发送一次，避免 TCP 缓冲区溢出
    static int skip_cnt = 0;
    skip_cnt++;
    if (skip_cnt < 3)
        return;
    skip_cnt = 0;

    // 1. imgotsu → 灰度缓冲区（含 x=80 中线）
    imgotsu_to_gray();

    // 2. 边界拷贝
    int l_num = copy_pts_to_boundary(points_l,   l_data_statics,
                                      xy_x1_boundary, xy_y1_boundary);
    int r_num = copy_pts_to_boundary(points_r,   r_data_statics,
                                      xy_x2_boundary, xy_y2_boundary);
    int m_num = copy_pts_to_boundary(guide_line, (int)mid_count,
                                      xy_x3_boundary, xy_y3_boundary);

    // 3. 最多点数作为发送点数
    int send_num = l_num;
    if (r_num > send_num) send_num = r_num;
    if (m_num > send_num) send_num = m_num;

    // 4. 更新边界配置
    seekfree_assistant_camera_boundary_config(
        XY_BOUNDARY,
        send_num,
        xy_x1_boundary, xy_x2_boundary, xy_x3_boundary,
        xy_y1_boundary, xy_y2_boundary, xy_y3_boundary
    );

    // 5. 发送
    seekfree_assistant_camera_send();
}

#endif  // ENABLE_VISUALIZATION

// ========================= 控制全局变量 =========================
zf_driver_pit pit_timer;
int time_flag = 0;
UartVofa* vofa = nullptr;

std::mutex display_mutex;
int ex_sp_l;
int ex_sp_r;

// ========================= 控制服务 =========================
void Pit_Fuwu(void)
{
    static int cnt = 0;
    cnt++;

    if(cnt >= 400)
    {
        time_flag = 0;
    }

    // Error = calculate_error();

    // direction_return(Error);

    // gyro_loop();

    // direction_loop(Error);

    loop_speed_LR(speed_target + correct_L, speed_target - correct_L);

    motor_control(out_L, out_R);
}

// ========================= 定时器中断 =========================
void pit_callback()
{
    Key_Tick();
    gyroscope_get_gyro();
    encoder_update();

    ex_sp_l = speed_target + correct_L;
    ex_sp_r = speed_target - correct_L;

    if (ring_ctrl.time_counter > 0)
    {
        ring_ctrl.time_counter--;
        
        if (ring_ctrl.time_counter <= 0)
        {
            ring_ctrl.time_counter = 0;
        }
            
    }

    // if (vofa)
    // {
    //     vofa->send_values(Error);
    // }

    if(vofa)
    {
       vofa->send_values(speed_target, speed_L, speed_R);
    }

    // if(vofa)
    // {
    //    vofa->send_values(ex_sp_l, speed_L,ex_sp_r,speed_R);
    // }

    // if(vofa)
    // {
    //    vofa->send_values(expect_gyro,avl_gyro_z);
    // }

    if (time_flag)
        Pit_Fuwu();
    else
        motor_control(0, 0);
}


//测试图像的
// void pit_callback()
// {   
//     gyroscope_get_gyro();
//     encoder_update();
//     Key_Tick();
    
//     if(ring_ctrl.time_counter > 0)
//     {
//         ring_ctrl.time_counter --;

//         if(ring_ctrl.time_counter <= 0)
//         {
//             ring_ctrl.time_counter = 0;
//         }
//     }
// }

// ========================= 主函数 =========================
int main()
{
    imu_init();
    motor_init();

    // ips200.init(FB_PATH);

    vofa = new UartVofa("/dev/ttyS0");

    if (uvc_dev.init(UVC_PATH) < 0)
        return -1;

#if ENABLE_VISUALIZATION
    bool vis_ok = (visualization_init() == 0);
#endif

    pit_timer.init_ms(5, pit_callback);

    while (1)
    {
//         if (uvc_dev.wait_image_refresh() == 0)
//         {
//             // auto start_time = std::chrono::high_resolution_clock::now();
//             deal_image();
//             // auto end_time = std::chrono::high_resolution_clock::now();
//             // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
//             // std::cout << "Frame processing time: " << duration.count() << " us" << std::endl;

// #if ENABLE_VISUALIZATION
//             if (vis_ok)
//                 visualization_send();
// #endif
//         }

        // debug_show_all();

        if (Key_Check(KEY0, KEY_SINGLE))
            time_flag = 1;
        else if (Key_Check(KEY1, KEY_SINGLE))
            time_flag = 0;
    }

    return 0;
}