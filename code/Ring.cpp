#include "deal_img.hpp"
#include "Ring.hpp"
#include "tuoluo.hpp"
#include "close_loop.hpp"
#include "cross.hpp"

#include <mutex>
#include <stdint.h>
#include <string.h>
#include <math.h>

static uint8 ring_l_cnt = 0;
static uint8 ring_r_cnt = 0;

#define  RING_CONFIRM_FRAMES   10     // 连续几帧确认圆环

//圆环参数
RingState ring_ctrl = 
{
    .state = Ring_state1,
    .type = RING_NONE,
    .phase_counter = 0,
    .sum_zangle = 0,
    .time_counter = 0,
    .flag = 0,
    .start_x = 0,
    .start_y = 0,
    .patch_Lpt  = 0,
    .patch_Lpt_x = 0,
    .patch_Lpt_y = 0,
    .patch_ux = 0.0f,
    .patch_uy = 0.0f,
    .patch_active = 0,
    .patch_Lpt_r   = 0,
    .patch_Lpt_r_x = 0,
    .patch_Lpt_r_y = 0,
    .patch_ux_r    = 0.0f,
    .patch_uy_r    = 0.0f,
    .patch_active_r = 0,
};

int check_boundary_monotone(uint16 pts[][2], int num)
{
    int last_x = -1;
    int first_x = -1;
    int last_y = -1;
    int inc_count = 0, dec_count = 0, flat_count = 0;
    int border_count = 0;

    for(int i = 0; i < num; i +=2 )
    {
        int x = pts[i][0];
        int y = pts[i][1];

        // 可选：过滤靠边的点
        if(x <= Deal_Left + 5 || x >= Deal_Right - 5) 
        {
           border_count++;
           continue;
        }
           

        if(first_x == -1)
            first_x = x;

        if(last_x >= 0)
        {
            int dx = x - last_x;
            int dy = y - last_y;
            if(dx > 0 ) inc_count++;
            else if(dx < 0 ) dec_count++;
            else flat_count++;
        }

        last_x = x;
        last_y = y;
    }

    if(border_count >= num * 0.8f)
    {

        return 3;
    }

    // 起点到终点变化判断
    if(abs(last_x - first_x) > 50)
    {

        return 0;
    }
    
    if(l_data_statics > 0 && r_data_statics > 0)
    {
        int lx = points_l[l_data_statics - 1][0];
        int rx = points_r[r_data_statics - 1][0];

        if(abs(rx - lx) < 50)
        {

           return 0;
        }
            
    }

    float inc_ratio = (inc_count + flat_count) * 1.0f / ( num / 2);
    float dec_ratio = (dec_count + flat_count) * 1.0f / ( num / 2);

    
    if(inc_ratio > 0.7f) 
       return 1;
       
    if(dec_ratio > 0.7f)
       return 2;


    return 0;
}


uint16 corner_above_border(uint16 pts[][2], int num, int corner_idx, int side)
{
    if(corner_idx < 0 || corner_idx >= num)
        return 0;

    int corner_y    = pts[corner_idx][1];
    int near_count  = 0;

    for(int i = 0; i < num; i++)
    {
        int x = pts[i][0];
        int y = pts[i][1];

        // 只检查角点上方15行（不含角点本行）
        if(y >= corner_y - 15 && y < corner_y)
        {
            if(side == 0 && x <= Deal_Left  + 8) near_count++;
            if(side == 1 && x >= Deal_Right - 8) near_count++;
        }
    }
    // 80%以上贴边才算有效
    return (near_count >= (int)(15 * 0.8f)) ? 1 : 0;
}

void check_ring(void)
{
    int leftmono = check_boundary_monotone(points_l, l_data_statics);
    int rightmono = check_boundary_monotone(points_r, r_data_statics);
    int left_cross = cross_column_check(points_l, Lpt_l, Lptl_num);
    int right_cross = cross_column_check(points_r, Lpt_r, Lptr_num);

    if(ring_ctrl.type != RING_NONE)
    {
        return;
    }

    if(ring_ctrl.time_counter > 0)
    {
        return;
    }

    // ★ 角点有效性过滤：y坐标太小（靠近图像顶部）的角点是假角点，直接忽略
    #define CORNER_MIN_Y  25

    int valid_lptl_num = 0;
    int valid_lptr_num = 0;

    for(int i = 0; i < Lptl_num; i++)
    {
        if(points_l[Lpt_l[i]][1] >= CORNER_MIN_Y)
            valid_lptl_num++;
    }

    for(int i = 0; i < Lptr_num; i++)
    {
        if(points_r[Lpt_r[i]][1] >= CORNER_MIN_Y)
            valid_lptr_num++;
    }

    int left_valid_idx = -1;
    for(int i = 0; i < Lptl_num; i++)
    {
        if(points_l[Lpt_l[i]][1] >= CORNER_MIN_Y)
        {
            left_valid_idx = Lpt_l[i];
            break;
        }
    }

    int left_corner_border = corner_above_border(points_l, l_data_statics, left_valid_idx, 0);

    // ----------------------------------------
    // 左圆环判定
    // ---------------------------------------- 
    bool left_ring_cond =
           (valid_lptl_num == 1) && (valid_lptr_num == 0)
        && (leftmono == 0) && (rightmono == 2)
        && (!left_cross) && (!right_cross) && (left_corner_border);

    if(left_ring_cond)
    {
        
        float l_r_ratio = (r_data_statics > 0)
                          ? (float)l_data_statics / (float)r_data_statics
                          : 999.0f;

        bool length_ok = (l_data_statics >= r_data_statics + 40)
                      || (l_r_ratio >= 1.2f);

        if(length_ok)
        {
            ring_l_cnt++;
            if(ring_l_cnt >= RING_CONFIRM_FRAMES)
            {
                ring_ctrl.type  = RING_LEFT;
                ring_ctrl.state = Ring_state1;
                ring_ctrl.flag  = 1;
                ring_ctrl.patch_active = 1; 
                // ⭐ 保存识别时的角点坐标作为补线锚点
                if(Lptl_num > 0)
                {
                    int idx = Lpt_l[0];
                    ring_ctrl.patch_Lpt   = idx;
                    ring_ctrl.patch_Lpt_x = points_l[idx][0];
                    ring_ctrl.patch_Lpt_y = points_l[idx][1];
                }
                printf("left_ring\r\n");
                ring_l_cnt = 0;
            }
        }
        else
        {
            ring_l_cnt = 0;
        }
    }
    else
    {
        ring_l_cnt = 0;
    }

    // ----------------------------------------
    // 右圆环判定
    // ----------------------------------------

    int right_valid_idx = -1;

    for(int i = 0; i < Lptr_num; i++)
    {
        if(points_r[Lpt_r[i]][1] >= CORNER_MIN_Y)
        {
            right_valid_idx = Lpt_r[i];
            break;
        }
    }

    int right_corner_border = corner_above_border(points_r, r_data_statics, right_valid_idx, 1);

    bool right_ring_cond =
           (valid_lptr_num == 1) && (valid_lptl_num == 0)
        && (leftmono == 1) && (rightmono == 0)
        && (!right_cross) && (!left_cross) && (right_corner_border);

    if(right_ring_cond)
    {
        float r_l_ratio = (l_data_statics > 0)
                          ? (float)r_data_statics / (float)l_data_statics
                          : 999.0f;

        bool length_ok = (r_data_statics >= l_data_statics + 50)
                      || (r_l_ratio >= 1.4f);

        if(length_ok)
        {
            ring_r_cnt++;
            if(ring_r_cnt >= RING_CONFIRM_FRAMES)
            {
                ring_ctrl.type           = RING_RIGHT;
                ring_ctrl.state          = Ring_state1;
                ring_ctrl.flag           = 1;
                ring_ctrl.patch_active_r = 1;
                ring_ctrl.patch_ux_r     = 0.0f;
                ring_ctrl.patch_uy_r     = 0.0f;
                if(Lptr_num > 0)
                {
                    int idx = Lpt_r[0];
                    ring_ctrl.patch_Lpt_r   = idx;
                    ring_ctrl.patch_Lpt_r_x = points_r[idx][0];
                    ring_ctrl.patch_Lpt_r_y = points_r[idx][1];
                }
                printf("right_ring\r\n");
                ring_r_cnt = 0;
            }
        }
        else
        {
            ring_r_cnt = 0;
        }
    }
    else
    {
        ring_r_cnt = 0;
    }
}

uint8 check_trend_left(uint16 pts[][2], int num, int start_idx)
{
    if(start_idx >= num - 1)
        return 0;

    int inc_x = 0;
    int dec_y = 0;
    int dist;

    dist = num - start_idx - 1;

    for(int i = start_idx; i < num - 1; i += 2)
    {
        int dx = pts[i + 1][0] - pts[i][0];
        int dy = pts[i + 1][1] - pts[i][1];

        if(dx > 0)
            inc_x++;

        if(dy < 0)
            dec_y++;

    }

    float x_ratio = inc_x * 1.0f / (dist / 2);
    float y_ratio = dec_y * 1.0f / (dist / 2);

    if(x_ratio > 0.7f && y_ratio > 0.7f)
        return 1;

    return 0;
}

uint8 check_trend_right(uint16 pts[][2], int num, int start_idx)
{
    if(start_idx >= num - 1)
        return 0;

    int dec_x = 0;
    int dec_y = 0;
    int dist;

    dist = num - start_idx - 1;

    for(int i = start_idx; i < num - 1; i += 2)
    {
        int dx = pts[i + 1][0] - pts[i][0];
        int dy = pts[i + 1][1] - pts[i][1];

        if(dx < 0)
            dec_x++;

        if(dy < 0)
            dec_y++;

    }

    float x_ratio = dec_x * 1.0f / (dist / 2);
    float y_ratio = dec_y * 1.0f / (dist / 2);

    if(x_ratio > 0.7f && y_ratio > 0.7f)
        return 1;

    return 0;
}

void add_line(uint16 pts[][2], int *num, int start, float dx, float dy)
{
    float x = pts[start][0];
    float y = pts[start][1];

    float len = sqrtf(dx*dx + dy*dy);

    if(len < 1e-3f) return;

    // 单位方向
    float ux = dx / len;
    float uy = dy / len;

    int step = start + 1;

    // 一直走到图像顶部
    while(y > Deal_Top - 2 && step < 800)
    {
        x += ux;
        y += uy;

        if(x < 0 || x >= XX) 
           break;

        pts[step][0] = (uint16)x;
        pts[step][1] = (uint16)y;

        step++;
    }

    *num = step;
}

uint16 add_three_point_bezier(uint16 p0[2], uint16 p1[2], uint16 p2[2],
                  uint16 pts[][2], int add_location, float dist)
{
    if (add_location >= 800) return 0;

    float dx1 = p1[0] - p0[0];
    float dy1 = p1[1] - p0[1];
    float len1 = sqrtf(dx1*dx1 + dy1*dy1);

    float dx2 = p2[0] - p1[0];
    float dy2 = p2[1] - p1[1];
    float len2 = sqrtf(dx2*dx2 + dy2*dy2);

    float curve_len = (len1 + len2) * 0.8f;

    int max_pts = curve_len / dist + 2;
    if (max_pts > (800 - add_location))
        max_pts = 800 - add_location;

    if (max_pts < 2) return 0;

    float step = 1.0f / (max_pts - 1);

    float prev_x = p0[0];
    float prev_y = p0[1];

    int len = 0;

    // 存起点
    pts[add_location][0] = p0[0];
    pts[add_location][1] = p0[1];
    len++;

    for (float t = step; t <= 1.0f; t += step)
    {
        float it = 1.0f - t;

        float x = it*it*p0[0] + 2*it*t*p1[0] + t*t*p2[0];
        float y = it*it*p0[1] + 2*it*t*p1[1] + t*t*p2[1];

        float dx = x - prev_x;
        float dy = y - prev_y;
        float d  = sqrtf(dx*dx + dy*dy);

        if (d >= dist)
        {
            if (x < 0 || x >= XX || y < 0 || y >= YY)
                break;

            pts[add_location + len][0] = (uint16)x;
            pts[add_location + len][1] = (uint16)y;

            prev_x = x;
            prev_y = y;

            len++;
        }
    }

    // 强制加终点
    if (len < (800 - add_location))
    {
        pts[add_location + len][0] = p2[0];
        pts[add_location + len][1] = p2[1];
        len++;
    }

    return len;
}

int find_top_left_point(uint16 pts[][2], int num)
{
    int idx = 0;
    int min_dist = 1000000;  // 足够大的初值

    for(int i = 0; i < num; i++)
    {
        int x = pts[i][0];
        int y = pts[i][1];

        // 距离左上角 (0,0) 的平方距离
        int dist = x*x + y*y;

        if(dist < min_dist)
        {
            min_dist = dist;
            idx = i;
        }
    }

    return idx;
}

int find_top_right_point(uint16 pts[][2], int num)
{
    int idx = 0;
    int min_dist = 1000000;  // 足够大的初值

    int corner_x = XX - 1;   // 右上角 X
    int corner_y = 0;        // 右上角 Y

    for(int i = 0; i < num; i++)
    {
        int x = pts[i][0];
        int y = pts[i][1];

        // 距离右上角的平方距离
        int dist = (x - corner_x)*(x - corner_x) + (y - corner_y)*(y - corner_y);

        if(dist < min_dist)
        {
            min_dist = dist;
            idx = i;
        }
    }

    return idx;
}

void run_ring(void)
{
    if(ring_ctrl.type == RING_LEFT )
    {
       switch(ring_ctrl.state)
       {
            //寻找入环时机
            case Ring_state1:
            {
                // ========== 第一步：用 patch_Lpt 角点斜率向上补线 ==========
                static uint16 points_l_patched[800][2];
                int l_patched_num = 0;

                if(Lptl_num > 0 && ring_ctrl.patch_active == 1)
                {

                    int idx = ring_ctrl.patch_Lpt;

                    // 更新角点坐标
                    ring_ctrl.patch_Lpt_x = points_l[idx][0];
                    ring_ctrl.patch_Lpt_y = points_l[idx][1];

                    // 斜率只算一次
                    if(ring_ctrl.patch_ux == 0.0f && ring_ctrl.patch_uy == 0.0f)
                    {
                        int idx_far  = idx - 20;
                        int idx_near = idx - 8;
                        if(idx_far  < 0) idx_far  = 0;
                        if(idx_near < 0) idx_near = 0;

                        float slope_dx = (float)(points_l[idx_near][0] - points_l[idx_far][0]);
                        float slope_dy = (float)(points_l[idx_near][1] - points_l[idx_far][1]);

                        float slope_len = sqrtf(slope_dx * slope_dx + slope_dy * slope_dy);
                        if(slope_len < 1e-3f) slope_len = 1.0f;

                        ring_ctrl.patch_ux = slope_dx / slope_len;
                        ring_ctrl.patch_uy = slope_dy / slope_len;
                    }

                    // 角点之前的部分原样复制
                    for(int i = 0; i < idx && i < 800; i++)
                    {
                        points_l_patched[i][0] = points_l[i][0];
                        points_l_patched[i][1] = points_l[i][1];
                    }
                    l_patched_num = idx;

                    // 从角点出发沿固定斜率向上延伸
                    float cx = (float)ring_ctrl.patch_Lpt_x;
                    float cy = (float)ring_ctrl.patch_Lpt_y;

                    while(cy > Deal_Top && l_patched_num < 800)
                    {
                        points_l_patched[l_patched_num][0] = (uint16)cx;
                        points_l_patched[l_patched_num][1] = (uint16)cy;
                        cx += ring_ctrl.patch_ux;
                        cy += ring_ctrl.patch_uy;
                        l_patched_num++;
                    }
                }
                else if(Lptl_num == 0 && ring_ctrl.patch_active == 1)
                {
                    // 角点消失：清除标记
                    ring_ctrl.patch_active = 0;
                    ring_ctrl.patch_ux     = 0.0f;
                    ring_ctrl.patch_uy     = 0.0f;
                    ring_ctrl.patch_Lpt    = 0;    
                    ring_ctrl.patch_Lpt_x  = 0;    
                    ring_ctrl.patch_Lpt_y  = 0;    

                    // 不补线，直接用原始 points_l
                    for(int i = 0; i < l_data_statics && i < 800; i++)
                    {
                        points_l_patched[i][0] = points_l[i][0];
                        points_l_patched[i][1] = points_l[i][1];
                    }
                    l_patched_num = l_data_statics;
                }
                else
                {
                    // patch_active == 0：角点已消失过，直接用原始 points_l
                    for(int i = 0; i < l_data_statics && i < 800; i++)
                    {
                        points_l_patched[i][0] = points_l[i][0];
                        points_l_patched[i][1] = points_l[i][1];
                    }
                    l_patched_num = l_data_statics;
                }

                // ========== 第二步：用临时左边界 + 原始右边界计算中线 ==========
                int mid_count_local = 0;

                for(int y = Deal_Bottom; y >= Deal_Top - 2; y--)
                {
                    int lx = -1;
                    int rx = -1;

                    for(int i = 0; i < l_patched_num; i++)
                    {
                        if(points_l_patched[i][1] == y && points_l_patched[i][0] > lx)
                            lx = points_l_patched[i][0];
                    }

                    for(int i = 0; i < r_data_statics; i++)
                    {
                        if(points_r[i][1] == y && (rx == -1 || points_r[i][0] < rx))
                            rx = points_r[i][0];
                    }

                    if(lx != -1 && rx != -1)
                    {
                        center_pts[mid_count_local][0] = (lx + rx) / 2;
                        center_pts[mid_count_local][1] = y;
                        mid_count_local++;
                    }

                    if(mid_count_local >= 500) break;
                }

                center_pts_num = mid_count_local;

                // ========== 第三步：patch_active==0 后新角点出现，进入 State2 ==========

                if(Lptl_num > 0 && ring_ctrl.patch_active == 0)
                {
                    for(int i = 0; i < Lptl_num; i++)
                    {
                        int idx = Lpt_l[i];
                        int trend = check_trend_left(points_l, l_data_statics, idx);

                        if(trend == 1)
                        {
                            ring_ctrl.state         = Ring_state2;
                            ring_ctrl.Lpt           = idx;
                            ring_ctrl.Lpt_x         = points_l[idx][0];
                            ring_ctrl.Lpt_y         = points_l[idx][1];
                            ring_ctrl.phase_counter = 0;
                            printf("enter Ring_state2,\r\n");
                            angle_clear();
                            break;
                        }
                    }
                }

                break;
            }
            //入环补线
            case Ring_state2:
            {

                int ida = (Lptl_num > 0) ? Lpt_l[0] : (l_data_statics - 1);

                // ===== 右边界贝塞尔曲线 =====
                uint16 p0[2] = { points_r[0][0], points_r[0][1] };
                uint16 p2[2] = { points_l[ida][0], points_l[ida][1] };
                uint16 p1[2];
                p1[0] = (p0[0] + p2[0]) / 2;
                p1[1] = (p0[1] + p2[1]) / 2 - 15;

                uint16 added = add_three_point_bezier(p0, p1, p2,
                                                    points_r, r_data_statics, 1.0f);
                r_data_statics = r_data_statics + added;

                // ===== 中线贝塞尔：用左右边界实际间距计算半宽 =====
                // p0 是右边界底部，points_l[0] 是左边界底部，两者x差即为赛道宽
                // 取底部和顶部各一个点估算平均半宽
                uint16 left_bot_x  = points_l[0][0];
                uint16 right_bot_x = points_r[0][0];

                // 赛道实际半宽（动态计算，比写死更准）
                // 右边界x > 左边界x（图像坐标，右大左小）
                float half_w = (right_bot_x > left_bot_x) 
                            ? (float)(right_bot_x - left_bot_x) * 0.5f 
                            : 40.0f;  // fallback

                // 中线控制点 = 右边界控制点向左平移 half_w
                uint16 mp0[2], mp1[2], mp2[2];

                mp0[0] = (p0[0] > (uint16)half_w) ? (uint16)(p0[0] - half_w) : 0;
                mp0[1] = p0[1];

                mp1[0] = (p1[0] > (uint16)half_w) ? (uint16)(p1[0] - half_w) : 0;
                mp1[1] = p1[1];

                mp2[0] = (p2[0] > (uint16)half_w) ? (uint16)(p2[0] - half_w) : 0;
                mp2[1] = p2[1];

                uint16 mid_added = add_three_point_bezier(mp0, mp1, mp2,
                                                        center_pts, 0, 1.0f);
                center_pts_num = mid_added;

                // ===== 状态切换 =====
                if(angle <= -45.0)
                    ring_ctrl.phase_counter++;
                else
                    ring_ctrl.phase_counter = 0;

                if(ring_ctrl.phase_counter >= 5)
                {
                    ring_ctrl.state = Ring_state3;
                    printf("enter Ring_state3 :%.2f\r\n", angle);
                    ring_ctrl.phase_counter = 0;
                }

                break;
            }
            //进入环岛
            case Ring_state3:
            { 
                printf("angle:%.2f\r\n",angle);
                
                int mid_count_local = 0;

                for(int y = Deal_Bottom; y >= Deal_Top - 2; y--)
                {
                    int lx = -1;
                    int rx = -1;

                    // 找这一行左边界
                    for(int i = 0; i < l_data_statics; i++)
                    {
                        if(points_l[i][1] == y)
                        {
                            if(points_l[i][0] > lx)   // 取最右
                                lx = points_l[i][0];
                        }
                    }

                    // 找这一行右边界
                    for(int i = 0; i < r_data_statics; i++)
                    {
                        if(points_r[i][1] == y)
                        {
                            if(rx == -1 || points_r[i][0] < rx)  // 取最左
                                rx = points_r[i][0];
                        }
                    }

                    if(lx != -1 && rx != -1)
                    {
                        center_pts[mid_count_local][0] = (lx + rx) / 2;
                        center_pts[mid_count_local][1] = y;

                        mid_count_local++;
                        if(mid_count_local >= 500)
                            break;
                    }
                }

                center_pts_num = mid_count_local;

                if(Lptl_num == 0 && Lptr_num == 1 && angle <= -220.0)
                {
                   ring_ctrl.phase_counter ++;
                }

                if(ring_ctrl.phase_counter >= 3) 
                {
                    ring_ctrl.state = Ring_state4;
                    printf("enter Ring_state4 : %.2f\r\n",angle);
                    ring_ctrl.phase_counter = 0;
                }
                break;
            }

            // 左环 Ring_state4
            case Ring_state4:
            {
                int ida = Lpt_r[0];
                ring_ctrl.Lpt = ida;

                uint16 p0[2] = { points_r[ida][0], points_r[ida][1] };
                int top_left_idx = find_top_left_point(points_l, l_data_statics);
                uint16 p2[2] = { points_l[top_left_idx][0], points_l[top_left_idx][1] };

                uint16 p1[2];
                p1[0] = (p0[0] + p2[0]) / 2;
                p1[1] = (p0[1] + p2[1]) / 2;

                uint16 added_points = add_three_point_bezier(p0, p1, p2, points_r, ida, 0.5f);
                r_data_statics = ida + added_points;

                // 直接偏移，不做贝塞尔
                track_right_to_center(points_r, r_data_statics, center_pts, 5, 25.0f);
                center_pts_num = r_data_statics;

                if(angle <= -230.0 && Lptr_num == 0)
                    ring_ctrl.phase_counter++;

                if(ring_ctrl.phase_counter >= 5)
                {
                    ring_ctrl.state = Ring_state5;
                    printf("enter Ring_state5:%.2f\r\n", angle);
                    ring_ctrl.phase_counter = 0;
                }
                break;
            }
            //退出圆环
            case Ring_state5:
            {

                // ================== 强制引导线（右下 -> 左上） ==================
                int mid_count_local = 0;

                float x = Deal_Right - 5;   // ⭐ 右下起点
                float y = Deal_Bottom;

                float target_x = Deal_Left + 5;  // ⭐ 左上终点
                float target_y = Deal_Top * 4;

                float dx = target_x - x;
                float dy = target_y - y;

                float len = sqrtf(dx * dx + dy * dy);

                if(len > 1e-3f)
                {
                    float ux = dx / len;
                    float uy = dy / len;

                    while(y > Deal_Top && mid_count_local < 500)
                    {
                        center_pts[mid_count_local][0] = (uint16)x;
                        center_pts[mid_count_local][1] = (uint16)y;

                        x += ux * 1.0f;   // ⭐ 和右环一致
                        y += uy * 1.0f;

                        mid_count_local++;
                    }
                }

                center_pts_num = mid_count_local;

                // ================== 出环判断 ==================
                if(angle <= -250.0)
                {
                    ring_ctrl.phase_counter++;

                    if(ring_ctrl.phase_counter >= 5)
                    {
                        ring_ctrl.type = RING_NONE;
                        printf("break :%.2f\r\n",angle);
                        ring_ctrl.flag = 0;
                        ring_ctrl.phase_counter = 0;  
                        ring_ctrl.time_counter = 800;
                    }
                }

                break;
            }

            case Ring_state6:
            {
                break;
            }

            default:
                break;
       }
    }
    else if(ring_ctrl.type == RING_RIGHT )
    { 
        switch(ring_ctrl.state)
        {
            // ================== 寻找入环 ==================
            case Ring_state1:
            {
                // ========== 第一步：用 patch_Lpt_r 角点斜率向上补线 ==========
                static uint16 points_r_patched[800][2];
                int r_patched_num = 0;

                if(Lptr_num > 0 && ring_ctrl.patch_active_r == 1)
                {
                    int idx = ring_ctrl.patch_Lpt_r;

                    // 更新角点坐标
                    ring_ctrl.patch_Lpt_r_x = points_r[idx][0];
                    ring_ctrl.patch_Lpt_r_y = points_r[idx][1];

                    // 斜率只算一次
                    if(ring_ctrl.patch_ux_r == 0.0f && ring_ctrl.patch_uy_r == 0.0f)
                    {
                        int idx_far  = idx - 20;
                        int idx_near = idx - 8;
                        if(idx_far  < 0) idx_far  = 0;
                        if(idx_near < 0) idx_near = 0;

                        float slope_dx = (float)(points_r[idx_near][0] - points_r[idx_far][0]);
                        float slope_dy = (float)(points_r[idx_near][1] - points_r[idx_far][1]);

                        float slope_len = sqrtf(slope_dx * slope_dx + slope_dy * slope_dy);
                        if(slope_len < 1e-3f) slope_len = 1.0f;

                        ring_ctrl.patch_ux_r = slope_dx / slope_len;
                        ring_ctrl.patch_uy_r = slope_dy / slope_len;
                    }

                    // 角点之前的部分原样复制
                    for(int i = 0; i < idx && i < 800; i++)
                    {
                        points_r_patched[i][0] = points_r[i][0];
                        points_r_patched[i][1] = points_r[i][1];
                    }
                    r_patched_num = idx;

                    // 从角点出发沿固定斜率向上延伸
                    float cx = (float)ring_ctrl.patch_Lpt_r_x;
                    float cy = (float)ring_ctrl.patch_Lpt_r_y;

                    while(cy > Deal_Top && r_patched_num < 800)
                    {
                        points_r_patched[r_patched_num][0] = (uint16)cx;
                        points_r_patched[r_patched_num][1] = (uint16)cy;
                        cx += ring_ctrl.patch_ux_r;
                        cy += ring_ctrl.patch_uy_r;
                        r_patched_num++;
                    }
                }
                else if(Lptr_num == 0 && ring_ctrl.patch_active_r == 1)
                {
                    // 角点消失：清除标记
                    ring_ctrl.patch_active_r = 0;
                    ring_ctrl.patch_ux_r     = 0.0f;
                    ring_ctrl.patch_uy_r     = 0.0f;

                    // 不补线，直接用原始 points_r
                    for(int i = 0; i < r_data_statics && i < 800; i++)
                    {
                        points_r_patched[i][0] = points_r[i][0];
                        points_r_patched[i][1] = points_r[i][1];
                    }
                    r_patched_num = r_data_statics;
                }
                else
                {
                    // patch_active_r == 0：角点已消失过，直接用原始 points_r
                    for(int i = 0; i < r_data_statics && i < 800; i++)
                    {
                        points_r_patched[i][0] = points_r[i][0];
                        points_r_patched[i][1] = points_r[i][1];
                    }
                    r_patched_num = r_data_statics;
                }

                // ========== 第二步：用原始左边界 + 临时右边界计算中线 ==========
                int mid_count_local = 0;

                for(int y = Deal_Bottom; y >= Deal_Top - 2; y--)
                {
                    int lx = -1;
                    int rx = -1;

                    for(int i = 0; i < l_data_statics; i++)
                    {
                        if(points_l[i][1] == y && points_l[i][0] > lx)
                            lx = points_l[i][0];
                    }

                    for(int i = 0; i < r_patched_num; i++)
                    {
                        if(points_r_patched[i][1] == y && (rx == -1 || points_r_patched[i][0] < rx))
                            rx = points_r_patched[i][0];
                    }

                    if(lx != -1 && rx != -1)
                    {
                        center_pts[mid_count_local][0] = (lx + rx) / 2;
                        center_pts[mid_count_local][1] = y;
                        mid_count_local++;
                    }

                    if(mid_count_local >= 500) break;
                }

                center_pts_num = mid_count_local;

                // ========== 第三步：patch_active_r==0 后新角点出现，进入 State2 ==========

                if(Lptr_num > 0 && ring_ctrl.patch_active_r == 0)
                {
                    for(int i = 0; i < Lptr_num; i++)
                    {
                        int idx = Lpt_r[i];
                        int trend = check_trend_right(points_r, r_data_statics, idx);

                        if(trend == 1)
                        {
                            ring_ctrl.state         = Ring_state2;
                            ring_ctrl.Lpt           = idx;
                            ring_ctrl.Lpt_x         = points_r[idx][0];
                            ring_ctrl.Lpt_y         = points_r[idx][1];
                            ring_ctrl.phase_counter = 0;
                            angle_clear();
                            printf("enter Ring_state2,\r\n");
                            break;
                        }
                    }
                }

                break;
            }

            // ================== 入环补线 ==================
            case Ring_state2:
            {
                int ida = (Lptr_num > 0) ? Lpt_r[0] : (r_data_statics - 1);

                // ===== 左边界贝塞尔补线（与左环对称）=====
                uint16 p0[2] = { points_l[0][0], points_l[0][1] };
                uint16 p2[2] = { points_r[ida][0], points_r[ida][1] };
                uint16 p1[2];
                p1[0] = (p0[0] + p2[0]) / 2;
                p1[1] = (p0[1] + p2[1]) / 2 - 15;

                uint16 added = add_three_point_bezier(p0, p1, p2,
                                                    points_l, l_data_statics, 1.0f);
                l_data_statics = l_data_statics + added;

                // ===== 中线贝塞尔：用左右边界实际间距计算半宽 =====
                uint16 left_bot_x  = points_l[0][0];
                uint16 right_bot_x = points_r[0][0];

                // 赛道实际半宽（动态计算）
                // 右环：右边界x > 左边界x，中线 = 左边界控制点向右平移 half_w
                float half_w = (right_bot_x > left_bot_x)
                            ? (float)(right_bot_x - left_bot_x) * 0.5f
                            : 40.0f;  // fallback

                // 中线控制点 = 左边界控制点向右平移 half_w
                uint16 mp0[2], mp1[2], mp2[2];

                mp0[0] = (uint16)(p0[0] + half_w);
                mp0[1] = p0[1];

                mp1[0] = (uint16)(p1[0] + half_w);
                mp1[1] = p1[1];

                mp2[0] = (uint16)(p2[0] + half_w);
                mp2[1] = p2[1];

                uint16 mid_added = add_three_point_bezier(mp0, mp1, mp2,
                                                        center_pts, 0, 1.0f);
                center_pts_num = mid_added;

                // ===== 状态切换 =====
                if(angle >= 35.0)
                    ring_ctrl.phase_counter++;
                else
                    ring_ctrl.phase_counter = 0;

                if(ring_ctrl.phase_counter >= 5)
                {
                    ring_ctrl.state = Ring_state3;
                    printf("enter Ring_state3 :%.2f\r\n", angle);
                    ring_ctrl.phase_counter = 0;
                }

                break;
            }

            // ================== 环内 ==================
            case Ring_state3:
            {
                 
                int mid_count_local = 0;

                for(int y = Deal_Bottom; y >= Deal_Top - 2; y--)
                {
                    int lx = -1;
                    int rx = -1;

                    for(int i = 0; i < l_data_statics; i++)
                    {
                        if(points_l[i][1] == y)
                        {
                            if(points_l[i][0] > lx)
                                lx = points_l[i][0];
                        }
                    }

                    for(int i = 0; i < r_data_statics; i++)
                    {
                        if(points_r[i][1] == y)
                        {
                            if(rx == -1 || points_r[i][0] < rx)
                                rx = points_r[i][0];
                        }
                    }

                    if(lx != -1 && rx != -1)
                    {
                        center_pts[mid_count_local][0] = (lx + rx) / 2;
                        center_pts[mid_count_local][1] = y;

                        mid_count_local++;
                        if(mid_count_local >= 500)
                            break;
                    }
                }

                center_pts_num = mid_count_local;

                if(Lptr_num == 0 && Lptl_num == 1 && angle >= 220.0)
                {
                    ring_ctrl.phase_counter++;
                }

                if(ring_ctrl.phase_counter >= 5)
                {
                    ring_ctrl.state = Ring_state4;
                    printf("enter Ring_state4 :%.2f\r\n",angle);
                    ring_ctrl.phase_counter = 0;
                }

                break;
            }

            // ================== 出环判断 ==================
            case Ring_state4:
            {
                int ida = Lpt_l[0];
                ring_ctrl.Lpt = ida - 7;

                uint16 p0[2] = { points_l[ida][0], points_l[ida][1] };
                int top_right_idx = find_top_right_point(points_r, r_data_statics);
                uint16 p2[2] = { points_r[top_right_idx][0], points_r[top_right_idx][1] };

                uint16 p1[2];
                p1[0] = (p0[0] + p2[0]) / 2;
                p1[1] = (p0[1] + p2[1]) / 2;

                uint16 added_points = add_three_point_bezier(p0, p1, p2, points_l, ida, 0.5f);
                l_data_statics = ida + added_points;

                // 直接偏移，不做贝塞尔
                track_left_to_center(points_l, l_data_statics, center_pts, 5, 25.0f);
                center_pts_num = l_data_statics;

                if(angle >= 240.0 && Lptl_num == 0)
                    ring_ctrl.phase_counter++;

                if(ring_ctrl.phase_counter >= 5)
                {
                    ring_ctrl.state = Ring_state5;
                    printf("enter Ring_state5 :%.2f\r\n", angle);
                    ring_ctrl.phase_counter = 0;
                }
                break;
            }

            // ================== 出环 ==================
            case Ring_state5:
            {

                // ================== 强制引导线（左下 -> 右上） ==================
                int mid_count_local = 0;

                float x = Deal_Left + 5;         // 左下起点（稍微往里一点）
                float y = Deal_Bottom;

                float target_x = Deal_Right - 5; // 右上终点
                float target_y = Deal_Top * 4;

                float dx = target_x - x;
                float dy = target_y - y;

                float len = sqrtf(dx * dx + dy * dy);

                if(len > 1e-3f)
                {
                    float ux = dx / len;
                    float uy = dy / len;

                    while(y > Deal_Top && mid_count_local < 500)
                    {
                        center_pts[mid_count_local][0] = (uint16)x;
                        center_pts[mid_count_local][1] = (uint16)y;

                        x += ux * 1.0f;   // ⭐ 步长可以调（2.0更平滑）
                        y += uy * 1.0f;

                        mid_count_local++;
                    }
                }

                center_pts_num = mid_count_local;

                // ================== 出环判断 ==================
                if(angle >= 260.0)
                {
                    ring_ctrl.phase_counter++;

                    if(ring_ctrl.phase_counter >= 5)
                    {
                        ring_ctrl.type = RING_NONE;
                        printf("break :%.2f\r\n",angle);
                        ring_ctrl.flag = 0;
                        ring_ctrl.phase_counter = 0;  
                        ring_ctrl.time_counter = 800;
                    }
                }

                break;
            }

            default:
                break;
        }
    }
    else
    {
        return;
    }
}