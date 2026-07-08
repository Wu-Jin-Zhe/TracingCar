#include "deal_img.hpp"
#include "cross.hpp"
#include "Ring.hpp"
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <cstdlib>     // ✅ 用 abs

cross_state_t cross_state = CROSS_NONE;
int cross_cnt = 0;

int leftmono = 0,rightmono = 0;



int cross_column_check(uint16 pts[][2], int corner_idx[], int corner_num)
{
    for(int i = 0; i < corner_num; i++)
    {
        int idx = corner_idx[i];
        int x = pts[idx][0];
        int y = pts[idx][1];  // y大的部分

        
        for(int j = 0; j < corner_num; j++)
        {
            int idx2 = corner_idx[j];
            int x2 = pts[idx2][0];
            int y2 = pts[idx2][1];

            if(i == j)
               continue;

            if(y2 > y)   // 必须在上面
                continue;

            if(abs(x2 - x) > 5)  // 同列容忍5像素
                continue;

            return 1;
        }
    }

    return 0;
}

void check_cross(void)
{

    if(ring_ctrl.type != RING_NONE)
    {
        return ;
    }

    if(cross_state == CROSS_IN)
    {
        return ;
    }

    if(Lptl_num + Lptr_num >= 2)
    {
        if(Lptl_num > 0 && Lptr_num > 0)
        {
            cross_cnt ++;
        }
        else
        {
           int left_cross = cross_column_check(points_l, Lpt_l, Lptl_num);
           int right_cross = cross_column_check(points_r, Lpt_r, Lptr_num);

           if((left_cross || right_cross))
           {
              int lx = points_l[0][0];
              int rx = points_r[0][0];
              int width = abs(rx - lx);

              if(width > ROAD_WIDTH_MIN)
              {
                 cross_cnt++;
              }
              else
              {
                 cross_cnt = 0;
              }
          }
          else
          {
              cross_cnt = 0;
          }
        } 

    }
    else
    {
        cross_cnt = 0;
    }

    if(cross_cnt >= 3)
    {
        cross_state = CROSS_IN;
        cross_cnt = 0;
        printf("find\r\n");
    }
}

void run_cross(void)
{
    // 如果当前处于十字状态
    if(cross_state == CROSS_IN)
    {
        // 判断退出条件
        // 1. 左右边界宽度太窄（恢复正常赛道宽度）或者
        // 2. 角点数不足（没有十字）
        int lx = points_l[0][0];
        int rx = points_r[0][0];
        int width = abs(rx - lx);

        if(width < ROAD_WIDTH_MIN || (Lptl_num + Lptr_num < 2))
        {
            cross_state = CROSS_NONE;  // 退出十字
        }
    }

    if(ring_ctrl.type != RING_NONE)
    {
        return ;
    }
    // 如果不是十字状态，不做任何处理
}