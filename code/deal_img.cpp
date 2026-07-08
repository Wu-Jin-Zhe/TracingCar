#include "deal_img.hpp"
#include <stdio.h>
#include "Ring.hpp"
#include "cross.hpp"
#include "encoder.hpp"
#include "tuoluo.hpp"

#include <opencv2/imgproc/imgproc.hpp>  // for cv::cvtColor
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#include <opencv2/opencv.hpp>

#include <iostream> // for std::cerr
#include <fstream>  // for std::ofstream
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <stdint.h>
#include <string.h>
#include <math.h>

/*** 宏定义区域 ***/
#define    start_w   ((160 - XX) / 2)  // (160-140)/2 = 10
#define    end_r     10


/*** 变量区域 ***/
zf_device_uvc uvc_dev; 
std::mutex frame_mutex;
uint8 imgotsu[60][160];   //二值化图像
uint8 imggray[60][160];   //灰度图像
uint8 img_threshold_group[3];  //三种不同距离的阈值
uint8 close_Threshold = 0;
uint8 far_Threshold = 0;
uint8 mid_Threshold = 0;
//每一列的白色像素点数量
uint32 white_col[XX] = {0};
//每一行的白色像素点数量
uint32 white_row[YY] = {0};
float sample_dist = 0.02;
float pixel_per_meter = 51.0f;
uint16 points_l[800][2] = { { 0 } };//左线
uint16 points_r[800][2] = { { 0 } };//右线
uint16 guide_line[800][2] = { {0},{0}};
uint16 mid_count = 0;
int l_data_statics = 0;
int r_data_statics = 0;
uint8 start_point_l = 0;
uint8 start_point_r = XX;
int start_center_y = 0; //起始行
int start_l_x = 0;
int start_r_x = 0;
int image_flag = 0;
uint32 Longest_WhiteCol[2] = {0};
uint16 Lost_Left[800];
uint16 Lost_Right[800];

uint16 filled_l[YY][2];   // 原始左边界 + 拉线点合并
uint16 filled_r[YY][2];   // 原始右边界 + 拉线点合并
int    filled_l_num = 0;
int    filled_r_num = 0;


// 左右边线局部角度变化
float lpts_angle[800] = { 0 };
float rpts_angle[800] = { 0 };
int lpts_nums = 0;
int rpts_nums = 0;

int Lpt_l[MAX_CORNER];
int Lpt_r[MAX_CORNER];

int Lptl_num = 0;
int Lptr_num = 0;


uint16 center_pts[800][2];
int center_pts_num = 0;

uint16 turn_left = 0;
uint16 turn_right = 0;
uint16 staright;
float turn_angle = 0;
int angle_flag = 0;
int step = 0;

float Error;  // 误差

/***  
 * 函数部分 * 
***/

//绝对值函数
int My_Abs(int value)
{
    if(value>=0) return value;
    else return -value;
}


// 从摄像头抓取灰度图
// 调用示例（放在你的主循环或图像处理函数中）
void crop_img(void)
{
    cv::Mat gray_full;
    cv::cvtColor(uvc_dev.get_frame_mjpg(), gray_full, cv::COLOR_BGR2GRAY);

    cv::Mat flipped;
    cv::flip(gray_full(cv::Rect(0, 40, 160, 60)), flipped, -1);

    for(int row = 0; row < 60; row++)
        memcpy(imggray[row], flipped.ptr(row), 160);
}

//快速大津法获取阈值
uint8 Fast_otsu(uint8 *image)
{
  
   int32_t Pixel_Max = 0;                
   int32_t Pixel_Min = 255;              
   uint16_t width  = XX;    
   uint16_t height = YY;    
   int32_t pixelCount[GrayScale];       
   float pixelPro[GrayScale];
   int32_t i, j;
   int32_t pixelSum = width * height/4;  
   int32_t nowThreshold = 0;            
   unsigned char* data = image;  
   uint32_t gray_sum=0;                 
   
   for (i = 0; i < GrayScale; i++)
   {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
   }
   
   
   for (i = 0; i < height; i+=2)      
   {
        for (j = 0; j < width; j+=2)    
        {
            pixelCount[(int32_t)data[i * width + j]]++; 
            gray_sum+=(int32_t)data[i * width + j];       
            
            if(data[i * width + j]>Pixel_Max) 
            {
               Pixel_Max=data[i * width + j];
            }
            
            if(data[i * width + j]<Pixel_Min)
            {
               Pixel_Min=data[i * width + j]; 
            }
        }
   }
   
   
   for(i = Pixel_Min; i < Pixel_Max; i++)
   {
        pixelPro[i] = (float)pixelCount[i]/pixelSum;
   }
   
    
   float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
   w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
   for (j = 0; j < GrayScale; j++)
   {
        w0 += pixelPro[j];  
        u0tmp += j * pixelPro[j];  
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              
        u1 = u1tmp / w1;              
        u = u0tmp + u1tmp;            
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            nowThreshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
   }

    

    if (nowThreshold < minthreshold)
    {
       nowThreshold = minthreshold;
    }


    else if (nowThreshold > maxthreshold)
    {
       nowThreshold = maxthreshold;
    }
    
    
    return nowThreshold;
    
}


//生成二值化图像
void get_imgotsu(uint8 imgotsu[60][160], int32_t threshold)
{
    int i, j;

    memset(white_col, 0, XX * sizeof(uint32_t)); 
    memset(white_row, 0, YY * sizeof(uint32_t)); 
    memset(imgotsu, Black, YY * XX * sizeof(unsigned char));

    for(i = 0; i < YY; i++)
    {
        for(j = Deal_Left; j < Deal_Right; j++)   // 只处理 5~154
        {
            if(imggray[i][j] >= threshold)
            {
                imgotsu[i][j] = White;
                white_col[j]++;
                white_row[i]++;
            }
            else
            {
                imgotsu[i][j] = Black;
            }
        }

        for(j = 0; j <= Deal_Left - 1; j++)
        {
            imgotsu[i][j] = Black;
        }

        for(j = Deal_Right ; j < XX; j++)
        {
            imgotsu[i][j] = Black;
        }
    }
}

/*** 寻找最长白列 ***/
void Search_LongWhite(void)
{
    int i;
    uint32 max_len = 0;
    uint32 max_idx = 0;

    for(i = 10; i < XX - 10; i++)
    {
        uint32 len = white_col[i];

        if(len > max_len)
        {
            max_len = len;
            max_idx = i;
        }
    }

    Longest_WhiteCol[0] = max_len;
    Longest_WhiteCol[1] = max_idx;
}

void track_left_to_center(uint16 pts_in[][2], int num,
                          uint16 pts_out[][2],
                          int approx_num, float dist)
{
    int max_n = (num > 500) ? 500 : num;

    for (int i = 0; i < max_n; i++)
    {
        int i1 = i - approx_num;
        int i2 = i + approx_num;

        if (i1 < 0) i1 = 0;
        if (i2 >= max_n) i2 = max_n - 1;

        float dx = pts_in[i2][0] - pts_in[i1][0];
        float dy = pts_in[i2][1] - pts_in[i1][1];

        float dn = sqrtf(dx * dx + dy * dy);
        if (dn == 0)
        {
            pts_out[i][0] = pts_in[i][0];
            pts_out[i][1] = pts_in[i][1];
            continue;
        } 

        dx /= dn;
        dy /= dn;

        int x = (int)(pts_in[i][0] - dy * dist);
        int y = (int)(pts_in[i][1] + dx * dist);

        if (x < 0 || x >= XX || y < 0 || y >= YY)
            continue;

        pts_out[i][0] = x;
        pts_out[i][1] = y;
    }
}


void track_right_to_center(uint16 pts_in[][2], int num,
                           uint16 pts_out[][2],
                           int approx_num, float dist)
{
    int max_n = (num > 500) ? 500 : num;

    for (int i = 0; i < max_n; i++)
    {
        int i1 = i - approx_num;
        int i2 = i + approx_num;

        if (i1 < 0) i1 = 0;
        if (i2 >= max_n) i2 = max_n - 1;

        float dx = pts_in[i2][0] - pts_in[i1][0];
        float dy = pts_in[i2][1] - pts_in[i1][1];

        float dn = sqrtf(dx * dx + dy * dy);
        if (dn == 0) 
        {
            pts_out[i][0] = pts_in[i][0];
            pts_out[i][1] = pts_in[i][1];
            continue;
        }

        dx /= dn;
        dy /= dn;


        int x = (int)(pts_in[i][0] + dy * dist);
        int y = (int)(pts_in[i][1] - dx * dist);


        if (x < 0 || x >= XX || y < 0 || y >= YY)
            continue;

        pts_out[i][0] = x;
        pts_out[i][1] = y;
    }
}

/*** 寻找基点 ***/
uint8 Get_Start_Point(void)
{
    int row, col;
    int l_found = 0, r_found = 0;


    for (row = YY - 1 ; row >= YY / 3 ; row--)
    {
        l_found = r_found = 0;
        start_center_y = row;

        /* 左边界 */
        for (col = Longest_WhiteCol[1]; col >= Deal_Left; col--)
        {
            if (imgotsu[row][col] == White &&
                imgotsu[row][col - 1] == Black &&
                imgotsu[row][col - 2] == Black)
            {
                start_l_x = col;

                points_l[0][0] = col;
                points_l[0][1] = row;

                l_found = 1;
                break;
            }
        }

        /* 右边界 */
        for (col = Longest_WhiteCol[1]; col <= Deal_Right; col++)
        {
            if (imgotsu[row][col] == White &&
                imgotsu[row][col + 1] == Black &&
                imgotsu[row][col + 2] == Black)
            {
                start_r_x = col;

                points_r[0][0] = col;
                points_r[0][1] = row;

                r_found = 1;
                break;
            }
        }

        if (l_found && r_found)
        {
            return 1;
        }
    }

    return 0;
}


// ---------------- 点集三角滤波（原地） ----------------
void blur_points_inplace(uint16 pts[][2], int num, int kernel)
{
    assert(kernel % 2 == 1);

    int half = kernel / 2;

    if (num <= 0) return;

    float tmp[num][2];   // 临时存储

    for (int i = 0; i < num; i++)
    {
        tmp[i][0] = tmp[i][1] = 0;

        for (int j = -half; j <= half; j++)
        {
            int idx = i + j;

            if (idx < 0) idx = 0;
            if (idx >= num) idx = num - 1;

            float weight = half + 1 - fabs((float)j);

            tmp[i][0] += pts[idx][0] * weight;
            tmp[i][1] += pts[idx][1] * weight;
        }

        float norm = (2 * half + 2) * (half + 1) / 2.0f;

        tmp[i][0] /= norm;
        tmp[i][1] /= norm;
    }

    // 写回原数组
    for (int i = 0; i < num; i++)
    {
        pts[i][0] = (uint16)(tmp[i][0] + 0.5f);
        pts[i][1] = (uint16)(tmp[i][1] + 0.5f);
    }
}


// ---------------- 点集等距采样（原地） ----------------
void resample_points_inplace(uint16 pts[][2], int *num, float dist)
{
    if (*num <= 0) return;

    float tmp[*num][2];

    int len = 0;
    float remain = 0.f;

    for (int i = 0; i < *num - 1 && len < *num; i++)
    {
        float x0 = pts[i][0];
        float y0 = pts[i][1];

        float dx = pts[i+1][0] - x0;
        float dy = pts[i+1][1] - y0;

        float dn = sqrtf(dx*dx + dy*dy);

        if (dn < 1e-6f) continue;

        dx /= dn;
        dy /= dn;

        while (remain < dn && len < *num)
        {
            x0 += dx * remain;
            y0 += dy * remain;

            tmp[len][0] = x0;
            tmp[len][1] = y0;

            len++;

            dn -= remain;
            remain = dist;
        }

        remain -= dn;
    }

    // 复制回原数组
    for (int i = 0; i < len; i++)
    {
        pts[i][0] = tmp[i][0];
        pts[i][1] = tmp[i][1];
    }

    *num = len;
}

/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */

 // 左手扶墙迷宫法
void search_l(int start_x, int start_y, int *num)
{
    int x = start_x;
    int y = start_y;
    int step = 0;
    int max_steps = *num;
    int dir = 0;   // 方向：0=上,1=右,2=下,3=左
    int turn = 0;

    // 坐标顺序 imgotsu[y][x]
    const int dir_front[4][2]     = {{-1,0},{0,1},{1,0},{0,-1}};    // {dy, dx}
    const int dir_frontleft[4][2] = {{-1,-1},{-1,1},{1,1},{1,-1}}; // {dy, dx}
    
    *num = 0;
    
    while(step < max_steps && x >= 1 && x < XX-2 && y > 0 && y < YY && turn < 4)
    {
        int fy = y + dir_front[dir][0];
        int fx = x + dir_front[dir][1];
        int lfy = y + dir_frontleft[dir][0];
        int lfx = x + dir_frontleft[dir][1];
    
        // 边界检查
        if(fx < 0 || fx >= XX || fy < 0 || fy >= YY) break;
        if(lfx < 0 || lfx >= XX || lfy < 0 || lfy >= YY) break;
    
        if(imgotsu[fy][fx] == Black)        // 前方有墙 → 右转
        {
            dir = (dir + 1) % 4;
            turn++;
        }
        else if(imgotsu[lfy][lfx] == Black) // 前方无墙 + 左前有墙 → 直走
        {
            x = fx;
            y = fy;
            points_l[step][0] = x;
            points_l[step][1] = y;
            step++;
            turn = 0;
        }
        else                                // 左前无墙 → 往左靠
        {
            x = lfx;
            y = lfy;
            dir = (dir + 3) % 4;           // 转向贴墙
            points_l[step][0] = x;
            points_l[step][1] = y;
            step++;
            turn = 0;
        }

    }

    *num = step;

}

// 右手扶墙迷宫法
void search_r(int start_x, int start_y, int *num)
{
    int x = start_x;
    int y = start_y;
    int step = 0;
    int max_steps = *num;
    int dir = 0;   // 方向：0=上,1=右,2=下,3=左
    int turn = 0;

    const int dir_front[4][2]      = {{-1,0},{0,1},{1,0},{0,-1}};     // {dy, dx}
    const int dir_frontright[4][2] = {{-1,1},{1,1},{1,-1},{-1,-1}};  // {dy, dx}
    
    *num = 0;
    
    while(step < max_steps && x >= 1 && x < XX-2 && y > 0 && y < YY && turn < 4)
    {
        int fy = y + dir_front[dir][0];
        int fx = x + dir_front[dir][1];
        int rfy = y + dir_frontright[dir][0];
        int rfx = x + dir_frontright[dir][1];
    
        // 边界检查
        if(fx < 0 || fx >= XX || fy < 0 || fy >= YY) break;
        if(rfx < 0 || rfx >= XX || rfy < 0 || rfy >= YY) break;
    
        if(imgotsu[fy][fx] == Black)        // 前方有墙 → 左转
        {
            dir = (dir + 3) % 4;
            turn++;
        }
        else if(imgotsu[rfy][rfx] == Black) // 前方无墙 + 右前有墙 → 直走
        {
            x = fx;
            y = fy;
            points_r[step][0] = x;
            points_r[step][1] = y;
            step++;
            turn = 0;
        }
        else                                // 右前无墙 → 往右靠
        {
            x = rfx;
            y = rfy;
            dir = (dir + 1) % 4;           // 转向贴墙
            points_r[step][0] = x;
            points_r[step][1] = y;
            step++;
            turn = 0;
        }
    }

    *num = step;

}



#define BORDER_TH        2
#define MIN_TRACK_WIDTH  20

// 统计某一行 y 在点集中是否存在有效点
// 返回该行的 x 坐标，-1 表示无数据
static int get_row_x_from_pts(uint16 pts[][2], int num, int y)
{
    int best = -1;
    for(int i = 0; i < num; i++)
    {
        if(pts[i][1] == y)
            best = pts[i][0];
    }
    return best;
}

/*** 生成引导线  ***/
void Get_Guideline(void)
{
    int mid_count_local = 0;

    if(cross_state == CROSS_IN && ring_ctrl.flag != 1 )
    {
        // ================= 十字中线 =================
        int n = 5;

        if(l_data_statics < n || r_data_statics < n)
        {
            mid_count = 0;
            return;
        }

        int start_lx = 0, start_rx = 0;
        int end_lx = 0, end_rx = 0;

        /* 前几个点 */
        for(int i = 0; i < n; i++)
        {
            start_lx += points_l[i][0];
            start_rx += points_r[i][0];
        }
        start_lx /= n;
        start_rx /= n;

        int start_mid = (start_lx + start_rx) / 2;
        int start_y = (points_l[0][1] + points_r[0][1]) / 2;

        /* 后几个点 */
        for(int i = 0; i < n; i++)
        {
            end_lx += points_l[l_data_statics - 1 - i][0];
            end_rx += points_r[r_data_statics - 1 - i][0];
        }
        end_lx /= n;
        end_rx /= n;

        int end_mid = (end_lx + end_rx) / 2;
        int end_y = (points_l[l_data_statics - 1][1] + points_r[r_data_statics - 1][1]) / 2;

        int dx = end_mid - start_mid;
        int dy = end_y - start_y;
        int steps = abs(dy);
        if(steps == 0) steps = 1;

        mid_count = 0;
        for(int i = 0; i <= steps; i++)
        {
            int y = start_y + i * (dy > 0 ? 1 : -1);
            int x = start_mid + dx * i / steps;

            guide_line[mid_count][0] = x;
            guide_line[mid_count][1] = y;
            mid_count++;
            if(mid_count >= 500) break;
        }
    }
    else if(cross_state != CROSS_IN && ring_ctrl.flag == 1)
    {
        // ================= 环岛中线 =================
        int mid_y[YY];  // 保存每行最新的x
        for(int i = 0; i < YY; i++)
            mid_y[i] = -1;  // -1表示这一行还没有点

        // 遍历 center_pts，从头到尾，后出现的点覆盖前面的
        for(int i = 0; i < center_pts_num; i++)
        {
            int x = center_pts[i][0];
            int y = center_pts[i][1];

            // 边界保护
            if(x < 0) x = 0;
            if(x >= XX) x = XX - 1;
            if(y < 0) y = 0;
            if(y >= YY) y = YY - 1;

            mid_y[y] = x;  // 覆盖同一行的中点
        }

        // 按行从下到上生成 guide_line
        mid_count_local = 0;
        for(int y = YY - 1; y >= 0 && mid_count_local < 500; y--)
        {
            if(mid_y[y] != -1)
            {
                guide_line[mid_count_local][0] = mid_y[y];
                guide_line[mid_count_local][1] = y;
                mid_count_local++;
            }
        }
        mid_count = mid_count_local;
    }
    else
    {
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
                guide_line[mid_count_local][0] = (lx + rx) / 2;
                guide_line[mid_count_local][1] = y;

                mid_count_local++;
                if(mid_count_local >= 500)
                    break;
            }
        }
        mid_count = mid_count_local;
    }
}
    

/***  点集局部角度变化率  ***/
void local_angle_points(uint16 pts_in[][2],int nums,float angle_out[],int dist)
{
     int i;
  
     for(i = 3; i < nums; i++)
     {
        if (i <= 0 || i >= nums - 1) 
        {
            angle_out[i] = 0;
            continue;        
        }

        int idx1 = i - dist;

        if (idx1 < 0) 
        {
            idx1 = 0;
        }

        if (idx1 > nums - 1)
        {
            idx1 = nums - 1;
        }
        

        float dx1 = pts_in[i][0] - pts_in[idx1][0];
        float dy1 = pts_in[i][1] - pts_in[idx1][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);

        int idx2 = i + dist;

        if (idx2 < 0) 
        {
            idx2 = 0;
        }

        if (idx2 > nums - 1)
        {
            idx2 = nums - 1;
        }

        float dx2 = pts_in[idx2][0] - pts_in[i][0];
        float dy2 = pts_in[idx2][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);

        if (dn1 < 1e-3f || dn2 < 1e-3f)
        {
            angle_out[i] = 0.0f;
            continue;
        }

        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;

        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1); 
     }  
}

// ====================== 非极大值抑制 NMS（覆盖原数组） ======================
// angle: 输入输出角度数组（覆盖原值）
// nums: 点数量
// dist: 局部最大值比较半径（用 ANGLE_DIST 即可）
void nms_angle_inplace(float angle[], int nums, int dist)
{
    if (nums <= 0) return;

    float temp[800]; // 临时保存原始值
    for (int i = 0; i < nums; i++)
        temp[i] = angle[i];

    for (int i = 0; i < nums; i++)
    {
        float val = temp[i];

        // 防止非有限值
        if (!isfinite(val)) 
        {
            angle[i] = 0.0f;
            continue;
        }

        int is_max = 1;

        // 局部最大值检查
        for (int j = -dist; j <= dist; j++)
        {
            if (j == 0) continue;

            int idx = i + j;
            if (idx < 0) idx = 0;
            if (idx >= nums) idx = nums - 1;

            if (!isfinite(temp[idx])) continue;

            if (fabs(temp[idx]) > fabs(val))
            {
                is_max = 0;
                break;
            }
        }

        if (!is_max)
            angle[i] = 0.0f;
    }
}


/*** 寻找L角点 ***/
#define CORNER_SAFE 5

void find_corners(void)
{
    int i;

    Lptl_num = 0;
    Lptr_num = 0;

    if (lpts_nums < (ANGLE_DIST * 2 + 1) ||
        rpts_nums < (ANGLE_DIST * 2 + 1))
    {
        return;
    }

    /* ================= 左边线 ================= */
    for (i = ANGLE_DIST; i < lpts_nums - ANGLE_DIST; i++)
    {
        if (!isfinite(lpts_angle[i]))
            continue;

        int x = points_l[i][0];

        /* ---------- 黑框过滤 ---------- */
        if (x <= Deal_Left + CORNER_SAFE ||
            x >= Deal_Right - CORNER_SAFE)
            continue;

        int im1 = i - ANGLE_DIST;
        int ip1 = i + ANGLE_DIST;

        if (!isfinite(lpts_angle[im1]) ||
            !isfinite(lpts_angle[ip1]))
            continue;

        float conf = fabs(lpts_angle[i]);

        if (conf >= 60.0f * Pi / 180.0f &&
            conf <= 140.0f * Pi / 180.0f)
        {
            if (Lptl_num < MAX_CORNER)
            {
                if (Lptl_num == 0 || i - Lpt_l[Lptl_num - 1] > 5)
                {
                    Lpt_l[Lptl_num++] = i;
                }
            }
        }
    }

    /* ================= 右边线 ================= */
    for (i = ANGLE_DIST; i < rpts_nums - ANGLE_DIST; i++)
    {
        if (!isfinite(rpts_angle[i]))
            continue;

        int x = points_r[i][0];

        /* ---------- 黑框过滤 ---------- */
        if (x <= Deal_Left + CORNER_SAFE ||
            x >= Deal_Right - CORNER_SAFE)
            continue;

        int im1 = i - ANGLE_DIST;
        int ip1 = i + ANGLE_DIST;

        if (!isfinite(rpts_angle[im1]) ||
            !isfinite(rpts_angle[ip1]))
            continue;

        float conf = fabs(rpts_angle[i]);

        if (conf >= 60.0f * Pi / 180.0f &&
            conf <= 140.0f * Pi / 180.0f)
        {
            if (Lptr_num < MAX_CORNER)
            {
                if (Lptr_num == 0 || i - Lpt_r[Lptr_num - 1] > 5)
                {
                    Lpt_r[Lptr_num++] = i;
                }
            }
        }
    }
}

#define MIN_CORNER_DIST 5  // 坐标距离小于等于 2 像素视为同一个角点

void remove_duplicate_corners(void)
{
    int i, j;

    // 遍历左边角点
    for(i = 0; i < Lptl_num; i++)
    {
        for(j = 0; j < Lptr_num; j++)
        {
            int xl = points_l[Lpt_l[i]][0];
            int yl = points_l[Lpt_l[i]][1];

            int xr = points_r[Lpt_r[j]][0];
            int yr = points_r[Lpt_r[j]][1];

            if(abs(xl - xr) <= MIN_CORNER_DIST && abs(yl - yr) <= MIN_CORNER_DIST)
            {
                // 左右角点距离过近，视为重复，删除右边角点
                // 把右边数组后面的往前覆盖
                for(int k = j; k < Lptr_num - 1; k++)
                {
                    Lpt_r[k] = Lpt_r[k + 1];
                }
                Lptr_num--; 
                j--; // 删除后当前索引要减一，继续检查
            }
        }
    }
}

#define ERROR_ROW_MIN  20
#define ERROR_ROW_MAX  26
#define CAM_CENTER_X   80

/**  计算误差 **/
float calculate_error(void)
{
    // 对应 y = 30, 31, 32, 33, 34, 35, 36
    const int weight[7] = {1, 2, 3, 3, 3, 2, 1};
    float error_now;

    float sum_wx    = 0.0f;
    int   sum_w     = 0;
    int   row_x[7];

    // 初始化为 -1（表示该行无数据）
    for (int i = 0; i < 7; i++)
        row_x[i] = -1;

    // 遍历引导线，记录每行的 x（同行多点取最后一个，与 Get_Guideline 行为一致）
    for (int i = 0; i < mid_count; i++)
    {
        int x = guide_line[i][0];
        int y = guide_line[i][1];

        if (y >= ERROR_ROW_MIN && y <= ERROR_ROW_MAX)
        {
            row_x[y - ERROR_ROW_MIN] = x;
        }
    }

    // 加权求和
    for (int i = 0; i < 7; i++)
    {
        if (row_x[i] != -1)
        {
            sum_wx += (float)row_x[i] * weight[i];
            sum_w  += weight[i];
        }
    }

    if (sum_w > 0)
    {
        float avg_x = sum_wx / sum_w;
        error_now = avg_x - CAM_CENTER_X;   // 正值 → 偏右，负值 → 偏左
    }
    else
    {
        error_now = 0.0f;  // 该区间无引导线点
    }

    return error_now;
}

void turn(void)
{
    float diff = 0;

    if(turn_left == 1)
    {
        switch(step)
        {
            case 0 :
            {
                encoder_clear();
                turn_angle = angle;
                step = 1;
                break;        
            }

            case 1:
            {
                printf("1\r\n");
                for(int i = 0; i < mid_count; i++)
                {
                    int new_x = (int)guide_line[i][0] - 45;
                    if(new_x >= XX) new_x = XX - 1;       
                    guide_line[i][0] = (uint16)new_x;
                }  

                diff = angle - turn_angle;

                if(distance >= 10000)
                {
                    encoder_clear();
                    turn_angle = angle;   
                    step = 2;
                    break;              
                }
                
                break;       
            }

            case 2:
            {
                printf("2\r\n");
                 diff = angle - turn_angle;

                 if(distance >= 3000 )
                 {
                    turn_left = 0;
                    encoder_clear();
                    printf("out\r\n");
                    break;
                 }
            }

            default:
               break;
        }        
    }
    else if(turn_right)
    {
        switch(step)
        {
            case 0 :
            {
                encoder_clear();
                turn_angle = angle;
                step = 1;
                break;        
            }

            case 1:
            {
                printf("1\r\n");
                for(int i = 0; i < mid_count; i++)
                {
                    int new_x = (int)guide_line[i][0] + 45;
                    if(new_x >= XX) new_x = XX - 1;       
                    guide_line[i][0] = (uint16)new_x;
                }  

                diff = angle - turn_angle;

                if(distance >= 10000)
                {
                    encoder_clear();
                    turn_angle = angle;   
                    step = 2;
                    break;              
                }
                
                break;       
            }

            case 2:
            {
                printf("2\r\n");
                 diff = angle - turn_angle;

                 if(distance >= 3000 )
                 {
                    turn_right = 0;
                    encoder_clear();
                    printf("out\r\n");
                    break;
                 }
            }

            default:
               break;
        }         
    }
}


void deal_image(void)
{
    int16 thres;

    /* 1. 图像裁剪 */
    crop_img();

    /* 2. 二值化 */
    thres = Fast_otsu(&imggray[0][0]);
    get_imgotsu(imgotsu, thres);


    Search_LongWhite();


    /* 3. 起始点检测 */
    if(Get_Start_Point() != 1)
    {
            // 完全找不到赛道 
            l_data_statics = 0;
            r_data_statics = 0;
            mid_count = 0;
    }
    else
    {
        /* ================= 正常巡线 ================= */
        
         l_data_statics = 799;
         r_data_statics = 799;

         search_l(start_l_x, start_center_y, &l_data_statics);
         search_r(start_r_x, start_center_y, &r_data_statics);

        //  blur_points_inplace(points_l,l_data_statics,5);
        //  blur_points_inplace(points_r,r_data_statics,5);

        //  resample_points_inplace(points_l,&l_data_statics,sample_dist * pixel_per_meter);
        //  resample_points_inplace(points_r,&r_data_statics,sample_dist * pixel_per_meter);

         lpts_nums = l_data_statics;
         rpts_nums = r_data_statics;

         local_angle_points(points_l, lpts_nums, lpts_angle, ANGLE_DIST);
         local_angle_points(points_r, rpts_nums, rpts_angle, ANGLE_DIST);

         nms_angle_inplace(lpts_angle, lpts_nums, ANGLE_DIST);
         nms_angle_inplace(rpts_angle, rpts_nums, ANGLE_DIST);

         find_corners();       
         remove_duplicate_corners();
    }

    // check_ring();

    // run_ring();

    Get_Guideline();

    // turn();
}