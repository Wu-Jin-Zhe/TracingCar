#ifndef __DEAL_IMG_HPP__
#define __DEAL_IMG_HPP__

#pragma once

#include "zf_common_headfile.hpp"
#include <cstdint>

#ifdef __cplusplus
#include <mutex>  // C++ STL 必须在 extern "C" 外
extern "C" {
#endif

// =================== 常量 ===================
#define FAR_POINTS_MAX_LEN  600
#define Pi            3.14159265358979f
#define XX            160
#define YY            60
#define GrayScale     256
#define maxthreshold  190
#define minthreshold  70
#define White         255
#define Black         0
#define Deal_Left     5             
#define Deal_Right    XX-5          
#define Deal_Bottom   YY-5             
#define Deal_Top      5          
#define USE_num       YY*4
#define ANGLE_DIST    5
#define MAX_CORNER    10

// =================== 全局变量 ===================
extern zf_device_uvc uvc_dev; 
extern uint8 imgotsu[60][160];   //二值化图像
extern uint8 imggray[60][160];   //灰度图像 
extern uint32 Longest_WhiteCol[2]; 
extern float Error;
extern uint16 points_l[800][2];//左线
extern uint16 points_r[800][2];//右线
extern uint16 guide_line[800][2];
extern int l_data_statics;
extern int r_data_statics;
extern uint16 mid_count;
extern int start_center_y; //起始行
extern int start_l_x;
extern int start_r_x;
extern int image_flag;

extern int lpts_nums;
extern int rpts_nums;
extern float lpts_angle[800];
extern float rpts_angle[800] ;

extern int Lpt_l[MAX_CORNER];
extern int Lpt_r[MAX_CORNER];

extern int Lptl_num ;
extern int Lptr_num ;

extern uint16 center_pts[800][2];
extern int center_pts_num;
extern uint16 turn_left;
extern uint16 turn_right;

// =================== 函数声明 ===================
void crop_img(void);
int My_Abs(int value);
uint8 Fast_otsu(uint8 *image);
void get_imgotsu(uint8 imgotsu[60][160],int32_t threshold);
uint8 Get_Start_Point(void);
void search_l(int x,int y,int *num);
void search_r(int x,int y, int *num);
void blur_points_inplace(uint16 pts[][2], int num, int kernel);
void resample_points_inplace(uint16 pts[][2], int *num, float dist);
void deal_image(void);
void Get_Guideline(void);
float calculate_error(void);
void local_angle_points(uint16 pts_in[][2],int nums,float angle_out[],int dist);
void find_corners(void);
void track_left_to_center(uint16 pts_in[][2], int num,
                          uint16 pts_out[][2],
                          int approx_num, float dist);
void track_right_to_center(uint16 pts_in[][2], int num,
                           uint16 pts_out[][2],
                           int approx_num, float dist);
void nms_angle_inplace(float angle[], int nums, int dist);

#ifdef __cplusplus
} // extern "C"

// =================== C++ 专用变量 ===================
extern std::mutex frame_mutex;

#endif // __cplusplus

#endif // __DEAL_IMG_H__