#ifndef __MENU_HPP__
#define __MENU_HPP__

#include "zf_common_headfile.hpp"

// 外部屏幕对象
extern zf_device_ips200 ips200;

// 接口函数
void ips200_show_edge_points(uint16 x0, uint16 y0);
void ips200_show_guideline(uint16 x0, uint16 y0);
void ips200_show_corners(uint16 x0, uint16 y0);
void ips200_show_corner_num(int Lptl_num, int Lptr_num);
void debug_show_all(void);

#endif