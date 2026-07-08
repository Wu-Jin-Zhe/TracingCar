#include "deal_img.hpp"
#include "menu.hpp"

zf_device_ips200 ips200;

// ================= 屏幕分辨率（写死最稳） =================
#define LCD_W 240
#define LCD_H 320

// ================= 显示偏移 =================
#define IMG_DISP_X   ((320 - XX) / 2)
#define IMG_DISP_Y   20

// ================= 安全画点（防负数炸显存） =================
#define DRAW_SAFE(x,y,color) \
    do{ \
        int _x = (x); \
        int _y = (y); \
        if(_x >= 0 && _x < LCD_W && _y >= 0 && _y < LCD_H) \
            ips200.draw_point(_x,_y,color); \
    }while(0)

// ================= 边界 =================
void ips200_show_edge_points(uint16 x0, uint16 y0)
{
    for(int i = 0; i < l_data_statics - 1; i++)
    {
        DRAW_SAFE(x0 + points_l[i][0], y0 + points_l[i][1], RGB565_RED);
    }

    for(int i = 0; i < r_data_statics - 1; i++)
    {
        DRAW_SAFE(x0 + points_r[i][0], y0 + points_r[i][1], RGB565_GREEN);
    }
}

// ================= 引导线 =================
void ips200_show_guideline(uint16 x0, uint16 y0)
{
    for(int i = 0; i < mid_count; i++)
    {
        DRAW_SAFE(x0 + guide_line[i][0], y0 + guide_line[i][1], RGB565_BLUE);
    }
}

// ================= 二值图 =================
void ips200_show_bin_image(uint16 x0, uint16 y0)
{
    for(int y = 0; y < YY; y++)
    {
        for(int x = 0; x < XX; x++)
        {
            if(imgotsu[y][x] == White)
                DRAW_SAFE(x0 + x, y0 + y, RGB565_WHITE);
            else
                DRAW_SAFE(x0 + x, y0 + y, RGB565_BLACK);
        }
    }
}

// ================= 粗十字 =================
static void draw_big_cross(int cx, int cy, uint16 color)
{
    for(int dx = -4; dx <= 4; dx++)
    {
        DRAW_SAFE(cx + dx, cy, color);
        DRAW_SAFE(cx + dx, cy - 1, color);
        DRAW_SAFE(cx + dx, cy + 1, color);
    }

    for(int dy = -4; dy <= 4; dy++)
    {
        DRAW_SAFE(cx, cy + dy, color);
        DRAW_SAFE(cx - 1, cy + dy, color);
        DRAW_SAFE(cx + 1, cy + dy, color);
    }
}

// ================= 角点 =================
void ips200_show_corners(uint16 x0, uint16 y0)
{
    for(int i = 0; i < Lptl_num; i++)
    {
        int idx = Lpt_l[i];
        draw_big_cross(x0 + points_l[idx][0],
                       y0 + points_l[idx][1],
                       RGB565_BLUE);
    }

    for(int i = 0; i < Lptr_num; i++)
    {
        int idx = Lpt_r[i];
        draw_big_cross(x0 + points_r[idx][0],
                       y0 + points_r[idx][1],
                       RGB565_BLUE);
    }
}

// ================= 数量显示 =================
void ips200_show_corner_num(int Lptl_num, int Lptr_num)
{
    ips200.show_string(180, 0,  "Lpt:");
    ips200.show_int   (210, 0,  Lptl_num, 2);

    ips200.show_string(180, 20, "Rpt:");
    ips200.show_int   (210, 20, Lptr_num, 2);
}

// ================= 总调试 =================
void debug_show_all(void)
{
    ips200_show_bin_image(0, 0);
    ips200_show_edge_points(0, 0);
    ips200_show_guideline(0, 0);
    ips200_show_corners(0, 0);

    ips200_show_corner_num(Lptl_num, Lptr_num);
}