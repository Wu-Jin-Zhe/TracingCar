#ifdef __cplusplus
#include <mutex>
#endif

#include "zf_common_headfile.hpp"
#include <cstdint>

// C 兼容部分
typedef enum
{
    RING_NONE = 0,
    RING_LEFT,
    RING_RIGHT,
} ring_types;

typedef enum
{
    Ring_state1,
    Ring_state2,
    Ring_state3,
    Ring_state4,
    Ring_state5,
    Ring_state6,
    Ring_state7,
    Ring_state8,
    Ring_state9,
} ring_state;

// 这里是 C++ 特性：uint32_t 可以用在 struct 里
typedef struct 
{
    ring_state state;
    ring_types type;
    uint8 phase_counter;
    float sum_zangle;
    uint8 last_high;
    uint32 time_counter;  // ✅ 改为标准类型
    uint8 flag;
    uint16 start_x;
    uint16 start_y;
    uint16 Lpt;
    uint16 Lpt_Found;
    uint16 Lpt_x;
    uint16 Lpt_y;
    uint16 patch_Lpt;    // ⭐ 新增：State1补线用的角点索引（左边界）
    uint16 patch_Lpt_x;
    uint16 patch_Lpt_y;    
    float  patch_ux;   // 补线方向 x 分量（一次性固定）
    float  patch_uy;   // 补线方向 y 分量（一次性固定） 
    uint8  patch_active; 
    uint16 patch_Lpt_r;
    uint16 patch_Lpt_r_x;
    uint16 patch_Lpt_r_y;
    float  patch_ux_r;
    float  patch_uy_r;
    uint8  patch_active_r;

} RingState;

#ifdef __cplusplus
extern "C" {
#endif
extern int time_flag;
extern RingState ring_ctrl;

int check_boundary_monotone(uint16_t pts[][2], int num);
void check_ring(void);
void run_ring(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern std::mutex frame_mutex;
#endif