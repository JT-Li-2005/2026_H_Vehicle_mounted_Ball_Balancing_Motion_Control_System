#include "zf_common_headfile.h"

/*******************************************************************************
 * 传感器值 (Five_Gray.c 的 Gray_scan 更新)
 * 排列 (左→右): L2, L1, M, R1, R2  黑色=1, 白色=0
 ******************************************************************************/
extern int level_1, level_2, level_3, level_4, level_5;

/* ========================= 模式参数结构体 ========================= */
typedef struct
{
    int   base_pwm;      // 基础速度PWM
    int   kp;            // PID比例
    int   ki;            // PID积分
    int   kd;            // PID微分
    int   i_lim;         // 积分限幅
    float lost_gain;     // 丢线转向增益
    int   lost_max;      // 最大丢线周期
} TrackConfig;

/* ========================= 4组预设模式 ========================= */
static const TrackConfig CFG[4] =
{
    //  Mode 1: 2小问 (适合调试/窄弯)
    {
        .base_pwm  = 4000,
        .kp        = 950,
        .ki        = 10,
        .kd        = 2200,
        .i_lim     = 2000,
        .lost_gain = 1.5f,
        .lost_max  = 200,
    },
    //  Mode 2: 4小问 (日常巡线)
    {
        .base_pwm  = 2800,
        .kp        = 520,
        .ki        = 5,
        .kd        = 1700,
        .i_lim     = 3000,
        .lost_gain = 1.8f,
        .lost_max  = 200,
    },
    //  Mode 3: 快速 (高速赛道)
    {
        .base_pwm  = 6000,
        .kp        = 2200,
        .ki        = 40,
        .kd        = 4000,
        .i_lim     = 3500,
        .lost_gain = 2.0f,
        .lost_max  = 200,
    },
    //  Mode 4: 冲刺 (极速)
    {
        .base_pwm  = 7000,
        .kp        = 2600,
        .ki        = 50,
        .kd        = 4600,
        .i_lim     = 4000,
        .lost_gain = 2.2f,
        .lost_max  = 200,
    },
};

/* ========================= 启停检测参数 ========================= */
#define STOP_SENSORS  4          // ≥4黑即判定停止线 (兼容11111/11110/01111)
// 上升沿检测 + 无消抖, 即刻响应

/* ========================= 误差滤波参数 ========================= */
#define ERR_SMOOTH     30        // EMA平滑系数 (30 = 30%新值 + 70%历史, 值越小越平滑)

/* ========================= 计时器 (PIT 1ms) ========================= */
#define STOPWATCH_PIT  PIT_TIM_G6       // 空闲G6通道, 1ms定时中断

static volatile uint32 tick_ms;          // 毫秒累加器 (PIT中断更新)
static          uint32 last_time;        // 最后一圈的用时(ms), 停止时保存
static          uint8  timing;           // 1=正在计时, 0=已停止

// PIT 1ms 中断回调: 每1ms累加一次
static void StopWatch_IRQ(uint32 event, void *ptr)
{
    (void)event; (void)ptr;
    tick_ms++;
}

// 初始化计时器
static void StopWatch_Init(void)
{
    tick_ms   = 0;
    last_time = 0;
    timing    = 0;
    pit_ms_init(STOPWATCH_PIT, 1, StopWatch_IRQ, NULL);
}

// 启动/重置计时 (车开始跑)
static void StopWatch_Start(void)
{
    tick_ms = 0;
    timing  = 1;
}

// 停止计时, 保存当前时间 (车停下)
static void StopWatch_Stop(void)
{
    timing    = 0;
    last_time = tick_ms;
}

// 获取当前时间 (计时中返回实时值, 已停止返回最终值)
static uint32 StopWatch_Get_Time(void)
{
    return timing ? tick_ms : last_time;
}

/* ========================= 状态变量 ========================= */
static int   prev_err;           // 上一帧平滑误差, 用于 D项
static int   prev_raw;           // 上一帧原始误差, D项用原始差分快速阻尼
static int   valid_err;          // 最后有效误差, 专用于丢线恢复
static int   integral;           // PID积分
static int   lost_cnt;           // 丢线计数
static int   smooth_err;         // EMA滤波后的平滑误差
static uint8 run;                // 0=停 1=跑
static uint8 on_stop;            // 已触发停止标记 (防止反复切换)
static uint8 prev_black;         // 上一帧黑点数, 用于上升沿检测
static uint8 mode;               // 当前模式 0~3

/* ========================= 工具函数 ========================= */
static uint8 all_white(void) { return !level_1&&!level_2&&!level_3&&!level_4&&!level_5; }

// 统计黑色(高电平)传感器数量
static uint8 black_cnt(void)
{
    return level_1 + level_2 + level_3 + level_4 + level_5;
}

// 纯计算: 位置误差 ×100, 权重 L2=-2 L1=-1 M=0 R1=+1 R2=+2
static int calc_pos(void)
{
    int w = 0, n = 0;
    if(level_1 == 1){ w += -2; n++; }
    if(level_2 == 1){ w += -1; n++; }
    if(level_3 == 1){ w +=  0; n++; }
    if(level_4 == 1){ w +=  1; n++; }
    if(level_5 == 1){ w +=  2; n++; }
    return (n > 0) ? (w * 100) / n : 0;
}

/* ========================= 对外接口 ========================= */
void Track_Init(void)
{
    prev_err = 0; valid_err = 0; integral = 0; smooth_err = 0; prev_raw = 0;
    lost_cnt = 0; run = 0; on_stop = 0; prev_black = 0;
    mode = 1;   // 默认Mode 2 (中速)
    key_init(10);   // 按键初始化, 10ms扫描周期
    StopWatch_Init();   // 计时器初始化 (PIT 1ms)
}

// 获取当前模式编号 (1~4)
uint8 Track_Get_Mode(void)
{
    return mode + 1;
}

// 独立于 Track_Process 的模式选择, 任何时刻都可调用
void Track_Mode_Select(void)
{
    key_scanner();      // 按键扫描 (每帧更新)

    if(key_get_state(KEY_1) == KEY_SHORT_PRESS) { key_clear_state(KEY_1); mode = 0; prev_err = 0; valid_err = 0; integral = 0; smooth_err = 0; prev_raw = 0; }
    if(key_get_state(KEY_2) == KEY_SHORT_PRESS) { key_clear_state(KEY_2); mode = 1; prev_err = 0; valid_err = 0; integral = 0; smooth_err = 0; prev_raw = 0; }
    if(key_get_state(KEY_3) == KEY_SHORT_PRESS) { key_clear_state(KEY_3); mode = 2; prev_err = 0; valid_err = 0; integral = 0; smooth_err = 0; prev_raw = 0; }
    if(key_get_state(KEY_4) == KEY_SHORT_PRESS) { key_clear_state(KEY_4); mode = 3; prev_err = 0; valid_err = 0; integral = 0; smooth_err = 0; prev_raw = 0; }
}

void Track_Process(void)
{
    Gray_scan();

    // ============================================================
    //  启停检测 (上升沿触发, 即刻响应)
    //  上一帧非全黑 → 当前帧全黑(11111) → 立即 toggle
    // ============================================================
    uint8 cur_black = black_cnt();

    // 上升沿: 从非全黑跳变到全黑, 且本轮未触发过 → 立刻动作
    if(cur_black >= STOP_SENSORS && prev_black < STOP_SENSORS && !on_stop) {
        on_stop = 1;
        run = !run;
        if(!run) { Motor_Stop(); StopWatch_Stop();   }   // 车停 → 保存时间
        else     {                  StopWatch_Start(); }   // 车跑 → 清零计时
        prev_err = 0; valid_err = 0; integral = 0; smooth_err = 0; prev_raw = 0;
    }

    // 离开全黑区域后, 允许下次触发
    if(cur_black < STOP_SENSORS) {
        on_stop = 0;
    }

    prev_black = cur_black;

    if(!run) return;

    // ---- 获取当前模式的参数引用 ----
    const TrackConfig *cfg = &CFG[mode];

    // --- 计算当前误差 & 丢线判断 ---
    int   raw  = calc_pos();                       // 原始离散误差
    smooth_err = (ERR_SMOOTH * raw + (100 - ERR_SMOOTH) * smooth_err) / 100;  // EMA平滑
    int   err  = smooth_err;                       // 使用平滑后的误差做PID
    uint8 lost = all_white();

    // ==================== PID ====================
    int p_out, i_out, d_out, offset;

    if(lost) {
        // ---- 丢线: 沿上次有效方向扫回, D=0, I 冻结 ----
        lost_cnt++;
        if(lost_cnt > cfg->lost_max) { Motor_Stop(); run = 0; return; }

        p_out = (cfg->kp * valid_err) / 100;
        i_out = integral;           // 积分冻结, 不继续累加
        d_out = 0;                  // 丢线时误差不变, 微分无意义
        offset = (int)((p_out + i_out) * cfg->lost_gain);

    } else {
        // ---- 正常: 完整 PID, 更新历史状态 ----
        lost_cnt   = 0;
        valid_err  = raw;           // 保存原始误差, 供丢线恢复

        // P
        p_out = (cfg->kp * err) / 100;

        // I: 积分累加 + 抗饱和
        integral += (cfg->ki * err) / 100;
        if(integral >  cfg->i_lim) integral =  cfg->i_lim;
        if(integral < -cfg->i_lim) integral = -cfg->i_lim;
        i_out = integral;

        // D: 用原始误差差分做快速阻尼, 不受滤波延迟影响
        d_out = (cfg->kd * (raw - prev_raw)) / 100;
        prev_raw  = raw;
        prev_err  = err;

        offset = p_out + i_out + d_out;
    }

    // --- PWM 映射 ---
    int L = cfg->base_pwm + offset;
    int R = cfg->base_pwm - offset;

    if(L >  10000) L =  10000;  if(L < -10000) L = -10000;
    if(R >  10000) R =  10000;  if(R < -10000) R = -10000;

    Motor_Set(L, R);
}

//=============================================================================
// 在IPS200上显示当前模式和参数
//   Row 9:  Mode: X   BASE:XXXX  KP:XXXX  KD:XXXX
//   Row 10: KI:XX  I_LIM:XXXX  GAIN:X.X
//=============================================================================
void Track_Mode_Show(void)
{
    const TrackConfig *cfg = &CFG[mode];

    // Row 9: 模式 + 核心参数
    ips200_show_string(0,      16 * 9, "Mode:");
    ips200_show_int   (40,     16 * 9, mode + 1, 1);

    ips200_show_string(56,     16 * 9, "BASE:");
    ips200_show_int   (96,     16 * 9, cfg->base_pwm, 4);

    ips200_show_string(136,    16 * 9, "KP:");
    ips200_show_int   (160,    16 * 9, cfg->kp, 4);

    ips200_show_string(200,    16 * 9, "KD:");
    ips200_show_int   (224,    16 * 9, cfg->kd, 4);

    // Row 10
    ips200_show_string(0,      16 * 10, "KI:");
    ips200_show_int   (24,     16 * 10, cfg->ki, 3);

    ips200_show_string(56,     16 * 10, "I_LIM:");
    ips200_show_int   (104,    16 * 10, cfg->i_lim, 4);

    ips200_show_string(144,    16 * 10, "GAIN:");
    ips200_show_float(184,     16 * 10, cfg->lost_gain, 1, 1);
}

//=============================================================================
// 在IPS200上显示计时器
//   Row 8:  TIME: M:SS.ms
//=============================================================================
void StopWatch_Show(void)
{
    uint32 t  = StopWatch_Get_Time();
    uint32 ms = t % 1000;
    uint32 s  = (t / 1000) % 60;
    uint32 m  = t / 60000;

    ips200_show_string(0,      16 * 8, "TIME:");
    ips200_show_int   (40,     16 * 8, m,  2);
    ips200_show_string(56,     16 * 8, ":");
    ips200_show_int   (64,     16 * 8, s,  2);
    ips200_show_string(80,     16 * 8, ".");
    ips200_show_int   (88,     16 * 8, ms, 3);

    if(timing) {
        ips200_show_string(120, 16 * 8, "RUN ");
    } else if(t > 0) {
        ips200_show_string(120, 16 * 8, "STOP");
    }
}
