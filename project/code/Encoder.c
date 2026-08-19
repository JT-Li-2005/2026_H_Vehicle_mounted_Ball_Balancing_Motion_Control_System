#include "Encoder.h"

/*******************************************************************************
 * 编码器脉冲计数 (A相上升沿触发, B相判断方向)
 ******************************************************************************/
static volatile int32 left_encoder_cnt  = 0;    // 左轮编码器累计脉冲数
static volatile int32 right_encoder_cnt = 0;    // 右轮编码器累计脉冲数

/*******************************************************************************
 * 速度值 (定期计算更新)
 ******************************************************************************/
static volatile float left_speed_rpm  = 0.0f;   // 左轮速度(RPM)
static volatile float right_speed_rpm = 0.0f;   // 右轮速度(RPM)

/*******************************************************************************
 * 上次计算时的脉冲计数值 (用于差分计算)
 ******************************************************************************/
static int32 last_left_cnt  = 0;
static int32 last_right_cnt = 0;

/*******************************************************************************
 * 每个测速周期的脉冲增量 (实时编码器输出, 可与理论PWM对比)
 ******************************************************************************/
static volatile int32 left_encoder_delta  = 0;   // 左轮单周期脉冲增量
static volatile int32 right_encoder_delta = 0;   // 右轮单周期脉冲增量



//=============================================================================
// 左编码器A相中断回调: 上升沿时读取B相电平判断方向
//=============================================================================
static void Left_Encoder_IRQ(uint32 event, void *ptr)
{
    if(event & EXTI_TRIGGER_RISING)
    {
        if(gpio_get_level(LEFT_ENCODER_B))           // B相高电平 → 正转
            left_encoder_cnt++;
        else                                         // B相低电平 → 反转
            left_encoder_cnt--;
    }
}

//=============================================================================
// 右编码器A相中断回调: 上升沿时读取B相电平判断方向
//=============================================================================
static void Right_Encoder_IRQ(uint32 event, void *ptr)
{
    if(event & EXTI_TRIGGER_RISING)
    {
        if(gpio_get_level(RIGHT_ENCODER_B))          // B相高电平 → 正转
            right_encoder_cnt++;
        else                                         // B相低电平 → 反转
            right_encoder_cnt--;
    }
}

//=============================================================================
// 速度计算定时器回调: 每10ms根据脉冲增量计算RPM
//   RPM = (delta_pulses * 60 * 1000) / (ENCODER_RES * SPEED_INTERVAL_MS)
//=============================================================================
static void Speed_Calc_IRQ(uint32 event, void *ptr)
{
    int32 delta_left;
    int32 delta_right;

    // 计算脉冲增量
    delta_left  = left_encoder_cnt  - last_left_cnt;
    delta_right = right_encoder_cnt - last_right_cnt;

    last_left_cnt  = left_encoder_cnt;
    last_right_cnt = right_encoder_cnt;

    // 保存脉冲增量, 供外部读取 (实时编码器输出)
    left_encoder_delta  = delta_left;
    right_encoder_delta = delta_right;

    // 转换为RPM: 脉冲增量 / 每圈脉冲数 * 每分钟圈数
    left_speed_rpm  = (float)delta_left  * 60000.0f / (ENCODER_RES * SPEED_INTERVAL_MS);
    right_speed_rpm = (float)delta_right * 60000.0f / (ENCODER_RES * SPEED_INTERVAL_MS);
}

//=============================================================================
// 编码器初始化
//=============================================================================
void Encoder_Init(void)
{
    // ---- 1. B相引脚初始化为上拉输入 (只需读取电平, 不需要中断) ----
    //     注意: A相引脚由 exti_init 内部自动初始化 (GPI_PULL_UP)
    gpio_init(LEFT_ENCODER_B,  GPI, 0, GPI_PULL_UP);
    gpio_init(RIGHT_ENCODER_B, GPI, 0, GPI_PULL_UP);

    // ---- 2. 配置A相上升沿中断 (exti_init 会自动将引脚初始化为 GPI_PULL_UP) ----
    //     仅A相接中断用于脉冲计数, B相在ISR中读取电平判断方向
    exti_init(LEFT_ENCODER_A,  EXTI_TRIGGER_RISING, Left_Encoder_IRQ,  NULL);
    exti_init(RIGHT_ENCODER_A, EXTI_TRIGGER_RISING, Right_Encoder_IRQ, NULL);

    // ---- 3. 配置PIT定时器用于速度计算 (10ms周期) ----
    //     使用 PIT_TIM_G12 (避免与PWM使用的TIMG8冲突)
    pit_ms_init(PIT_TIM_G12, SPEED_INTERVAL_MS, Speed_Calc_IRQ, NULL);
}

//=============================================================================
// 获取左右轮速度值
//=============================================================================
void Encoder_Get_Speed(int16 *left_rpm, int16 *right_rpm)
{
    *left_rpm  = (int16)left_speed_rpm;
    *right_rpm = (int16)right_speed_rpm;
}

//=============================================================================
// 获取左轮速度(RPM)
//=============================================================================
int16 Encoder_Get_Left_Speed(void)
{
    return (int16)left_speed_rpm;
}

//=============================================================================
// 获取右轮速度(RPM)
//=============================================================================
int16 Encoder_Get_Right_Speed(void)
{
    return (int16)right_speed_rpm;
}

//=============================================================================
// 获取左轮单个测速周期的脉冲增量 (实时编码器输出, 等效实际PWM)
//=============================================================================
int32 Encoder_Get_Left_Delta(void)
{
    return left_encoder_delta;
}

//=============================================================================
// 获取右轮单个测速周期的脉冲增量 (实时编码器输出, 等效实际PWM)
//=============================================================================
int32 Encoder_Get_Right_Delta(void)
{
    return right_encoder_delta;
}

//=============================================================================
// 获取左轮编码器累计脉冲数
//=============================================================================
int32 Encoder_Get_Left_Cnt(void)
{
    return left_encoder_cnt;
}

//=============================================================================
// 获取右轮编码器累计脉冲数
//=============================================================================
int32 Encoder_Get_Right_Cnt(void)
{
    return right_encoder_cnt;
}

//=============================================================================
// 在IPS200上显示编码器实时脉冲增量和速度
//   Delta = 每个SPEED_INTERVAL_MS周期内的脉冲增量, 即编码器实测输出
//   可与 Motor_PWM_Show 的理论PWM对比, 评估实际输出与理论值的偏差
// 显示布局:  (8x16字体, 320x240 → 40列×15行)
//   Row 3:   "L-Pls: xxxxx   Speed: xxxxx RPM"
//   Row 5:   "R-Pls: xxxxx   Speed: xxxxx RPM"
//=============================================================================
void Encoder_Show(void)
{
    int32 l_delta, r_delta;
    int16 l_speed, r_speed;

    // 获取编码器实时数据
    l_delta = Encoder_Get_Left_Delta();
    r_delta = Encoder_Get_Right_Delta();
    l_speed = Encoder_Get_Left_Speed();
    r_speed = Encoder_Get_Right_Speed();

    // ---- 左轮数据 ----
    ips200_show_string(8 * 0, 16 * 1, "L-Pls:");
    ips200_show_int   (8 * 6, 16 * 1, l_delta, 5);
    ips200_show_string(8 * 0, 16 * 2, "Speed:");
    ips200_show_int   (8 * 7, 16 * 2, l_speed, 5);
    ips200_show_string(8 * 11, 16 * 2, "RPM");

    // ---- 右轮数据 ----
    ips200_show_string(8 * 16, 16 * 1, "R-Pls:");
    ips200_show_int   (8 * 22, 16 * 1, r_delta, 5);
    ips200_show_string(8 * 16, 16 * 2, "Speed:");
    ips200_show_int   (8 * 23, 16 * 2, r_speed, 5);
    ips200_show_string(8 * 27, 16 * 2, "RPM");
}
