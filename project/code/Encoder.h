#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "zf_common_headfile.h"

/*******************************************************************************
 * 编码器引脚定义 (可根据实际接线修改)
 * WHEELTEC电机编码器接线: C1(黄色)=A相, C2(白色)=B相
 ******************************************************************************/
#define LEFT_ENCODER_A      B27         // 左轮编码器A相 (黄色), 用于脉冲计数中断
#define LEFT_ENCODER_B      B26         // 左轮编码器B相 (白色), 用于方向判断
#define RIGHT_ENCODER_A     B8         // 右轮编码器A相 (黄色), 用于脉冲计数中断
#define RIGHT_ENCODER_B     B9         // 右轮编码器B相 (白色), 用于方向判断

/*******************************************************************************
 * 编码器参数 (请根据实际电机型号修改减速比)
 ******************************************************************************/
#define ENCODER_PPR         13          // 编码器线数(电机轴每圈脉冲数, 霍尔编码器通常13线)
#define GEAR_RATIO          30.0f       // 减速比 (MG310为30, 请根据实际型号修改)
#define ENCODER_RES         390         // 输出轴每圈脉冲数 = PPR * 减速比 = 13 * 30

/*******************************************************************************
 * 测速参数
 ******************************************************************************/
#define SPEED_INTERVAL_MS   10          // 速度计算周期(ms), 越小响应越快但精度越低

/*******************************************************************************
 * API函数声明
 ******************************************************************************/
void Encoder_Init(void);                                      // 编码器初始化
void Encoder_Get_Speed(int16 *left_rpm, int16 *right_rpm);  // 获取左右轮速度(RPM)
int16 Encoder_Get_Left_Speed(void);                           // 获取左轮速度(RPM)
int16 Encoder_Get_Right_Speed(void);                          // 获取右轮速度(RPM)
int32 Encoder_Get_Left_Delta(void);                           // 获取左轮单周期脉冲增量 (等效实际PWM)
int32 Encoder_Get_Right_Delta(void);                          // 获取右轮单周期脉冲增量 (等效实际PWM)
int32 Encoder_Get_Left_Cnt(void);                             // 获取左轮累计脉冲数
int32 Encoder_Get_Right_Cnt(void);                            // 获取右轮累计脉冲数
void Encoder_Show(void);                                      // 在IPS200显示实时脉冲增量+速度, 用于与理论PWM对比

#endif /* _ENCODER_H_ */
