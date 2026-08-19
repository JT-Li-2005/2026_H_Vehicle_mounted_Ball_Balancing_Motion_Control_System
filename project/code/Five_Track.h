#include "zf_common_headfile.h"

void  Track_Init(void);          // 循迹初始化
void  Track_Mode_Select(void);   // 模式选择(按键独立, 任何时刻可调用)
void  Track_Process(void);       // 循迹主逻辑 (while循环中调用)
uint8 Track_Get_Mode(void);      // 获取当前模式 1~4
void  Track_Mode_Show(void);     // 在IPS200显示当前模式和参数
void  StopWatch_Show(void);      // 在IPS200显示计时器 (M:SS.ms)
