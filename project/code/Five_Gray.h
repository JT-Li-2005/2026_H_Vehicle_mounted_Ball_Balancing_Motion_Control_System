#define L2    B16
#define L1    B15
#define M     B14
#define R1    A0
#define R2    A1//A12 A9引脚冲突

extern int level_1, level_2, level_3, level_4, level_5;

void Gray_sensor_init(void);
void Gray_scan(void);
void Gray_text(void);
float Gray_GetError(void);