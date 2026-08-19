#define L_PWM   PWM_TIM_G8_CH1_A27
#define L_DIR   B11
#define R_PWM   PWM_TIM_G8_CH0_A26
#define R_DIR   B10

extern int g_left_pwm;    // 左轮当前PWM值 (Motor_Set更新)
extern int g_right_pwm;   // 右轮当前PWM值 (Motor_Set更新)

void Motor_Init(void);
void Motor_Left(int pwm);
void Motor_Right(int pwm);
void Motor_Set(int left_pwm,int right_pwm);
void Motor_Stop(void);
void Motor_PWM_Show(void);