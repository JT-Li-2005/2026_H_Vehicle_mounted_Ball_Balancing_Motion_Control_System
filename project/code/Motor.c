#include "zf_common_headfile.h"
int g_left_pwm = 0;
int g_right_pwm = 0;
//---------------------------------------------------电机初始化
void Motor_Init(void)
{
    gpio_init(L_DIR,GPO,0,GPO_PUSH_PULL);
    gpio_init(R_DIR,GPO,0,GPO_PUSH_PULL);

    pwm_init(L_PWM,17000,0);
    pwm_init(R_PWM,17000,0);
}

//---------------------------------------------------左电机
void Motor_Left(int pwm)
{
    if(pwm>=0)
    {
        gpio_set_level(L_DIR,1);
    }
    else
    {
        gpio_set_level(L_DIR,0);
        pwm=-pwm;
    }

    if(pwm>10000) pwm=10000;

    pwm_set_duty(L_PWM,pwm);
}

//---------------------------------------------------右电机
void Motor_Right(int pwm)
{
    if(pwm>=0)
    {
        gpio_set_level(R_DIR,1);
    }
    else
    {
        gpio_set_level(R_DIR,0);
        pwm=-pwm;
    }

    if(pwm>10000) pwm=10000;

    pwm_set_duty(R_PWM,pwm);
}

//---------------------------------------------------双轮控制
void Motor_Set(int left_pwm,int right_pwm)
{
		g_left_pwm = left_pwm;
    g_right_pwm = right_pwm;
    Motor_Left(left_pwm);
    Motor_Right(right_pwm);
}

//---------------------------------------------------停止
void Motor_Stop(void)
{
    pwm_set_duty(L_PWM,0);
    pwm_set_duty(R_PWM,0);
}

//---------------------------------------------------电机pwm显示
void Motor_PWM_Show(void)
{
    ips200_show_string(8 * 0,16 * 0, "L_PWM:");
    ips200_show_int(8 * 6,16 * 0, g_left_pwm, 5);
    ips200_show_string(8 * 16,16 * 0, "R_PWM:");
    ips200_show_int(8 * 22,16 * 0, g_right_pwm, 5);
}