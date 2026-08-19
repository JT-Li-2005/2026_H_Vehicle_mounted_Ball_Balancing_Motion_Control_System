#include "zf_common_headfile.h"
//一定要使用稳定的5V电源供电,探头离地面高度为 15mm-30mm 为佳

void Gray_sensor_init(void)//灰度传感器初始化
{
		 gpio_init(L2, GPI, 1, GPI_PULL_UP);
		 gpio_init(L1, GPI, 1, GPI_PULL_UP);
		 gpio_init(M, GPI, 1, GPI_PULL_UP);
		 gpio_init(R1, GPI, 1, GPI_PULL_UP);
	   	gpio_init(R2, GPI, 1, GPI_PULL_UP);
}
	int level_1=0;
	int level_2=0;
	int level_3=0;
	int level_4=0;
	int level_5=0;

void Gray_scan(void)//读取灰度传感器数值
{
		level_1=gpio_get_level(L2);
		level_2=gpio_get_level(L1);
		level_3=gpio_get_level(M);
		level_4=gpio_get_level(R1);
		level_5=gpio_get_level(R2);

}

void Gray_text(void)//数值屏幕显示
{
			ips200_show_string(8*3,16*3, "L2");		ips200_show_int(8*3,16*4,level_1,1);
			ips200_show_string(8*7,16*3, "L1");		ips200_show_int(8*7,16*4,level_2,1);
			ips200_show_string(8*11,16*3, "M");		ips200_show_int(8*11,16*4,level_3,1);
			ips200_show_string(8*15,16*3, "R1");		ips200_show_int(8*15,16*4,level_4,1);
	    ips200_show_string(8*19,16*3, "R2");		ips200_show_int(8*19,16*4,level_5,1);
}

float Gray_GetError(void)//误差
{
    static float LastError=0;

    Gray_scan();

    int sum=0;
    int cnt=0;

    if(level_1==0){sum+=-2;cnt++;}
    if(level_2==0){sum+=-1;cnt++;}
    if(level_3==0){sum+=0;cnt++;}
    if(level_4==0){sum+=1;cnt++;}
    if(level_5==0){sum+=2;cnt++;}

    if(cnt==0)
        return LastError;

    LastError=(float)sum/cnt;

    return LastError;
}