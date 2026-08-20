# 2026-H-车载平衡滚球运动控制系统
 小车循迹和滚球平衡是两个相对独立的系统,两个系统之间无串口通信。控制滚球平衡的舵机直接由视觉模块（K230）控制。循迹部分和滚球控制部分仅有电源部分共用。<br>
### 小车循迹模块:<br>
 1、下载LineFollow.zip后解压。<br>
 2、使用keil打开路径LineFollow/projiect/Keil/SeekFree_MSPM0G3507_Device_Library.uvprojx<br>
 3、使用DAP下载器连接MSPM0G3507核心板，编译烧录。<br>  
### 滚球控制模块:<br>
1、下载main.py<br>
2、使用tpye-c接口连接K230和电脑，将main.py拷入k230的sd卡中（sdcard而不是data）
（注：K230的sd卡初始化步骤见淘宝幻尔机器人提供的资料包） <br>
链接: https://pan.baidu.com/s/1h4ChUXApCscN0UfuirByRQ 提取码: un33<br>
