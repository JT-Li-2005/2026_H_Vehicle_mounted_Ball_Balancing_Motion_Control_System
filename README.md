# 2026-H-车载平衡滚球运动控制系统
 小车循迹和滚球平衡是两个相对独立的系统,两个系统之间无数据交换通路。控制滚球平衡的舵机直接由视觉模块（K230）控制。循迹部分和滚球控制部分仅有电源部分共用。<br>
### 小车循迹模块使用:<br>
 1、下载LineFollow.zip后解压。<br>
 2、使用keil打开路径LineFollow/projiect/Keil/SeekFree_MSPM0G3507_Device_Library.uvprojx<br>
 3、使用DAP下载器连接MSPM0G3507核心板，编译烧录。<br>  
### 滚球控制模块:<br>
1、下载main.py<br>
2、使用tpye-c接口连接K230和电脑，将main.py拷入k230的sd卡中（sdcard而不是data）
（注：K230的sd卡初始化步骤见XXX文件）<br>
