##NXP BLE4.0 Light 调试记录

简单列一下调试过程中出现的问题以及解决方案

### 1 LED驱动端口选择
采用PWM配合MOS管来驱动WRGB LED，封装采用5050
KW40Z有3个Timer，T0/T1/T2，其中T0被协议栈使用，这里采用T1/T2，刚好4个通道，分别分配给White/Blue/Green/Red

驱动代码参考官方蜂鸣器BEEP驱动，稍加修改即可

### 2 BLE Server Profile
这里选择HRS（心率）Profile 特征值0x2A39，修改权限为读写，长度修改为4个字节
，Write Notify

在GATT Server Callback中，添加Database写函数，更新数据库数据

### 3 Android BLE代码编写
采用Google官方BLE GATT 工程，然后添加启动页面Active，添加SeekBar和ImageView，在Seekbar数据更新回调函数中，更新ImageView的Bitmap颜色数据，然后对BLE 0x2A39特征值写入WRGB数据，就完成了Ardoid调整颜色动作

### 4 PCB Layout
用Cadence绘制原理图，自行绘画元件封装（其实官方的Cadence硬件工程文件，可以直接导出库），然后Layout即可

射频注意事项：单端50欧阻抗控制，共面波导模型，2层板，放置传输线地过孔
