
## 1. NXP BLE4.0 WRGB Light
本项目采用NXP KW40Z SOC芯片，KW40Z支持BLE4.1，Zigbee无线并发通讯
官方SDK和BLE框架设计很优秀，相比TI的CC2540架构，NXP软件API用起来非常简单，尤其是BLE GATT Database Server宏设计，极其精巧


## 2. android 编译环境
- Android SDK v23
- Android Build Tools v23.0.0
- Android Support Repository

## 3. KW40Z 编译环境
-	IAR 7.40.2

## 4. 硬件原理图和PCB
-	Cadence 16.6
-	Allegro

## 5. WRGB硬件接口
NXP软件框架中使用了Timer0，所以这里采用Timer1/Timer2两个PWM定时器
每个定时器使用两个PWM通道Ch0，Ch1

硬件连接如下

LED W -- T1 -- Ch0 -- PC4 -- KW40Z Board J3 Pin2
LED B -- T1 -- Ch1 -- PC5 -- KW40Z Board J4 Pin10
LED G -- T2 -- Ch0 -- PC6 -- KW40Z Board J1 Pin2
LED R -- T2 -- Ch1 -- PC7 -- KW40Z Board J1 Pin4


## 6.  用法
-	1.	下载固件到KW40Z板子后，板载2个LED快速闪烁，按下SW4，LED停止闪烁
-	2.	打开安卓APK，开启蓝牙，然后会看到FSL开头的蓝牙4.0设备，点击连接
-	3.	拖动4个WRGB指示条，颜色框会实时变化，同时会在KW40Z板子上输出4路20KHz的PWM波形



