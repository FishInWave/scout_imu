# IMU node
## 编译
依赖于Eigen和Boost以及Serial，需要事先安装Serial。
```bash
sudo apt-get install ros-melodic-serial
cd ~/catkin_ws
catkin_make
```
## 给串口/dev/ttyUSB0权限
`sudo chmod 777 /dev/ttyUSB0`
或者修改配置文件：
在 /etc/udev/rules.d 目录下，自己再添加一个 20-usb-serial.rules文件，内容如下：
`sudo touch 20-usb-serial.rules`
```bash
KERNEL=="ttyUSB*"  MODE="0777" 
KERNEL=="ttyS*"  MODE="0777" 
```
保存
重启电脑，或者拔掉由串口过来的USB线，重新连接
查看权限：`ls /dev/ttyS0 -l`


## 运行
确保IMU已经上电后，插上USB，静置5秒钟，等待IMU初始化，然后输入：
```bash
roscore
rosrun scout_imu scout_imu_node
```

## 文档说明
### independentofros/ 

老师创建的独立于ros的驱动,但尚存在bug，即其中断是一个字节一个字节触发的，并不能解析整个包
### src/和include/
`imu.cpp`和`imu.hpp`是imu的库文件，不过里面有两个版本。一个是基于`uint8_t*`，另一个是基于`std::vector<uint8_t>`。这主要是因为ros驱动目前也有两份，一个是基于io中断机制的`imu_node.cpp`（现行的版本）和基于定时器机制的`scout_imu_node.cpp`（错误版本）。
错误原因见于博客：https://blog.csdn.net/a850565178/article/details/111409986

后面可能哪次更新的时候就删了错误版本的相关代码吧。现在先留着提醒自己。

## 程序说明
1. 欧拉角上位机测试\
在Windows测试后发现，似乎是因为万向锁问题，当pitch到达90附近时候，yaw和roll剧变（180°附近），不过暂时并不影响程序的使用。

2. 噪声测试\
借助rqt_plot工具，可视化了加速度和角速度，波形基本保持平稳，说明IMU本身已对输出做了较好的滤波

3. 坐标系对齐\
从原始的串口数据可以得到，欧拉角的坐标系是符合ROS坐标系的定义，即XYZ-前左上，而加速度计和陀螺仪的坐标系为XYZ-右前上。在上位机中已统一转换至前者坐标系。

### TODO
1. IMU初始化检测\
目前代码中是不知道IMU有没有初始化的，而且似乎如果没初始化就启动节点会一直卡在那边。目前不知道具体原因。
2. 方差检验\
目前加速度计和陀螺仪的方差是来自于老师的原始代码，并不清楚是来自于厂家还是随便填的，至少我在说明书上没有看见。应该还是需要自己测试的，这一点将会影响预积分的误差。
这个开源程序可能可以提供一些便利：https://github.com/gaowenliang/imu_utils
