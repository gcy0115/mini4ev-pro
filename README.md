# mini4ev-pro
ROS2 workspace for mini4ev-pro. ROS2 version: Jazzy, Ubuntu version: 24.04

## Installation

### 安装 ROS2

`wget http://fishros.com/install -O fishros && . fishros`  
这里选择Jazzy版本，并使用配套的`rosdepc`来完成`rosdep init`过程（国内）；
 

### 设置 DynamixelSDK python

At ~/ directory
```bash
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/python/
sudo python setup.py install
```
### 串口设备设置

#### 安装 CH343_serial 芯片组驱动

At ~/ directory
```bash
git clone https://github.com/WCHSoftGroup/ch343ser_linux.git
cd ch343ser_linux/driver
sudo make install
```

若出现类似如下报错
```bash
make -C /lib/modules/6.8.0-1015-raspi/build  M=/home/mini4ev/ch343ser_linux/driver   
make[1]: *** /lib/modules/6.8.0-1015-raspi/build: No such file or directory.  Stop.
make: *** [Makefile:7: default] Error 2
```
是因为缺少内核头文件导致的，参考以下方案解决：
1. 确认当前内核版本：
```bash
uname -r
```
若使用树莓派，输出应该是类似 6.8.0-1015-raspi 的版本。
2. 安装内核头文件：
```bash
sudo apt update
sudo apt install raspberrypi-kernel-headers
```
3. 如果使用的不是树莓派官方系统，例如刷入ubuntu系统，可以尝试以下命令安装对应的内核头文件：
```bash
sudo apt install linux-headers-$(uname -r)
```
#### 禁用cdc_ACM驱动
在 Linux 系统中，`cdc_acm` 是一个内核模块，用于支持 USB 调制解调器和其他基于 ACM（通信设备类抽象控制模型）接口的设备。
当使用：
```bash
sudo rmmod cdc_acm
```
如果设备在 cdc_acm 被卸载后重新插入，它可能会再次加载模块。这会导致串口板上设备相关节点编译失败。因此我们需要永久禁用。


#### 规则设置

详细参考rules/rules.md