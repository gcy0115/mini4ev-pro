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
在Linux系统中，可以通过编辑`/etc/modprobe.d/blacklist`文件来永久禁用`cdc_acm`模块。打开该文件，并添加以下行：

```bash
blacklist cdc_acm
```

#### 检查驱动是否完成安装
重新拔插一次设备，在终端中，使用`sudo dmesg`
```bash
[  592.120393] usb 4-1.1: new high-speed USB device number 25 using xhci-hcd
[  592.220802] usb 4-1.1: New USB device found, idVendor=1a86, idProduct=55d5, bcdDevice=80.48
[  592.220814] usb 4-1.1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[  592.220819] usb 4-1.1: Product: USB Quad_Serial
[  592.220822] usb 4-1.1: Manufacturer: wch.cn
[  592.220825] usb 4-1.1: SerialNumber: 0123456789
[  592.372143] usb_ch343 4-1.1:1.0: ttyCH343USB0: usb to uart device
[  592.372329] usb_ch343 4-1.1:1.0: USB to GPIO device now attached to ch343_iodev3
[  592.372810] usb_ch343 4-1.1:1.2: ttyCH343USB1: usb to uart device
[  592.373826] usb_ch343 4-1.1:1.4: ttyCH343USB2: usb to uart device
[  592.374218] usb_ch343 4-1.1:1.6: ttyCH343USB3: usb to uart device

mini4ev@mini4ev-desktop:~$ modinfo ch343
filename:       /lib/modules/6.8.0-1017-raspi/kernel/drivers/usb/serial/ch343.ko
alias:          char-major-170-*
license:        GPL
version:        V1.8 On 2024.03
description:    USB serial driver for ch342/ch343/ch344/ch347/ch339/ch9101/ch9102/ch9103/ch9104/ch9143, etc.
author:         WCH
srcversion:     B0F39AC8D1C2A1CA5A6AAB1
alias:          usb:v1A86p55DFd*dc*dsc*dp*ic*isc*ip*in*
alias:          usb:v1A86p55D7d*dc*dsc*dp*ic*isc*ip*in*
alias:          usb:v1A86p55D4d*dc*dsc*dp*ic*isc*ip*in*
alias:          usb:v1A86p55D8d*dc*dsc*dp*ic*isc*ip*in*
alias:          usb:v1A86p55E7d*dc*dsc*dp*ic*isc*ip*in00*
alias:          usb:v1A86p55DEd*dc*dsc*dp*ic*isc*ip*in02*
alias:          usb:v1A86p55DEd*dc*dsc*dp*ic*isc*ip*in00*
alias:          usb:v1A86p55DDd*dc*dsc*dp*ic*isc*ip*in00*
alias:          usb:v1A86p55DBd*dc*dsc*dp*ic*isc*ip*in00*
alias:          usb:v1A86p55DAd*dc*dsc*dp*ic*isc*ip*in*
alias:          usb:v1A86p55D6d*dc*dsc*dp*ic*isc*ip*in*
alias:          usb:v1A86p55D5d*dc*dsc*dp*ic*isc*ip*in*
alias:          usb:v1A86p55D3d*dc*dsc*dp*ic*isc*ip*in*
alias:          usb:v1A86p55D2d*dc*dsc*dp*ic*isc*ip*in*
depends:        
name:           ch343
vermagic:       6.8.0-1016-raspi SMP preempt mod_unload modversions aarch64
```
#### 规则设置
<!-- ### 低延时设置：
`sudo apt install expect`  
`sudo apt-get install -y setserial` -->


详细参考rules/rules.md，这用于给usb_ch343设备重新命名，并固定名称。此步骤不完成，工程将无法编译。