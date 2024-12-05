# damiao-linux下速度位置模式控制思路与介绍

## 1. 通讯设置
4310电机使用can通讯，串口部分用于连接上位机调试。上位机仅在windows平台上使用。详细的连接和调试说明，参考[使用说明书](调试助手使用说明书（达妙驱动控制协议）V1.4.pdf)。

代码运行平台使用Ubuntu22，can设备为[usb转can](http://e.tb.cn/h.T1gwEpVj8v1dp8S?tk=ocHw3sugto3)，我这里选了1.0隔离版本。
在 Ubuntu 或其他 Linux 发行版中，使用以下命令安装：
```shell
sudo apt update
sudo apt install can-utils  #确保 can-utils 已安装
```

分析仪默认刷了pcan的固件，在我这里一直没有用起来，于是选择刷canable的官方固件，参考[官方updater](https://canable.io/updater/canable1.html)，刷入固件后在Ubuntu下即可识别：

```shell
ifconfig -a

    can0: flags=193<UP,RUNNING,NOARP>  mtu 16
        unspec 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  txqueuelen 10  (未指定)
        RX packets 219792  bytes 1758336 (1.7 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 112701  bytes 901608 (901.6 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```
可测试如下内容：

```sh
sudo ip link set can0 type can bitrate 1000000  #设置can0通讯波特率
sudo ifconfig can0 up  #启用can0
cansend can0 123#1122334455667788  #发送一个 CAN 帧，ID 为 123，数据为 11 22 33 44 55 66 77 88
sudo ifconfig can0 down  #停用can0
```
可开启另一个终端使用`candump`命令监听发送的指令是否被接收：
```shell
candump can0

    can0  123   [8]  11 22 33 44 55 66 77 88  #应收到的内容
```

## 2. 控制流程
电机完成校准、标定以及参数设置后，即可以使用，控制使用 CAN 标准帧
格式，固定波特率为 1Mbps，按功能可分为接收帧和反馈帧，接收帧为接收到的控制数据，用于实现对电机的命令控制；反馈帧为电机向上层控制器发送电机的状态数据。根据电机选定的不同模式，其接收帧帧格式定义以及帧 ID 各不相同，但各种模式下的反馈帧是相同的。

注意：**4310的反馈帧只有接收can指令后才会发送，所以在电机使能后，必须持续发送控制帧才能得到持续的反馈帧。**

在实现思路上，我们在主函数中开启三个线程，

1. **接收线程**: 用于接收，无限循环，后续可加上ROS广播节点；
2. **控制线程**: 用于发送控制帧，需要维持一定的控制频率，过高可能导致can负载过大，我这里单电机可运行在1000hz，更多电机时需要适当降低控制频率或增加多路can；
3. **订阅线程**: 用于接收外界指令，并传递给控制线程。目前订阅来自键盘的输入，后续可加上ROS订阅节点。

### 2.0 初始化收发状态
主要需要对can接口进行设置，主要函数参见[can_opration.hpp](include/can_opration.hpp)。参考通讯设置中需要在终端中的操作命令，我们使用`system`函数在代码中执行该命令。实际操作中，使用如下代码即可完成通讯部分的设置：
```cpp
std::string can_id = "can0";  //指定can口
initial_can(can_id);  //完成can0的波特率、上拉设置
int sock = openCANSocket(can_id.c_str());  //初始化一个socket网络，设置为can样式，并绑定到can0上
```
需要时，可以使用以下代码下拉can，或用于重新启动can口时的前置操作，避免出现`device busy`样式的报错：
```cpp
terminate_can(can_id);
```


### 2.1 接收线程
接收线程主要需


---
# TODO
1. ~~设置can状态时，需要`sudo`权限，导致每次需要在命令行中输入密码才能继续执行脚本，可以参考如下方案：~~

方法 2：使用 sudo 与 -S 参数
另一个方法是通过 sudo 的 -S 选项来传递密码给 sudo，使得在执行命令时不要求手动输入密码。然而，这种方法通常不推荐，因为你必须将密码硬编码到程序中，这样会带来安全隐患。为了演示，这里是一个如何用 -S 选项在程序中传递密码的例子：

```cpp
#include <iostream>
#include <cstdlib>

int main() {
    const char* command = "echo 'your_password' | sudo -S /path/to/your_command";
    system(command);
    return 0;
}
```
echo 'your_password'：模拟输入密码。
sudo -S：-S 选项允许 sudo 从标准输入读取密码。
|：通过管道将密码传递给 sudo。
警告：将密码硬编码到程序中是非常不安全的，这样做容易导致密码泄露。强烈建议使用第一种方法（修改 sudoers 文件），以确保安全性。

已完成，参考如下指令即可：

```cpp
"echo '1234' | sudo -S ip link set " + can_name + " type can bitrate 1000000"
```

2. ROS2下数据的收发
考虑订阅来自`controller`的电机转角信息，并广播电机状态信息。








