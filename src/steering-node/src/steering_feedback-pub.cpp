#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "include/steering-node/can_opration.hpp"
#include "include/steering-node/dm4310.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// 接收电机反馈帧
auto receiveThread(int sock, dmMotor motor) {
    while (true) {
        std::vector<uint8_t> received_data;
        if (receiveCANFrame(sock, motor.getMasterID(), received_data)) {
            // 处理接收到的反馈数据

            // printReceivedCANFrame(sock, motor.getMasterID());
            MotorFeedback feedback = parseCANFeedback(motor.getMasterID(), received_data);
            motor.setERR(feedback.ERR);
            motor.setPOS(feedback.POS);
            motor.setVEL(feedback.VEL);
            motor.setT(feedback.T);
            motor.setTMOS(feedback.T_MOS);
            motor.setTRotor(feedback.T_Rotor);
            // std::cout << "Received from Motor: " << feedback.motorID << std::endl;
            // std::cout << "ERR: " << feedback.ERR << std::endl;
            // std::cout << "POS: " << feedback.POS << std::endl;
            // std::cout << "VEL: " << feedback.VEL << std::endl;
            // std::cout << "T: " << feedback.T << std::endl;
            // std::cout << "T_MOS: " << static_cast<int>(feedback.T_MOS) << std::endl;
            // std::cout << "Received feedback from motor :" << motor.getMasterID() << std::endl;
        }
    }
    return motor;
}

class SteeringMotorPublisher : public rclcpp::Node
/* 类 MinimalPublisher 的功能
1. 继承自 rclcpp::Node： SteeringMotorPublisher 是一个 ROS 2 节点，通过继承 rclcpp::Node 基类获得节点功能。
2. 节点名称：在构造函数中通过 Node("steering_motor_publisher") 指定节点名称为 steering_motor_publisher。

核心任务：

每次触发时，在话题 steering_motor_feedback 上发布当前收到的转向电机信息。*/

/*
运行流程
ROS 2 节点启动时，SteeringMotorPublisher 类被实例化。
构造函数中：
初始化发布者，监听话题 topic。
设置定时器，每隔 500 毫秒调用一次 timer_callback。
每次 timer_callback 执行时：
创建消息对象。
生成消息内容。
打印日志。
将消息发布到话题 topic。
ROS 2 网络中，任何订阅了 topic 的节点都会接收到这些消息。
*/

{
  public:
    SteeringMotorPublisher()
    : Node("steering_motor_publisher"), count_(0)
    /* 节点初始化：
    调用父类构造函数，初始化节点，指定名称为 "minimal_publisher"。
    初始化 count_ 为 0，用于消息计数。*/
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      /*
      创建一个发布者，负责向话题 topic 发布 std_msgs::msg::String 类型的消息。
      第二个参数 10 是发布队列的深度，表示最多可以缓冲 10 条未发送的消息。*/


    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      // 创建一个消息对象，类型为 std_msgs::msg::String，用来存储即将发布的数据。
      message.data = "Hello, world! " + std::to_string(count_++);
      /*
      设置消息的 data 字段。
      count_++ 是一个计数器，用于在每条消息后附加递增的数字。*/
      
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      /*
      使用 ROS 日志系统打印发布的消息内容。
      this->get_logger() 返回该节点的日志记录器*/
      publisher_->publish(message);
      // 将创建的消息发布到话题 topic。
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    /*
    rclcpp::TimerBase::SharedPtr timer_：指向定时器的智能指针，用于触发 timer_callback。
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_：指向发布者的智能指针，用于发布消息。
    size_t count_：计数器，用于生成递增的消息内容。*/
};