#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <iostream>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/msg/angle.hpp"
#include "custom_interfaces/msg/steering_motors.hpp"

#include "../include/steering_motors_package/can_opration.hpp"
#include "../include/steering_motors_package/dm4310.hpp"

using namespace std::chrono_literals;


class SteeringPublisher : public rclcpp::Node

{
public:
  SteeringPublisher(int sock)
      : Node("steering_publisher"), sock_(sock)
  {
    // 创建发布者
    publisher_ = this->create_publisher<custom_interfaces::msg::SteeringMotors>("steering_state_message", 10);

    // 启动接收线程
    receive_thread_ = std::thread(&SteeringPublisher::receiveFeedback, this);
  }

  ~SteeringPublisher()
  {
    stop_thread_ = true; // 停止接收线程
    if (receive_thread_.joinable())
    {
      receive_thread_.join();
    }
  }


  private:
    int sock_;

    // 使用单独声明的 dmMotor 对象
    dmMotor motorFL{0x101, 0x021}; // 前左电机，主ID和子ID
    dmMotor motorFR{0x102, 0x022}; // 前右电机，主ID和子ID
    dmMotor motorBR{0x103, 0x023}; // 后右电机，主ID和子ID
    dmMotor motorBL{0x104, 0x024}; // 后左电机，主ID和子ID

    // 将电机对象存入 vector
    std::vector<dmMotor> motors_{motorFL, motorFR, motorBR, motorBL};

    void receiveFeedback()
    {
      while (!stop_thread_)
      {
        custom_interfaces::msg::SteeringMotors message;

        for (auto &motor : motors_)
        {
          std::vector<uint8_t> received_data;
          if (receiveCANFrame(sock_, motor.getMasterID(), received_data))
          {
            MotorFeedback feedback = parseCANFeedback( received_data);

            // 更新电机状态
            motor.setERR(feedback.ERR);
            motor.setPOS(feedback.POS);
            motor.setVEL(feedback.VEL);
            motor.setT(feedback.T);
            motor.setTMOS(feedback.T_MOS);
            motor.setTRotor(feedback.T_Rotor);

            // 填充消息
            if (motor.getMasterID() == 0x101)
            {
              message.motor1.position = motor.getPOS();
              message.motor1.velocity = motor.getVEL();
              message.motor1.torque = motor.getT();
              message.motor1.temperature = motor.getTRotor();
              message.motor1.err = motor.getERR();
              message.motor1.canid = motor.getMasterID();
              message.motor1.name = "FL";
            }
            else if (motor.getMasterID() == 0x102)
            {
              message.motor2.position = motor.getPOS();
              message.motor2.velocity = motor.getVEL();
              message.motor2.torque = motor.getT();
              message.motor2.temperature = motor.getTRotor();
              message.motor2.err = motor.getERR();
              message.motor2.canid = motor.getMasterID();
              message.motor2.name = "FR";
            }
            else if (motor.getMasterID() == 0x103)
            {
              message.motor3.position = motor.getPOS();
              message.motor3.velocity = motor.getVEL();
              message.motor3.torque = motor.getT();
              message.motor3.temperature = motor.getTRotor();
              message.motor3.err = motor.getERR();
              message.motor3.canid = motor.getMasterID();
              message.motor3.name = "BR";
            }
            else if (motor.getMasterID() == 0x104)
            {
              message.motor4.position = motor.getPOS();
              message.motor4.velocity = motor.getVEL();
              message.motor4.torque = motor.getT();
              message.motor4.temperature = motor.getTRotor();
              message.motor4.err = motor.getERR();
              message.motor4.canid = motor.getMasterID();
              message.motor4.name = "BL";
            }
          }
        }

        // 发布消息
        RCLCPP_INFO(this->get_logger(), "Publishing steering motors' states");
        publisher_->publish(message);
      }
    }

    rclcpp::Publisher<custom_interfaces::msg::SteeringMotors>::SharedPtr publisher_;

    // 接收线程
    std::thread receive_thread_;
    bool stop_thread_ = false;
  };

int main() {

    // const char* can_interface = "can0";  // CAN 接口名称
    std::string can_id = "can0";
    initial_can(can_id);

    int sock = openCANSocket(can_id.c_str());
    // .c_str() 返回一个 const char* 指针，指向 std::string 中的字符数组，并在末尾加上空字符 \0。c_str() 返回的指针是只读的，不应被修改，否则会导致未定义行为。
    if (sock < 0) {
        return -1;
    }
    
    // SocketCAN 的套接字可以复用，即：
    // 一旦套接字被成功创建并绑定到指定的 CAN 接口上，你可以通过 write 函数不断向套接字发送新的 CAN 帧数据，而无需重新打开或绑定套接字。
    // 只需在 can_frame 结构体中更新 can_id 和 data，然后调用 write 函数即可将新的数据发送到 CAN 总线上。


  auto node = std::make_shared<SteeringPublisher>(sock);

  rclcpp::spin(node);

  rclcpp::shutdown();
  
  // 关闭 CAN 通道
  terminate_can(can_id);
  // 关闭套接字
  close(sock);
  return 0;
}