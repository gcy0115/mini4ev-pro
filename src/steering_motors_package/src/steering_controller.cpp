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
#include "custom_interfaces/msg/steering_angle.hpp"
#include "custom_interfaces/msg/steering_motors.hpp"

#include "../include/steering_motors_package/can_opration.hpp"
#include "../include/steering_motors_package/dm4310.hpp"

using namespace std::chrono_literals;

const float g_max_speed = 3.0f;  // 单位 rad/s

class SteeringPublisher : public rclcpp::Node{
  public:
    SteeringPublisher(int sock)
        : Node("steering_publisher"), sock_(sock)
    {
      // 创建发布者
      publisher_ = this->create_publisher<custom_interfaces::msg::SteeringMotors>("steering_state_message", 10);
      RCLCPP_INFO(this->get_logger(), "Creat a publisher");

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
      std::map<int, bool> motor_data_ready = {
          {0x021, false}, // 前左电机
          {0x022, false}, // 前右电机
          {0x023, false}, // 后右电机
          {0x024, false}  // 后左电机
      };

      custom_interfaces::msg::SteeringMotors message;

      while (!stop_thread_)
      {
          for (auto &motor : motors_)
          {
              can_frame received_data = receiveCANFrame(sock_);

              if (checkCANFrame(received_data))
              {
                  MotorFeedback feedback = parseCANFeedback(convertCANDataToVector(received_data));

                  // 更新电机状态
                  motor.setERR(feedback.ERR);
                  motor.setPOS(feedback.POS);
                  motor.setVEL(feedback.VEL);
                  motor.setT(feedback.T);
                  motor.setTMOS(feedback.T_MOS);
                  motor.setTRotor(feedback.T_Rotor);
                  motor.setMasterID(received_data.can_id);

                  // 填充消息
                  if (motor.getMasterID() == 0x021)
                  {
                      RCLCPP_INFO(this->get_logger(), "Receiving from 0x021");
                      message.motor1.position = motor.getPOS();
                      message.motor1.velocity = motor.getVEL();
                      message.motor1.torque = motor.getT();
                      message.motor1.temperature = motor.getTRotor();
                      message.motor1.err = motor.getERR();
                      message.motor1.canid = motor.getMasterID();
                      message.motor1.name = "FL";
                      motor_data_ready[0x021] = true;
                  }
                  else if (motor.getMasterID() == 0x022)
                  {
                      RCLCPP_INFO(this->get_logger(), "Receiving from 0x022");
                      message.motor2.position = motor.getPOS();
                      message.motor2.velocity = motor.getVEL();
                      message.motor2.torque = motor.getT();
                      message.motor2.temperature = motor.getTRotor();
                      message.motor2.err = motor.getERR();
                      message.motor2.canid = motor.getMasterID();
                      message.motor2.name = "FR";
                      motor_data_ready[0x022] = true;
                  }
                  else if (motor.getMasterID() == 0x023)
                  {
                      RCLCPP_INFO(this->get_logger(), "Receiving from 0x023");
                      message.motor3.position = motor.getPOS();
                      message.motor3.velocity = motor.getVEL();
                      message.motor3.torque = motor.getT();
                      message.motor3.temperature = motor.getTRotor();
                      message.motor3.err = motor.getERR();
                      message.motor3.canid = motor.getMasterID();
                      message.motor3.name = "BR";
                      motor_data_ready[0x023] = true;
                  }
                  else if (motor.getMasterID() == 0x024)
                  {
                      RCLCPP_INFO(this->get_logger(), "Receiving from 0x024");
                      message.motor4.position = motor.getPOS();
                      message.motor4.velocity = motor.getVEL();
                      message.motor4.torque = motor.getT();
                      message.motor4.temperature = motor.getTRotor();
                      message.motor4.err = motor.getERR();
                      message.motor4.canid = motor.getMasterID();
                      message.motor4.name = "BL";
                      motor_data_ready[0x024] = true;
                  }
              }
              else
              {
                  RCLCPP_INFO(this->get_logger(), "No CAN frame received!");
              }
          }

          // 检查是否所有电机数据都已准备好
          bool all_data_ready = std::all_of(
              motor_data_ready.begin(),
              motor_data_ready.end(),
              [](const std::pair<int, bool> &entry) { return entry.second; });

          if (all_data_ready)
          {
              RCLCPP_INFO(this->get_logger(), "Publishing steering motors' states");
              publisher_->publish(message);

              // 重置标志位
              for (auto &entry : motor_data_ready)
              {
                  entry.second = false;
              }
          }
      }
    }


    rclcpp::Publisher<custom_interfaces::msg::SteeringMotors>::SharedPtr publisher_;

    // 接收线程
    std::thread receive_thread_;
    bool stop_thread_ = false;
};

class SteeringController : public rclcpp::Node {
  public:
    SteeringController(int sock) : Node("steering_controller"), sock_(sock) {
        // 订阅 "steering_command" 主题
        subscription_ = this->create_subscription<custom_interfaces::msg::SteeringAngle>(
            "steering_command",
            10,
            std::bind(&SteeringController::commandCallback, this, std::placeholders::_1)
        );
    }

  private:
    void commandCallback(const custom_interfaces::msg::SteeringAngle::SharedPtr msg) {
        // 处理每个电机的目标角度
        if (PosControlFrame(sock_, 0x101, static_cast<float>(msg->angle1), g_max_speed) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send frame for motor 1");
        }
        if (PosControlFrame(sock_, 0x102, static_cast<float>(msg->angle2), g_max_speed) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send frame for motor 2");
        }
        if (PosControlFrame(sock_, 0x103, static_cast<float>(msg->angle3), g_max_speed) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send frame for motor 3");
        }
        if (PosControlFrame(sock_, 0x104, static_cast<float>(msg->angle4), g_max_speed) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send frame for motor 4");
        }

        RCLCPP_INFO(this->get_logger(), "Control commands sent successfully");
    }

    // ROS 2 订阅者
    rclcpp::Subscription<custom_interfaces::msg::SteeringAngle>::SharedPtr subscription_;

    // CAN socket
    int sock_;
};

int main(int argc, char * argv[]) {

    // const char* can_interface = "can0";  // CAN 接口名称
  std::string can_id = "can0";
  initial_can(can_id);

  int sock = openCANSocket(can_id.c_str());
    // .c_str() 返回一个 const char* 指针，指向 std::string 中的字符数组，并在末尾加上空字符 \0。c_str() 返回的指针是只读的，不应被修改，否则会导致未定义行为。
  if (sock < 0) {
        return -1;
  }

  // TODO:使能电机并检查电机状态


  rclcpp::init(argc, argv);

    // 使用多线程执行器同时运行发布者和订阅者
  rclcpp::executors::MultiThreadedExecutor executor;
    
    // 创建节点实例
  auto publisher_node = std::make_shared<SteeringPublisher>(sock);
  auto subscriber_node = std::make_shared<SteeringController>(sock);
    // 将节点添加到执行器中
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);
    // 启动执行器
  executor.spin();
  rclcpp::shutdown();
    
    // 关闭 CAN 通道
  terminate_can(can_id);
    // 关闭套接字
  close(sock);
  return 0;
}