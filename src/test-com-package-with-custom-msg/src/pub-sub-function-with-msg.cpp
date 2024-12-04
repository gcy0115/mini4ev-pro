#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/msg/angle.hpp"
#include "custom_interfaces/msg/steering_motors.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class MinimalPublisher : public rclcpp::Node

{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    /* 节点初始化：
    调用父类构造函数，初始化节点，指定名称为 "minimal_publisher"。
    初始化 count_ 为 0，用于消息计数。*/
    {
      publisher_ = this->create_publisher<custom_interfaces::msg::SteeringMotors>("steering_state_message", 10);

      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
      /*
      创建一个定时器，每隔 500 毫秒触发一次。
      std::bind(&MinimalPublisher::timer_callback, this) 将 timer_callback 函数与当前类实例绑定，使定时器调用该函数。*/
    }

  private:
    void timer_callback()
    {
      auto message = custom_interfaces::msg::SteeringMotors();
      // 创建一个消息对象，类型为 custom_interfaces::msg::SteeringMotors()，用来存储即将发布的数据。
      
      // 填充motor1的数据，实际内容可以使用电机反馈帧
      message.motor1.position = 1.0;
      message.motor1.velocity = 0.5;
      message.motor1.torque = 0.2;
      message.motor1.temperature = 40.0;
      message.motor1.err = 1;
      message.motor1.canid = 0x101;
      message.motor1.name = "FL";

      // 填充motor2的数据，实际内容可以使用电机反馈帧
      message.motor2.position = 2.0;
      message.motor2.velocity = 1.5;
      message.motor2.torque = 1.2;
      message.motor2.temperature = 40.0;
      message.motor2.err = 1;
      message.motor2.canid = 0x102;
      message.motor2.name = "FR";

      // 填充motor3的数据，实际内容可以使用电机反馈帧
      message.motor3.position = 3.0;
      message.motor3.velocity = 2.5;
      message.motor3.torque = 1.6;
      message.motor3.temperature = 42.0;
      message.motor3.err = 1;
      message.motor3.canid = 0x103;
      message.motor3.name = "BR";

      // 填充motor4的数据，实际内容可以使用电机反馈帧
      message.motor4.position = 4.0;
      message.motor4.velocity = 4.5;
      message.motor4.torque = 1.9;
      message.motor4.temperature = 43.0;
      message.motor4.err = 1;
      message.motor4.canid = 0x104;
      message.motor4.name = "BL";

      
      RCLCPP_INFO(this->get_logger(), "Publishing steering motors\' states");
      /*
      使用 ROS 日志系统打印发布的消息内容。
      this->get_logger() 返回该节点的日志记录器*/
      publisher_->publish(message);
      // 将创建的消息发布到话题 message。
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_interfaces::msg::SteeringMotors>::SharedPtr publisher_;
    size_t count_;
    /*
    rclcpp::TimerBase::SharedPtr timer_：指向定时器的智能指针，用于触发 timer_callback。
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_：指向发布者的智能指针，用于发布消息。
    size_t count_：计数器，用于生成递增的消息内容。*/
};


class MinimalSubscriber : public rclcpp::Node

{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    /*
    节点初始化：
    调用父类构造函数，初始化节点，指定名称为 "minimal_subscriber"。
    */
    {
      subscription_ = this->create_subscription<custom_interfaces::msg::SteeringMotors>(
      "steering_state_message", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }


  private:
    void topic_callback(const custom_interfaces::msg::SteeringMotors::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received motor states:");
        
      RCLCPP_INFO(this->get_logger(), "Received from: motor1 - Position: %.2f, Velocity: %.2f, Torque: %.2f, Temperature: %.2f",
                    msg->motor1.position, msg->motor1.velocity, msg->motor1.torque, msg->motor1.temperature);

      RCLCPP_INFO(this->get_logger(), "Received from: motor2 - Position: %.2f, Velocity: %.2f, Torque: %.2f, Temperature: %.2f",
                    msg->motor2.position, msg->motor2.velocity, msg->motor2.torque, msg->motor2.temperature);

      RCLCPP_INFO(this->get_logger(), "Received from: motor3 - Position: %.2f, Velocity: %.2f, Torque: %.2f, Temperature: %.2f",
                    msg->motor3.position, msg->motor3.velocity, msg->motor3.torque, msg->motor3.temperature);

      RCLCPP_INFO(this->get_logger(), "Received from: motor4 - Position: %.2f, Velocity: %.2f, Torque: %.2f, Temperature: %.2f",
                    msg->motor4.position, msg->motor4.velocity, msg->motor4.torque, msg->motor4.temperature);
    }

    rclcpp::Subscription<custom_interfaces::msg::SteeringMotors>::SharedPtr subscription_;
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_：指向订阅的智能指针，用于接收消息。
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 使用多线程执行器同时运行发布者和订阅者
  rclcpp::executors::MultiThreadedExecutor executor;
  
  // 创建节点实例
  auto publisher_node = std::make_shared<MinimalPublisher>();
  auto subscriber_node = std::make_shared<MinimalSubscriber>();

  // 将节点添加到执行器中
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  // 启动执行器
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
