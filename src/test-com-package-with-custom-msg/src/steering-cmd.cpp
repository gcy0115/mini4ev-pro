#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"
#include "custom_interfaces/msg/steering_motors.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalPublisher : public rclcpp::Node

{
  public:
    MinimalPublisher()
    : Node("steering_command_generator"), count_(0)
    /* 节点初始化：
    调用父类构造函数，初始化节点，指定名称为 "minimal_publisher"。
    初始化 count_ 为 0，用于消息计数。*/
    {
      publisher_ = this->create_publisher<custom_interfaces::msg::SteeringAngle>("steering_command", 10);

      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
      
    }

  private:
    void timer_callback()
    {
      auto message = custom_interfaces::msg::SteeringAngle();
      // 创建一个消息对象，类型为 custom_interfaces::msg::SteeringMotors()，用来存储即将发布的数据。
      
      // 填充motor1的数据，实际内容可以使用电机反馈帧
      message.angle1 = 1.0;
      message.angle2 = 0.0;
      message.angle3 = -1.0;
      message.angle4 = 1.0;

      
      RCLCPP_INFO(this->get_logger(), "Publishing steering motors\' states");
      /*
      使用 ROS 日志系统打印发布的消息内容。
      this->get_logger() 返回该节点的日志记录器*/
      publisher_->publish(message);
      // 将创建的消息发布到话题 message。
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr publisher_;
    size_t count_;
    /*
    rclcpp::TimerBase::SharedPtr timer_：指向定时器的智能指针，用于触发 timer_callback。
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_：指向发布者的智能指针，用于发布消息。
    size_t count_：计数器，用于生成递增的消息内容。*/
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
  
    // 创建节点实例
    auto publisher_node = std::make_shared<MinimalPublisher>();
    rclcpp::spin(publisher_node);

    rclcpp::shutdown();
    return 0;
}