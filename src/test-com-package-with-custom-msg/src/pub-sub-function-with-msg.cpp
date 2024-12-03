#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/msg/angle.hpp"

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
      publisher_ = this->create_publisher<custom_interfaces::msg::Angle>("message", 10);

      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
      /*
      创建一个定时器，每隔 500 毫秒触发一次。
      std::bind(&MinimalPublisher::timer_callback, this) 将 timer_callback 函数与当前类实例绑定，使定时器调用该函数。*/
    }

  private:
    void timer_callback()
    {
      auto message = custom_interfaces::msg::Angle();
      // 创建一个消息对象，类型为 std_msgs::msg::String，用来存储即将发布的数据。
      message.angle = count_++;
      /*
      设置消息的 data 字段。
      count_++ 是一个计数器，用于在每条消息后附加递增的数字。*/
      
      RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.angle);
      /*
      使用 ROS 日志系统打印发布的消息内容。
      this->get_logger() 返回该节点的日志记录器*/
      publisher_->publish(message);
      // 将创建的消息发布到话题 message。
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_interfaces::msg::Angle>::SharedPtr publisher_;
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
      subscription_ = this->create_subscription<custom_interfaces::msg::Angle>(
      "message", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    /*
    功能：
    1. 创建一个订阅者，订阅 ROS 2 话题 message。
    2. 订阅的消息类型是 std_msgs::msg::String。
    3. 消息队列大小为 10，表示最多可以缓存 10 条未处理的消息。
    4. 当有新消息到达时，会调用 topic_callback 函数处理消息。

    关键点：
    std::bind(&MinimalSubscriber::topic_callback, this, _1)：
      1.将成员函数 topic_callback 绑定到当前类的实例。
      2. this 指向当前对象（MinimalSubscriber 的实例），确保 topic_callback 在正确的上下文中执行。
      3. _1 表示占位符，用于接收订阅到的消息。
    */
    }


  private:
    void topic_callback(const custom_interfaces::msg::Angle & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%ld'", msg.angle);
    }
    /*
    功能
    参数：
      const std_msgs::msg::String & msg：引用类型参数，表示接收到的 ROS 2 消息。
      消息内容存储在 msg.data 中，是一个 std::string。
    
    打印消息
      使用 ROS 2 的日志系统 RCLCPP_INFO 将接收到的消息内容打印到终端。
      this->get_logger()：返回该节点的日志记录器。
      msg.data.c_str()：将 std::string 转换为 C 风格字符串，方便格式化输出。
    */

    rclcpp::Subscription<custom_interfaces::msg::Angle>::SharedPtr subscription_;
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
