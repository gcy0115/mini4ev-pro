#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class MinimalPublisher : public rclcpp::Node
/* 类 MinimalPublisher 的功能
1. 继承自 rclcpp::Node： MinimalPublisher 是一个 ROS 2 节点，通过继承 rclcpp::Node 基类获得节点功能。
2. 节点名称：在构造函数中通过 Node("minimal_publisher") 指定节点名称为 minimal_publisher。

核心任务：
定时器每隔 500 毫秒触发一次。
每次触发时，在话题 topic 上发布一条字符串消息。*/

/*
运行流程
ROS 2 节点启动时，MinimalPublisher 类被实例化。
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
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    /* 节点初始化：
    调用父类构造函数，初始化节点，指定名称为 "minimal_publisher"。
    初始化 count_ 为 0，用于消息计数。*/
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      /*
      创建一个发布者，负责向话题 topic 发布 std_msgs::msg::String 类型的消息。
      第二个参数 10 是发布队列的深度，表示最多可以缓冲 10 条未发送的消息。*/

      /*
      在 C++ 中，this 是一个指针，指向当前类的实例。
      this 的含义是指向当前类 MinimalPublisher 实例的指针。它用于调用类的成员函数或访问类的成员变量。
      create_publisher 是 rclcpp::Node 类的成员函数，用于创建一个 ROS 2 发布者。由于 MinimalPublisher 类继承自 rclcpp::Node，它也继承了 create_publisher 方法。

      作用：
      使用 this 明确调用的是当前实例的 create_publisher 方法。
      这里的 this 指代 MinimalPublisher 的实例，因为该类继承了 rclcpp::Node，所以可以通过 this 调用 rclcpp::Node 的成员函数。

      在大多数情况下，省略 this 是可以的。但在某些上下文（如避免歧义或在需要明确指明作用域的情况下），显式使用 this 是必要的。
      */

      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
      /*
      创建一个定时器，每隔 500 毫秒触发一次。
      std::bind(&MinimalPublisher::timer_callback, this) 将 timer_callback 函数与当前类实例绑定，使定时器调用该函数。*/
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


class MinimalSubscriber : public rclcpp::Node
/*
类 MinimalSubscriber 的功能
1. 继承自 rclcpp::Node：
  MinimalSubscriber 是一个 ROS 2 节点，继承了 rclcpp::Node 类。
  通过继承，它获得了 ROS 2 节点的基本功能，比如创建订阅者和发布者、访问日志系统等。

核心任务：
订阅话题 topic。
  当接收到类型为 std_msgs::msg::String 的消息时，调用回调函数 topic_callback 处理并打印消息内容。
*/
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    /*
    节点初始化：
    调用父类构造函数，初始化节点，指定名称为 "minimal_subscriber"。
    */
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    /*
    功能：
    1. 创建一个订阅者，订阅 ROS 2 话题 topic。
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
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
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

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_：指向订阅的智能指针，用于接收消息。
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}