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
每次触发时，在话题 message 上发布一条字符串消息。*/

/*
运行流程
ROS 2 节点启动时，MinimalPublisher 类被实例化。
构造函数中：
初始化发布者，监听话题 message。
设置定时器，每隔 500 毫秒调用一次 timer_callback。
每次 timer_callback 执行时：
创建消息对象。
生成消息内容。
打印日志。
将消息发布到话题 message。
ROS 2 网络中，任何订阅了 message 的节点都会接收到这些消息。
*/

{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    /* 节点初始化：
    调用父类构造函数，初始化节点，指定名称为 "minimal_publisher"。
    初始化 count_ 为 0，用于消息计数。*/
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("message", 10);
      /*
      创建一个发布者，负责向话题 message 发布 std_msgs::msg::String 类型的消息。
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
      // 将创建的消息发布到话题 message。
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
订阅话题 message。
  当接收到类型为 std_msgs::msg::String 的消息时，调用回调函数 message_callback 处理并打印消息内容。
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

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalSubscriber>());
//   rclcpp::spin(std::make_shared<MinimalPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }

/*
您的代码中包含了一个发布者（MinimalPublisher）和一个订阅者（MinimalSubscriber）类，旨在实现同一程序中同时运行发布者和订阅者。整体思路是对的，但存在一个问题：rclcpp::spin 在 ROS 2 中是一个阻塞函数，它会阻塞直到节点停止。因此，代码中的以下部分会导致问题：

rclcpp::spin(std::make_shared<MinimalSubscriber>());
rclcpp::spin(std::make_shared<MinimalPublisher>());

rclcpp::spin 会阻塞直到 MinimalSubscriber 节点退出。这样，MinimalPublisher 的 spin 调用实际上永远不会执行，因为第一个 spin 调用已经阻塞了执行。因此，您需要使用多线程或其他方式来让发布者和订阅者同时运行。

解决方法
可以使用 rclcpp::executors::MultiThreadedExecutor 来使两个节点并行运行。MultiThreadedExecutor 会在多个线程中同时执行多个节点，这样发布者和订阅者可以在不同的线程中并行工作。
*/


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

/*
使用 std::thread 实现多线程
在这种情况下，您可以分别在两个线程中调用 rclcpp::spin，一个线程处理发布者，另一个线程处理订阅者。下面是使用 std::thread 进行多线程的修改版本：

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 创建节点实例
  auto publisher_node = std::make_shared<MinimalPublisher>();
  auto subscriber_node = std::make_shared<MinimalSubscriber>();

  // 创建两个线程：一个用于发布者，另一个用于订阅者
  std::thread publisher_thread([publisher_node]() {
    rclcpp::spin(publisher_node);  // 发布者节点的spin
  });

  std::thread subscriber_thread([subscriber_node]() {
    rclcpp::spin(subscriber_node);  // 订阅者节点的spin
  });

  // 等待两个线程完成
  publisher_thread.join();
  subscriber_thread.join();

  rclcpp::shutdown();
  return 0;
}

主要修改点：
引入 std::thread：

在 main 函数中，创建两个线程：一个用于处理发布者节点，另一个用于处理订阅者节点。
分别调用 rclcpp::spin：

每个线程内分别调用 rclcpp::spin 来处理各自的 ROS 2 节点。
等待线程完成：

使用 join 来确保主线程等待两个子线程完成。这样，主线程在 ROS 2 节点关闭后才会退出。

优点：
手动控制线程：
您可以精确控制线程的行为（如启动、同步等）。
灵活性：
可以根据需要调整线程的数量和任务。对于复杂的应用，使用 std::thread 提供了更高的灵活性。

缺点：
复杂性：
手动管理线程需要小心处理线程同步、资源共享等问题。如果多个线程共享同一资源（如日志、变量等），需要考虑线程安全问题。
与 ROS 2 的 executor 比较：
使用 std::thread 是一种手动的多线程管理方式，可能会比 rclcpp::executors::MultiThreadedExecutor 更加复杂。MultiThreadedExecutor 会自动管理线程，适用于大多数用例。
*/