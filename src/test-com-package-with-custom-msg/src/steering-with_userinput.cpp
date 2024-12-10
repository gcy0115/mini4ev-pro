#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <mutex>
#include <sstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"

class MotorCommandPublisher : public rclcpp::Node
{
public:
    MotorCommandPublisher()
        : Node("motor_command_publisher"), rate_(100)
    {
        publisher_ = this->create_publisher<custom_interfaces::msg::SteeringAngle>("steering_command", 10);

        // 初始目标位置为0.0
        angles_[0] = 0.0;
        angles_[1] = 0.0;
        angles_[2] = 0.0;
        angles_[3] = 0.0;

        RCLCPP_INFO(this->get_logger(), "Motor Command Publisher is running...");
        RCLCPP_INFO(this->get_logger(), "Initial positions are set to 0.0");
        RCLCPP_INFO(this->get_logger(), "Listening for new target positions...");
        
        // 创建发布线程
        publish_thread_ = std::thread(&MotorCommandPublisher::publish_loop, this);

        // 开启用户输入线程
        input_thread_ = std::thread(&MotorCommandPublisher::input_loop, this);
    }

    ~MotorCommandPublisher()
    {
        if (publish_thread_.joinable())
            publish_thread_.join();
        if (input_thread_.joinable())
            input_thread_.join();
    }

private:
    void publish_loop()
    {
        rclcpp::WallRate loop_rate(rate_);
        while (rclcpp::ok())
        {
            auto message = custom_interfaces::msg::SteeringAngle();

            // 锁定角度数组，填充消息
            {
                std::lock_guard<std::mutex> lock(angle_mutex_);
                message.angle1 = angles_[0];
                message.angle2 = angles_[1];
                message.angle3 = angles_[2];
                message.angle4 = angles_[3];
            }

            // 发布消息
            publisher_->publish(message);
            RCLCPP_INFO_ONCE(this->get_logger(), "Publishing angles at 100Hz...");
            loop_rate.sleep();
        }
    }

    void input_loop()
    {
        while (rclcpp::ok())
        {
            std::string input;
            std::getline(std::cin, input);

            std::istringstream iss(input);
            double angle1, angle2, angle3, angle4;
            if (!(iss >> angle1 >> angle2 >> angle3 >> angle4))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid input. Please enter four space-separated numbers.");
                continue;
            }

            // 更新目标角度
            {
                std::lock_guard<std::mutex> lock(angle_mutex_);
                angles_[0] = angle1;
                angles_[1] = angle2;
                angles_[2] = angle3;
                angles_[3] = angle4;
            }

            RCLCPP_INFO(this->get_logger(), "Updated angles: %.2f %.2f %.2f %.2f", angle1, angle2, angle3, angle4);
        }
    }

    rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr publisher_;
    std::thread publish_thread_;  // 发布线程
    std::thread input_thread_;    // 用户输入线程
    std::mutex angle_mutex_;      // 保护目标角度的互斥锁
    double angles_[4];            // 电机目标位置数组
    int rate_;                    // 发布频率
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorCommandPublisher>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
