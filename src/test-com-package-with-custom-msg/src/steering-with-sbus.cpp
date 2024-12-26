#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "sbus_interface/msg/sbus.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"

class MotorCommandSubscriber : public rclcpp::Node
{
public:
    MotorCommandSubscriber()
        : Node("motor_command_subscriber"), rate_(50)
    {
        // 创建订阅者，订阅 /sbus 话题
        sbus_subscription_ = this->create_subscription<sbus_interface::msg::Sbus>(
            "/sbus", 10, std::bind(&MotorCommandSubscriber::sbus_callback, this, std::placeholders::_1));

        // 创建发布者，发布给转向电机
        motor_command_publisher_ = this->create_publisher<custom_interfaces::msg::SteeringAngle>("steering_command", 10);

        RCLCPP_INFO(this->get_logger(), "Motor Command Subscriber is running...");
    }

private:
    void sbus_callback(const sbus_interface::msg::Sbus::SharedPtr msg)
    {
        // 提取 mapped_channels[0] 并映射到 -1 到 1 范围
        float mapped_value = static_cast<float>(msg->mapped_channels[0]) / 255.0f * 2.0f - 1.0f; // 映射到 -1 到 1

        // 映射后的值控制四个电机的转向角度
        double motor_angle = mapped_value * 1.0; // 将映射的值转换成角度范围（例如 -45 到 45 度）

        // 创建消息并设置电机的目标角度
        auto steering_msg = custom_interfaces::msg::SteeringAngle();
        steering_msg.angle1 = motor_angle;
        steering_msg.angle2 = motor_angle;
        steering_msg.angle3 = motor_angle;
        steering_msg.angle4 = motor_angle;

        if (msg->frame_lost || msg->failsafe){
            steering_msg.angle1 = 0;
            steering_msg.angle2 = 0;
            steering_msg.angle3 = 0;
            steering_msg.angle4 = 0;
        }


        // 发布转向命令
        motor_command_publisher_->publish(steering_msg);
        RCLCPP_INFO(this->get_logger(), "Published motor angles: %.2f %.2f %.2f %.2f", motor_angle, motor_angle, motor_angle, motor_angle);
    }

    rclcpp::Subscription<sbus_interface::msg::Sbus>::SharedPtr sbus_subscription_;
    rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr motor_command_publisher_;
    int rate_; // 发布频率
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorCommandSubscriber>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
