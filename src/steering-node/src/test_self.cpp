// test socket CAN via cpp
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include "include/steering-node/can_opration.hpp"
#include "include/steering-node/dm4310.hpp"


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

std::atomic<float> target_position_FR(0.0f);
std::atomic<float> target_velocity(30.0);
std::mutex mtx;


// 线程函数：持续发送控制帧
void controlLoop(int sock, int motor_id) {
    std::cout << "Starting control loop" << std::endl;
    while (true) {
        float pos, vel;
        {
            std::lock_guard<std::mutex> lock(mtx);  // 锁住以防数据竞争
            pos = target_position_FR.load();  //从全局变量中获取参数
            vel = target_velocity.load();
        }
        // std::cout << "Get:" << pos << "&" << vel << std::endl;
        PosControlFrame(sock, motor_id, pos, vel);

        // 每隔 1 毫秒发送一次
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 用户输入线程函数，后续可改成ros接收线程
void userInputLoop(int sock, int motor_id) {
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "Starting userinput" << std::endl;
    while (true) {
        // std::this_thread::sleep_for(std::chrono::seconds(3));
        std::cout << "Enter a new position (-2.0 to 2.0, or 'q' to quit): ";
        std::string input;
        std::cin >> input;

        // 检查退出条件
        if (input == "q") {
            std::cout << "Exiting control loop." << std::endl;
            disable_motor(sock, motor_id);
            exit(0);  // 终止整个程序
        }

        // 尝试将输入转换为浮点数
        try {
            float new_position = std::stof(input);
            if (new_position < -2.0f || new_position > 2.0f) {
                std::cerr << "Error: Position out of range (-2.0 to 2.0)." << std::endl;
                continue;
            }

            // 更新全局变量
            std::lock_guard<std::mutex> lock(mtx);
            target_position_FR.store(new_position);
            std::cout << "Updated position command: " << new_position << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid input. Please enter a float value or 'q' to quit." << std::endl;
        }
    }
}

int main(){


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

    // 实例化一个dmMotor对象，使用ID和masterID初始化变量 
    dmMotor motorFR(0x102, 0x022);
    enable_motor(sock, motorFR.getID());

    // starting receving
    std::thread receiver(receiveThread, sock, motorFR);

    // starting sending
    // std::thread sender(sendThread, sock, motorFR);

    // // 启动控制线程
    std::thread control_thread(controlLoop, sock, motorFR.getID());

    // // 启动用户输入线程
    std::thread input_thread(userInputLoop, sock, motorFR.getID());


    receiver.join();
    // sender.join();
    control_thread.join();
    input_thread.join();


    terminate_can(can_id);
    // 关闭套接字
    close(sock);

    return 0;
}