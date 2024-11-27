#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <memory>  // 使用 std::shared_ptr

// 模拟电机类，包含控制和反馈内容
class Motor {
public:
    int id;                  // 电机 ID
    float p_des;             // 目标位置
    float v_des;             // 目标速度
    float kp, kd;            // 控制系数
    float t_ff;              // 前馈转矩

    float pos, vel, torque;  // 反馈内容：位置、速度、扭矩

    Motor(int id) : id(id), p_des(0), v_des(0), kp(0), kd(0), t_ff(0), pos(0), vel(0), torque(0) {}
};

// 共享队列，用于线程通信
std::queue<std::shared_ptr<Motor>> control_queue;
std::queue<std::shared_ptr<Motor>> feedback_queue;
std::mutex mtx;
std::condition_variable cv_control, cv_feedback;

// 第一个线程：生成控制目标
void generateControlGoal(int motor_id) {
    while (true) {
        std::shared_ptr<Motor> motor = std::make_shared<Motor>(motor_id);  // 使用 shared_ptr
        motor->p_des = static_cast<float>(rand() % 100) / 10.0;  // 随机目标位置
        motor->v_des = static_cast<float>(rand() % 50) / 10.0;   // 随机目标速度
        motor->kp = 1.0f;  // 固定比例控制系数
        motor->kd = 0.5f;  // 固定微分控制系数
        motor->t_ff = 0.1f; // 固定前馈转矩

        // 将控制目标放入队列
        {
            std::lock_guard<std::mutex> lock(mtx);
            control_queue.push(motor);
            std::cout << "Generated control goal for motor " << motor_id << " | p_des: " << motor->p_des << ", v_des: " << motor->v_des << std::endl;
        }
        cv_control.notify_one(); // 通知等待的线程
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟生成目标的间隔
    }
}

// 第二个线程：接收电机反馈帧
void receiveMotorFeedback(int motor_id) {
    while (true) {
        std::shared_ptr<Motor> motor = std::make_shared<Motor>(motor_id);

        // 模拟反馈数据
        motor->pos = static_cast<float>(rand() % 100) / 10.0;   // 随机位置
        motor->vel = static_cast<float>(rand() % 50) / 10.0;    // 随机速度
        motor->torque = static_cast<float>(rand() % 20) / 10.0; // 随机扭矩

        // 将反馈数据放入队列
        {
            std::lock_guard<std::mutex> lock(mtx);
            feedback_queue.push(motor);
            std::cout << "Received feedback for motor " << motor_id << " | pos: " << motor->pos << ", vel: " << motor->vel << ", torque: " << motor->torque << std::endl;
        }
        cv_feedback.notify_one(); // 通知等待的线程
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟接收反馈帧的间隔
    }
}

// 第三个线程：处理控制目标和反馈帧
void processControlAndFeedback(int motor_id) {
    while (true) {
        std::shared_ptr<Motor> control_motor, feedback_motor;

        // 从控制目标队列中获取数据
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv_control.wait(lock, [] { return !control_queue.empty(); }); // 等待有数据
            control_motor = control_queue.front();
            control_queue.pop();
        }

        // 从反馈帧队列中获取数据
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv_feedback.wait(lock, [] { return !feedback_queue.empty(); }); // 等待有数据
            feedback_motor = feedback_queue.front();
            feedback_queue.pop();
        }

        // 处理控制目标和反馈数据
        std::cout << "Processing motor " << motor_id << " | Control Goal -> p_des: " << control_motor->p_des
                  << ", Feedback -> pos: " << feedback_motor->pos << std::endl;

        // 模拟处理时间
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 主线程：访问电机对象的参数
void monitorMotor(int motor_id) {
    while (true) {
        std::shared_ptr<Motor> motor_control, motor_feedback;

        // 从控制队列获取最新的控制目标
        {
            std::unique_lock<std::mutex> lock(mtx);
            if (!control_queue.empty()) {
                motor_control = control_queue.front();
                control_queue.pop();
                std::cout << "Main thread - Control Target: p_des = " << motor_control->p_des << ", v_des = " << motor_control->v_des << std::endl;
            }
        }

        // 从反馈队列获取最新的反馈数据
        {
            std::unique_lock<std::mutex> lock(mtx);
            if (!feedback_queue.empty()) {
                motor_feedback = feedback_queue.front();
                feedback_queue.pop();
                std::cout << "Main thread - Feedback: pos = " << motor_feedback->pos << ", vel = " << motor_feedback->vel << ", torque = " << motor_feedback->torque << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 每隔2秒输出一次
    }
}

int main() {
    int motor_id = 1;

    // 创建三个线程
    std::thread control_thread(generateControlGoal, motor_id);
    std::thread feedback_thread(receiveMotorFeedback, motor_id);
    std::thread processing_thread(processControlAndFeedback, motor_id);
    std::thread monitor_thread(monitorMotor, motor_id); // 用于主线程中访问电机数据

    // 等待线程完成（此处实际上是一个无限循环）
    control_thread.join();
    feedback_thread.join();
    processing_thread.join();
    monitor_thread.join();

    return 0;
}


/*
Generated control goal for motor 1 | p_des: 7.3, v_des: 2.1
Received feedback for motor 1 | pos: 4.8, vel: 2.2, torque: 1.6
Processing motor 1 | Control Goal -> p_des: 7.3, Feedback -> pos: 4.8
Main thread - Control Target: p_des = 7.3, v_des = 2.1
Main thread - Feedback: pos = 4.8, vel = 2.2, torque = 1.6
Generated control goal for motor 1 | p_des: 4.5, v_des: 1.2
Received feedback for motor 1 | pos: 2.1, vel: 0.9, torque: 0.

*/