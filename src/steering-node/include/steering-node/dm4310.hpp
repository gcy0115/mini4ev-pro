#include <iostream>
#include <stdexcept>
#include <vector>
#include <cstdint>
#include <linux/can.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstdlib>  // stdlib.h
#include <iostream>
#include <thread>
#include <cstring>
#include <cerrno>

#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

const float Pos_min = -12.5, Pos_max = 12.5;
const float Vel_min = -30.0, Vel_max = 30.0;
const float Tor_min = -18.0, Tor_max = 18.0;


float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max- x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


int float_to_uint(float x,float x_min, float x_max, int bits){
    // Converts afloat to anunsigned int, given range and number ofbits

    float span = x_max-x_min;
    float offset =x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}


// void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan){

//     p_int=(_hcan->pRxMsg->Data[1]<<8)|_hcan->pRxMsg->Data[2];
//     v_int=(_hcan->pRxMsg->Data[3]<<4)|(_hcan->pRxMsg->Data[4]>>4);
//     t_int=((_hcan->pRxMsg->Data[4]&0xF)<<8)|_hcan->pRxMsg->Data[5];
//     position = uint_to_float(p_int, P_MIN, P_MAX, 16);// (-12.5,12.5)
//     velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);// (-45.0,45.0)
//     torque = uint_to_float(t_int, T_MIN, T_MAX, 12); //(-18.0,18.0)
    
//     /*#### add enable can it again to solve can receive only one ID problem!!!####**/
//     __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
// }

class dmMotor {
private:
    int ID;                // 电机的 CAN ID
    int master_ID;
    float p_des;           // 期望位置
    float v_des;           // 期望速度
    float Kp;              // 位置比例系数
    float Kd;              // 位置微分系数
    float T_ff;            // 转矩给定值
    float POS;             // 实际位置
    float VEL;             // 实际速度
    float T;               // 电机的扭矩信息
    int T_MOS;             // 驱动器温度
    int T_Rotor;           // 电机线圈温度
    int ERR;               // 状态信息，
                                // 0——失能；
                                // 1——使能；
                                // 8——超压；
                                // 9——欠压；
                                // A——过电流；
                                // B——MOS过温；
                                // C——电机线圈过温；
                                // D——通讯丢失；
                                // E——过载；

    public:
    // 构造函数，初始化各个参数
    // 初始化电机时，应直接给定其ID和masterID
    dmMotor(int id, int master_id, float p_des = 0.0f, float v_des = 0.0f, float kp = 0.0f, float kd = 0.0f, float t_ff = 0.0f,
          float pos = 0.0f, float vel = 0.0f, float t = 0.0f, int t_mos = 0, int t_rotor = 0, int err = 0)
        : ID(id), master_ID(master_id), p_des(p_des), v_des(v_des), Kp(kp), Kd(kd), T_ff(t_ff),
          POS(pos), VEL(vel), T(t), T_MOS(t_mos), T_Rotor(t_rotor), ERR(err) {}

    // Getter 和 Setter 方法
    int getID() const { return ID; }
    void setID(int id) { ID = id; }

    int getMasterID() const { return master_ID; }
    void setMasterID(int master_id) { master_ID = master_id; }

    float getPDes() const { return p_des; }
    void setPDes(float p) { p_des = p; }

    float getVDes() const { return v_des; }
    void setVDes(float v) { v_des = v; }

    float getKp() const { return Kp; }
    void setKp(float kp) { Kp = kp; }

    float getKd() const { return Kd; }
    void setKd(float kd) { Kd = kd; }

    float getTFF() const { return T_ff; }
    void setTFF(float t_ff) { T_ff = t_ff; }

    float getPOS() const { return POS; }
    void setPOS(float pos) { POS = pos; }

    float getVEL() const { return VEL; }
    void setVEL(float vel) { VEL = vel; }

    int getT() const { return T; }
    void setT(int t) { T = t; }

    int getTMOS() const { return T_MOS; }
    void setTMOS(int t_mos) { T_MOS = t_mos; }

    float getTRotor() const { return T_Rotor; }
    void setTRotor(float t_rotor) { T_Rotor = t_rotor; }

    int getERR() const { return ERR; }
    void setERR(int err) { ERR = err; }
    
};

// ------------------------DEMO----------------------------//
// int main() {
//     Motor motor(1);  // 创建一个 ID 为 1 的电机实例

//     motor.setPDes(100.0f);  // 设置期望位置
//     motor.setKp(1.2f);      // 设置位置比例系数

//     int E = 5;  // 假设状态信息最大值为 E = 5
//     try {
//         motor.setERR(3, E);  // 设置状态信息
//     } catch (const std::out_of_range& e) {
//         std::cerr << "Error: " << e.what() << std::endl;
//     }

//     std::cout << "Motor ID: " << motor.getID() << std::endl;
//     std::cout << "Desired Position: " << motor.getPDes() << std::endl;
//     std::cout << "Kp: " << motor.getKp() << std::endl;
//     std::cout << "Error Status (ERR): " << motor.getERR() << std::endl;

//     return 0;
// }

// 解析 CAN 总线上的反馈信息


int PosControlFrame(int sock, int motor_id, float p_des, float v_des) {
    // 将浮点数指针转为字节指针
    uint8_t *pbuf, *vbuf;
    pbuf = reinterpret_cast<uint8_t*>(&p_des);
    vbuf = reinterpret_cast<uint8_t*>(&v_des);

    // 填充 CAN 帧
    struct can_frame pos_control_frame;
    pos_control_frame.can_id = motor_id;  // 添加基础 ID 偏移
    pos_control_frame.can_dlc = 8;               // 数据长度（固定为 8 字节）
    pos_control_frame.data[0] = pbuf[0];         // 填充 p_des 数据
    pos_control_frame.data[1] = pbuf[1];
    pos_control_frame.data[2] = pbuf[2];
    pos_control_frame.data[3] = pbuf[3];
    pos_control_frame.data[4] = vbuf[0];         // 填充 v_des 数据
    pos_control_frame.data[5] = vbuf[1];
    pos_control_frame.data[6] = vbuf[2];
    pos_control_frame.data[7] = vbuf[3];

    // 发送 CAN 帧
    ssize_t nbytes = write(sock, &pos_control_frame, sizeof(pos_control_frame));
    // std::cout << "Sending control frame" << std::endl;
    if (nbytes != sizeof(pos_control_frame)) {
        std::cerr << "Error: Failed to send CAN frame (bytes sent: " << nbytes
                  << ", expected: " << sizeof(pos_control_frame) << ")" << std::endl;
        return -1;  // 返回错误码
    }

    return 0;  // 成功返回 0
}


struct MotorFeedback {
    int motorID;      // 电机 ID
    int ERR;          // 错误信息
    float POS;          // 16 位实际位置
    float VEL;          // 12 位实际速度
    float T;            // 12 位扭矩
    uint8_t T_MOS;    // 8 位驱动器温度
    uint8_t T_Rotor;  // 8 位电机线圈温度
};
MotorFeedback parseCANFeedback(int can_id, const std::vector<uint8_t>& data) {
    if (data.size() < 8) {
        throw std::invalid_argument("CAN 数据长度不足 8 字节");
    }

    MotorFeedback feedback;

    // 第1位：低 4 位为电机 ID，高 4 位为 ERR 信息ERR

    feedback.motorID = data[0] & 0x0F;        // 提取ID 信息
    feedback.ERR = data[0] >> 4;  // 提取high 4 位的电机 ERR

    // 第2、3位：POS 高 8 位和低 8 位，总长 16 位
    // feedback.POS = (data[1] << 8) | data[2];
    feedback.POS = uint_to_float((data[1] << 8) | data[2], Pos_min, Pos_max, 16);
    // std::cout << "POS_Raw: " << ((data[1] << 8) | data[2]) << std::endl;
    // std::cout << "POS_2_f: " << feedback.POS << std::endl;

    // 第4、5位：VEL 的高 8 位和低 4 位，总长 12 位
    feedback.VEL = uint_to_float((data[3] << 4) | (data[4] >> 4), Vel_min, Vel_max, 12);

    // 第5位（低 4 位）和第6位：T 的高 4 位和低 8 位，总长 12 位
    feedback.T = uint_to_float(((data[4] & 0x0F) << 8) | data[5], Tor_min, Tor_max, 12);

    // 第7位：T_MOS（8 位）
    feedback.T_MOS = data[6];

    // 第8位：T_Rotor（8 位）
    feedback.T_Rotor = data[7];

    return feedback;
}


// 测试
// int main() {
//     // 模拟 CAN 数据
//     int can_id = 123;
//     std::vector<uint8_t> data = { 0x01, 0x00, 0x01, 0x12, 0x34, 0x56, 0x78, 0x9A };

//     try {
//         MotorFeedback feedback = parseCANFeedback(can_id, data);

//         std::cout << "Motor ID: " << feedback.motorID << std::endl;
//         std::cout << "ERR: " << feedback.ERR << std::endl;
//         std::cout << "POS: " << feedback.POS << std::endl;
//         std::cout << "VEL: " << feedback.VEL << std::endl;
//         std::cout << "T: " << feedback.T << std::endl;
//         std::cout << "T_MOS: " << static_cast<int>(feedback.T_MOS) << std::endl;
//         std::cout << "T_Rotor: " << static_cast<int>(feedback.T_Rotor) << std::endl;
//     } catch (const std::exception& e) {
//         std::cerr << "Error parsing CAN feedback: " << e.what() << std::endl;
//     }

//     return 0;
// }

// enable motor
int enable_motor(int sock, int motor_id){

    struct can_frame enable_frame;
        enable_frame.can_id = motor_id;  // 设置 CAN ID
        enable_frame.can_dlc = 8;     // 数据长度（0-8）
        enable_frame.data[0] = 0xFF;  // 填充数据
        enable_frame.data[1] = 0xFF;
        enable_frame.data[2] = 0xFF;
        enable_frame.data[3] = 0xFF;
        enable_frame.data[4] = 0xFF;
        enable_frame.data[5] = 0xFF;
        enable_frame.data[6] = 0xFF;
        enable_frame.data[7] = 0xFC;

        write(sock, &enable_frame, sizeof(enable_frame));
        // std::cout << "Enable motor " << motor_id << ": " << std::endl;

    return 0;

}

// enable motor
int disable_motor(int sock, int motor_id){

    struct can_frame enable_frame;
        enable_frame.can_id = motor_id;  // 设置 CAN ID
        enable_frame.can_dlc = 8;     // 数据长度（0-8）
        enable_frame.data[0] = 0xFF;  // 填充数据
        enable_frame.data[1] = 0xFF;
        enable_frame.data[2] = 0xFF;
        enable_frame.data[3] = 0xFF;
        enable_frame.data[4] = 0xFF;
        enable_frame.data[5] = 0xFF;
        enable_frame.data[6] = 0xFF;
        enable_frame.data[7] = 0xFD;

        write(sock, &enable_frame, sizeof(enable_frame));
        // std::cout << "disable motor " << motor_id << ": " << std::endl;

    return 0;

}

// set current postion as zero point
int setzero_motor(int sock, int motor_id){

    struct can_frame enable_frame;
        enable_frame.can_id = motor_id;  // 设置 CAN ID
        enable_frame.can_dlc = 8;     // 数据长度（0-8）
        enable_frame.data[0] = 0xFF;  // 填充数据
        enable_frame.data[1] = 0xFF;
        enable_frame.data[2] = 0xFF;
        enable_frame.data[3] = 0xFF;
        enable_frame.data[4] = 0xFF;
        enable_frame.data[5] = 0xFF;
        enable_frame.data[6] = 0xFF;
        enable_frame.data[7] = 0xFE;

        write(sock, &enable_frame, sizeof(enable_frame));
        // std::cout << "Send zero " << motor_id << ": " << std::endl;

    return 0;

}

// 快速检查收发问题
void sendThread(int sock, dmMotor motor) {
    while (true) {
        enable_motor(sock, motor.getID());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 控制发送频率
        disable_motor(sock, motor.getID());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}