// test socket CAN via cpp
#include <iostream>
#include <thread>

#include "include/can_opration.hpp"
#include "include/dm4310.hpp"

int main(){


    // const char* can_interface = "can0";  // CAN 接口名称
    std::string can_id = "can0";
    initial_can(can_id);

    int sock = openCANSocket(can_id.c_str());
    // .c_str() 返回一个 const char* 指针，指向 std::string 中的字符数组，并在末尾加上空字符 \0。c_str() 返回的指针是只读的，不应被修改，否则会导致未定义行为。
    if (sock < 0) {
        return -1;
    }

    // 实例化一个dmMotor对象，使用ID和masterID初始化变量 
    dmMotor motorFR(0x102, 0x22);

    // starting sending
    disable_motor(sock, motorFR.getID());


    terminate_can(can_id);
    // 关闭套接字
    close(sock);

    return 0;
}