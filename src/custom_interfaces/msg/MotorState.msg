int64 id            # 电机编号，CAN使用CANid，EtherCAT使用物理地址或逻辑地址
string name         # 电机名称
float32 position    # 电机当前位置
float32 velocity    # 电机当前速度
float32 torque      # 电机当前力矩
float32 temperature # 电机当前温度
int64 err           # 电机当前ERR码