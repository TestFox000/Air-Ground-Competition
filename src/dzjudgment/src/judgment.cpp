#include "judgment.h"

dzjudgment::dzjudgment() : Node("dzjudgment_node")
{
    m_baudrate = 115200;
    // ★ 默认值必须是 udev 稳定软链，不能写死 ttyACMx。
    //   两块板子的 ACM 序号会随插拔/上电顺序互换：
    //     1a86:55d4  QinHeng   = 云台/底盘板  -> /dev/dzactuator
    //     303a:1001  Espressif = 裁判/激光板  -> /dev/dzjudgment
    //   （规则见 /etc/udev/rules.d/dz{judgment,actuator}.rules，按 idVendor/idProduct 固定）
    //
    //   历史事故（2026-10-25 实车）：本节点被 ros2 run 手工拉起时读不到 robot.yaml，
    //   回落到写死的 ttyACM0 —— 正好是云台板，与 dzactuator 抢同一个串口。
    //   后果：开火帧 B5 5B 03 21 01 25 被写进云台板，日志里只有一句"开火指令已下发"，
    //   激光不亮、裁判无记录，串口还每秒刷 "device reports readiness to read"。
    //   改成软链后，即使绕过 launch 直接 ros2 run，也不会再打到隔壁板子。
    m_serialport = "/dev/dzjudgment";

    this->declare_parameter("mcubaudrate", m_baudrate);
    this->get_parameter("mcubaudrate", m_baudrate);
    this->declare_parameter("mcuserialport", m_serialport);
    this->get_parameter("mcuserialport", m_serialport);

    // paramters
    std::cout << FRED("Copyright©2016-2020 duzhong robot. All rights reserved ") << std::endl;
    std::cout << FYEL("*****dzjudgment:parameters*******************") << std::endl;
    std::cout << FGRN("judgment_baudrate: ") << m_baudrate << std::endl;
    std::cout << FGRN("judgment_serialport: ") << m_serialport << std::endl;
    std::cout << FYEL("*****dzjudgment:parameters end***************") << std::endl;

    int retryCount = 3;
    while (retryCount > 0)
    {
        try
        {
            std::cout << "[dzjudgment-->]" << "Serial initialize start!" << std::endl;
            ser.setPort(m_serialport.c_str());
            ser.setBaudrate(m_baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(30);
            ser.setTimeout(to);
            ser.open();
            break;
        }
        catch (serial::IOException &e)
        {
            std::cout << "[dzjudgment-->]" << "Unable to open port! Retrying..." << std::endl;
            retryCount--;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    if (ser.isOpen())
    {
        std::cout << "[dzjudgment-->]" << "Serial initialize successfully!" << std::endl;
    }
    else
    {
        std::cout << "[dzjudgment-->]" << "Serial port failed after multiple attempts!" << std::endl;
        rclcpp::shutdown();
    }

    sub_LaserShot_Command = this->create_subscription<std_msgs::msg::UInt8>(
        "LaserShot_Command", 2, std::bind(&dzjudgment::callback_LaserShot_Command, this, std::placeholders::_1));

    // ★ 裁判系统【通用指令通道】：把收到的字节原样写到串口。
    //   为什么需要它：目前代码里只知道"开火"这一条指令帧 B5 5B 03 21 01 25，
    //   而"开启/关闭裁判系统、重置计数"等指令的字节在硬件协议里（案例9 PDF），
    //   代码里没有。有了这个通道，就能从网页直接下发任意协议帧做验证，
    //   不用每次改 C++ 重新编译。
    sub_referee_cmd = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/referee_cmd", 10, std::bind(&dzjudgment::callback_referee_cmd, this, std::placeholders::_1));

    pub_hpandhismsg = this->create_publisher<std_msgs::msg::UInt8MultiArray>("HpAndHitmsg", 10);
    pub_all_Material_Number = this->create_publisher<std_msgs::msg::UInt8MultiArray>("all_Material_Number", 10);
    pub_enemy_Material_Number = this->create_publisher<std_msgs::msg::UInt8MultiArray>("enemy_Material_Number", 10);
    pub_self_Material_Number = this->create_publisher<std_msgs::msg::UInt8MultiArray>("self_Material_Number", 10);
}

dzjudgment::~dzjudgment()
{
    if (ser.isOpen())
    {
        ser.close();
        RCLCPP_INFO(this->get_logger(), "[dzjudgment-->] Serial port closed.");
    }
}

void dzjudgment::callback_LaserShot_Command(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if (msg->data == 1)
    {
        sendLaserShot();
    }
}

void dzjudgment::callback_referee_cmd(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    if (msg->data.empty())
    {
        RCLCPP_WARN(this->get_logger(), "[dzjudgment-->] /referee_cmd 收到空帧，忽略");
        return;
    }
    // 原样下发，不做任何协议假设
    std::vector<uint8_t> buf = msg->data;
    std::string hex;
    char tmp[8];
    for (size_t i = 0; i < buf.size(); ++i)
    {
        snprintf(tmp, sizeof(tmp), "%02X ", buf[i]);
        hex += tmp;
    }
    try
    {
        const size_t n = ser.write(buf.data(), buf.size());
        if (n != buf.size())
        {
            RCLCPP_ERROR(this->get_logger(), "[dzjudgment-->] 自定义帧只写出 %zu/%zu 字节", n, buf.size());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[dzjudgment-->] ★ 已下发自定义裁判帧: %s", hex.c_str());
        }
    }
    catch (const serial::SerialException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[dzjudgment-->] 自定义帧发送失败: %s", e.what());
    }
}

void dzjudgment::run()
{
    // ★ 原来是 10Hz —— 这是"按下开火后延迟很大"的主因：
    //   本循环里 rclcpp::spin_some() 每 100ms 才执行一次，而开火回调
    //   callback_LaserShot_Command 只有在 spin_some 时才会被触发。
    //   也就是说指令最多要排 100ms 才发出去，加上串口耗时，手感就是"慢半拍"。
    //   提到 100Hz 后，指令到串口的排队延迟 ≤10ms。
    //   （串口读取本身是非阻塞有界的，见 recvESP32Info，提高频率不会空转烧 CPU。）
    int run_rate = 100; // HZ
    rclcpp::Rate rate(run_rate);
    while (rclcpp::ok())
    {
        fflush(stdout);
        rclcpp::spin_some(this->get_node_base_interface());
        try
        {
            recvESP32Info();
        }
        // ★ 用 SerialException 而不是 IOException：IOException 是它的子类，
        //   而串口库还会抛 SerialException 本身（端口消失/写失败等）。
        //   只 catch 子类会漏掉父类异常 -> 未捕获 -> 进程 SIGABRT（dzactuator 已踩过同样的坑）。
        catch (const serial::SerialException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "[dzjudgment-->] Serial communication error: %s. Trying to reconnect...", e.what());
            if (!reconnectSerial())
            {
                RCLCPP_ERROR(this->get_logger(), "[dzjudgment-->] Failed to reconnect serial port.");
            }
        }

        // 每 100 个 tick（100Hz 下约 1 秒）主动上报一次裁判状态。
        // 目的：裁判板没推 0x07 帧时（比如还没开启裁判系统），网页面板也能显示
        //       "当前已知值"，一眼区分是"未开启"还是"有数据"。
        if (++heartbeat_ticks_ >= 100)
        {
            heartbeat_ticks_ = 0;
            publish_referee_state();
        }

        rate.sleep();
    }
}

bool dzjudgment::reconnectSerial()
{
    if (ser.isOpen())
    {
        ser.close();
    }

    int retryCount = 3;
    while (rclcpp::ok())
    {
        try
        {
            RCLCPP_INFO(this->get_logger(), "[dzjudgment-->] Reconnecting serial port...");
            ser.setPort(m_serialport.c_str());
            ser.setBaudrate(m_baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(30);
            ser.setTimeout(to);
            ser.open();
            RCLCPP_INFO(this->get_logger(), "[dzjudgment-->] Serial port reconnected successfully!");
            return true;
        }
        catch (serial::IOException &e)
        {
            RCLCPP_WARN(this->get_logger(), "[dzjudgment-->] Unable to reconnect port! Retrying...");
            retryCount--;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    return false;
}

void dzjudgment::publish_referee_state()
{
    std_msgs::msg::UInt8MultiArray msg;
    msg.data.clear();
    msg.data.push_back(targethp);       // [0] 对方血量
    msg.data.push_back(self_hp);        // [1] 己方血量
    msg.data.push_back(Bing_hit);       // [2] 被击打方位 0x10=未被击打
    msg.data.push_back(Shooting_Count); // [3] 累计射击次数（硬件侧）
    msg.data.push_back(ammo);           // [4] 剩余弹药
    pub_hpandhismsg->publish(msg);
}

void dzjudgment::recvESP32Info()
{
    // =====================================================================
    //  ★★★ 这里曾经是全项目最阴的一个 bug，务必保留这段说明 ★★★
    //
    //  旧写法是 while (true && rclcpp::ok()) 死等一个完整帧，只有
    //  header1==0xB5 && header2==0x5B 且帧体读齐 时才 break。
    //
    //  问题出在"串口安静的时候"（裁判板没上电 / 线松了 / 就是没在发数据）：
    //      ser.available() 恒为 0，header1/header2 是【未初始化】的栈变量，
    //      这个循环永远不会 break，于是 ——
    //        · CPU 直接跑满（实测 102%）
    //        · 更要命：run() 里那个 10Hz 循环被彻底卡死，
    //          rclcpp::spin_some() 再也不会被调用
    //          → 所有订阅回调失效 → 开火指令收不到 → 激光不亮
    //      而 ros2 topic info 依然显示 Subscription count: 1，
    //      看起来一切正常，排查时极具迷惑性。
    //
    //  修法：改成【有界 + 非阻塞】——
    //        · 串口没数据立刻收工，把控制权还给 run() 去 spin
    //        · 每次最多处理 8 帧，防止对端灌数据时又把自己卡住
    //        · 半帧状态靠成员 previousHeader2 跨调用保留
    // =====================================================================
    for (int guard = 0; guard < 8; ++guard)
    {
        if (ser.available() <= 0)
        {
            return;                     // 没数据就立刻让出，绝不空转
        }

        uint8_t header1 = 0, header2 = 0;

        if (previousHeader2 == 0xB5)
        {
            header1 = previousHeader2;
        }
        else
        {
            while (ser.available() > 0)
            {
                ser.read(&header1, 1);
                if (header1 == 0xB5)
                    break;
            }
        }

        if (ser.available() <= 0)
        {
            return;                     // 只有帧头，帧体下次再读
        }
        ser.read(&header2, 1);
        previousHeader2 = header2;

        if (header1 != 0xB5 || header2 != 0x5B)
        {
            continue;                   // 不是帧头，接着找
        }

        if (ser.available() <= 0)
        {
            return;                     // 长度字节还没到
        }
        uint8_t dataLength;
        ser.read(&dataLength, 1);

        if (ser.available() < static_cast<size_t>(dataLength))
        {
            return;                     // 帧体没齐，等下个周期凑齐
        }

        uint8_t charArray[dataLength + 3];
        charArray[0] = header1;
        charArray[1] = header2;
        charArray[2] = dataLength;
        ser.read(&charArray[3], dataLength);

        processData(charArray, dataLength);
    }
}

void dzjudgment::processData(uint8_t *charArray, uint8_t dataLength)
{

    if (charArray[0] == 0xB5 && charArray[1] == 0x5B)
    {
        validDataLength = dataLength - 2;
        if (validDataLength > MAX_VALID_DATA_LENGTH)
        {
            RCLCPP_ERROR(this->get_logger(), "[dzjudgment-->] Data length exceeds maximum limit!");
            return;
        }

        std_msgs::msg::UInt8MultiArray msg;
        msg.data.clear();

        switch (charArray[3])
        {
        case 0x02:
            memset(all_Material_Number, 0, sizeof(all_Material_Number));
            for (int i = 0; i < validDataLength; i++)
            {
                all_Material_Number[i] = charArray[i + 4];
                msg.data.push_back(all_Material_Number[i]);
            }
            pub_all_Material_Number->publish(msg);
            break;
        case 0x03:
            memset(enemy_Material_Number, 0, sizeof(enemy_Material_Number));
            for (int i = 0; i < validDataLength; i++)
            {
                enemy_Material_Number[i] = charArray[i + 4];
                msg.data.push_back(enemy_Material_Number[i]);
            }
            pub_enemy_Material_Number->publish(msg);
            break;
        case 0x04:
            memset(self_Material_Number, 0, sizeof(self_Material_Number));
            for (int i = 0; i < validDataLength; i++)
            {
                self_Material_Number[i] = charArray[i + 4];
                msg.data.push_back(self_Material_Number[i]);
            }
            pub_self_Material_Number->publish(msg);
            break;
        case 0x07:
            // ★ 长度守卫(code_review C5)：下面无条件读 charArray[4..8]，
            //   但 charArray 只有 dataLength+3 字节，帧体过短时会读到越界栈内存
            //   (垃圾血量/弹药，甚至崩溃)。读满 charArray[8] 要求 dataLength >= 6。
            if (dataLength < 6)
            {
                RCLCPP_WARN(this->get_logger(),
                            "[dzjudgment-->] case 0x07 帧体过短(dataLength=%u < 6)，已丢弃",
                            dataLength);
                break;
            }
            targethp = charArray[4];
            self_hp = charArray[5];
            Bing_hit = charArray[6];
            Shooting_Count = charArray[7];
            ammo = charArray[8];
            msg.data.push_back(charArray[4]);
            msg.data.push_back(charArray[5]);
            msg.data.push_back(charArray[6]);
            msg.data.push_back(charArray[7]);
            msg.data.push_back(charArray[8]);
            pub_hpandhismsg->publish(msg);
            break;
        default:
            break;
        }
    }
}

void dzjudgment::sendLaserShot()
{
    unsigned char buf[6] = {0};
    buf[0] = 0xB5; // hdr1
    buf[1] = 0x5B; // hdr2
    buf[2] = 0x03; // len
    buf[3] = 0x21; //
    buf[4] = 0x01; //
    buf[5] = 0x25; // 校验位
#if 0
    printf("write to 32: ");
    for(int i = 2; i < writesize;i++){
    printf("buf[%d] = %02x ,",i,buf[i]);
    }
    printf("\n");
#endif

    try
    {
        size_t writesize = ser.write(buf, 6);
        if (writesize != 6)
        {
            RCLCPP_ERROR(this->get_logger(), "[dzjudgment-->] Failed to send laser shot command!");
        }
        else
        {
            // ★ 成功也要有日志 —— 这点很关键：
            //   开火是个"一次性动作"，如果成功时静默，出问题就完全分不清
            //     "指令压根没送到" 还是 "送到了但激光/裁判模块没响应"。
            //   有了这行，排查就变成一眼的事：
            //     日志里有 "★开火" -> 指令链路 OK，问题在硬件侧
            //     日志里没有      -> 话题没人发/没人订阅，查 /LaserShot_Command
            RCLCPP_WARN(this->get_logger(),
                        "[dzjudgment-->] ★ 开火指令已下发 (B5 5B 03 21 01 25)");
        }
    }
    catch (const serial::SerialException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[dzjudgment-->] Serial communication error while sending laser shot command: %s. Trying to reconnect...", e.what());
        if (!reconnectSerial())
        {
            RCLCPP_ERROR(this->get_logger(), "[dzjudgment-->] Failed to reconnect serial port.");
        }
    }
}