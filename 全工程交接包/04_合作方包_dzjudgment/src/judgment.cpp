#include "judgment.h"

dzjudgment::dzjudgment() : Node("dzjudgment_node")
{
    m_baudrate = 115200;
    m_serialport = "/dev/ttyACM0";

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

void dzjudgment::run()
{
    int run_rate = 10; // HZ
    rclcpp::Rate rate(run_rate);
    while (rclcpp::ok())
    {
        fflush(stdout);
        rclcpp::spin_some(this->get_node_base_interface());
        try
        {
            recvESP32Info();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "[dzjudgment-->] Serial communication error: %s. Trying to reconnect...", e.what());
            if (!reconnectSerial())
            {
                RCLCPP_ERROR(this->get_logger(), "[dzjudgment-->] Failed to reconnect serial port.");
            }
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

void dzjudgment::recvESP32Info()
{
    while (true && rclcpp::ok())
    {
        uint8_t header1, header2;

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

        if (ser.available() > 0)
        {
            ser.read(&header2, 1);
            previousHeader2 = header2;
        }
        // printf("header1 = %02x,header2 = %02x\n",header1,header2);

        if (header1 == 0xB5 && header2 == 0x5B)
        { 
            uint8_t dataLength;
            if (ser.available() > 0)
            {
                ser.read(&dataLength, 1);
                if (ser.available() >= static_cast<size_t>(dataLength))
                {
                    uint8_t charArray[dataLength + 3];
                    charArray[0] = header1;
                    charArray[1] = header2;
                    charArray[2] = dataLength;
                    ser.read(&charArray[3], dataLength);

                    processData(charArray, dataLength);
                    break;
                }
            }
        }
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
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[dzjudgment-->] Serial communication error while sending laser shot command: %s. Trying to reconnect...", e.what());
        if (!reconnectSerial())
        {
            RCLCPP_ERROR(this->get_logger(), "[dzjudgment-->] Failed to reconnect serial port.");
        }
    }
}