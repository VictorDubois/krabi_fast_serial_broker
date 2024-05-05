#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <iomanip>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "krabi_msgs/msg/encoders.hpp"
#include "krabi_msgs/msg/motors_cmd.hpp"
#include "krabi_msgs/msg/motors_parameters.hpp"
#include "krabi_msgs/msg/odom_lighter.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <bitset>


using namespace std::chrono_literals;

class SimpleBrokerSTM32 : public rclcpp::Node
{
public:
    SimpleBrokerSTM32() : Node("simple_broker_stm32")
    {
        subscription_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&SimpleBrokerSTM32::callback_cmd_vel, this, std::placeholders::_1));

        subscription_motors_cmd_ = this->create_subscription<krabi_msgs::msg::MotorsCmd>(
            "motors_cmd", 10, std::bind(&SimpleBrokerSTM32::callback_motors_cmd, this, std::placeholders::_1));

        subscription_motors_parameters_ = this->create_subscription<krabi_msgs::msg::MotorsParameters>(
            "motors_parameters", 10, std::bind(&SimpleBrokerSTM32::callback_motors_parameters, this, std::placeholders::_1));

        subscription_enable_motor_ = this->create_subscription<std_msgs::msg::Bool>(
            "enable_motor", 10, std::bind(&SimpleBrokerSTM32::callback_enable_motors, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<krabi_msgs::msg::OdomLighter>("odom_lighter", 10);
        encoders_pub_ = this->create_publisher<krabi_msgs::msg::Encoders>("encoders", 10);

        timer_ = this->create_wall_timer(10ms, std::bind(&SimpleBrokerSTM32::read_serial, this));

        serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
        //serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "open_port: Unable to open /dev/ttyACM0 - ");
        }
        else
        {
            fcntl(serial_fd_, F_SETFL, 0);

            configure_serial_port(serial_fd_);
            /*termios options;
            tcgetattr(serial_fd_, &options);
            cfsetispeed(&options, B115200);
            cfsetospeed(&options, B115200);
            options.c_cflag |= (CLOCAL | CREAD);
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS8;
            options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            options.c_oflag &= ~OPOST;
            options.c_cc[VMIN] = 1;
            options.c_cc[VTIME] = 0;
            tcsetattr(serial_fd_, TCSANOW, &options);*/
        }
    }

    ~SimpleBrokerSTM32()
    {
        if (serial_fd_ != -1)
        {
            close(serial_fd_);
        }
    }

private:

    // Define CRC parameters
    #define CRC_POLYNOMIAL 0x1021
    #define CRC_INITIAL_VALUE 0xFFFF

    // Function to calculate CRC
    uint16_t calculate_crc(const uint8_t *data, int length) {
        uint16_t crc = CRC_INITIAL_VALUE;
        
        for (int i = 0; i < length; i++) {
            crc ^= data[i] << 8;
            for (int j = 0; j < 8; j++) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ CRC_POLYNOMIAL;
                } else {
                    crc <<= 1;
                }
            }
        }
        
        return crc;
    }

    void configure_serial_port(int serial_fd)
    {
        termios options;
        tcgetattr(serial_fd, &options);

        // Set baud rate
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);

        // Set data bits
        options.c_cflag &= ~CSIZE; // Clear data size bits
        options.c_cflag |= CS8;    // Set 8 data bits

        // Set parity
        options.c_cflag &= ~PARENB; // Disable parity

        // Set stop bits
        options.c_cflag &= ~CSTOPB; // Set 1 stop bit

        // Disable hardware flow control
        options.c_cflag &= ~CRTSCTS;

        // Set terminal to raw mode
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;

        // Set minimum characters to read
        options.c_cc[VMIN] = 0;
        // Set timeout for read in deciseconds
        options.c_cc[VTIME] = 1;

        // Apply the settings
        tcsetattr(serial_fd, TCSANOW, &options);
    }
    void callback_enable_motors(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::vector<float> float_array_to_send;
        float_array_to_send.push_back(msg->data);
        std::cout << float_array_to_send[0] << std::endl;

        write_float_to_serial('e', float_array_to_send);
    }

    void callback_motors_parameters(const krabi_msgs::msg::MotorsParameters::SharedPtr msg)
    {
        std::vector<float> float_array_to_send;
        float_array_to_send.push_back(msg->max_current_left);
        float_array_to_send.push_back(msg->max_current_right);
        float_array_to_send.push_back(msg->max_current);

        //RCLCPP_WARN_STREAM(this->get_logger(),  float_array_to_send[0] << ", " << float_array_to_send[1] << ", " << float_array_to_send[2]);

        write_float_to_serial('p', float_array_to_send);
    }

    void callback_motors_cmd(const krabi_msgs::msg::MotorsCmd::SharedPtr msg)
    {
        std::vector<float> float_array_to_send;
        float_array_to_send.push_back(msg->enable_motors);
        float_array_to_send.push_back(msg->override_pwm);
        float_array_to_send.push_back(msg->pwm_override_left);
        float_array_to_send.push_back(msg->pwm_override_right);
        float_array_to_send.push_back(msg->reset_encoders);
        //RCLCPP_WARN_STREAM(this->get_logger(),  float_array_to_send[0] << ", " << float_array_to_send[1] << ", " << float_array_to_send[2]);
 
        write_float_to_serial('c', float_array_to_send);
    }

    void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::vector<float> float_array_to_send;
        float_array_to_send.push_back(msg->linear.x);
        float_array_to_send.push_back(msg->angular.z);
        //RCLCPP_WARN_STREAM(this->get_logger(),  float_array_to_send[0] << ", " << float_array_to_send[1]);

        write_float_to_serial('v', float_array_to_send);
    }

    /*void float_to_hex(float float_value, std::string &hex_value)
    {
        std::stringstream stream;
        stream << std::hex << std::setfill('0') << std::setw(8) << std::uppercase << std::bitset<32>(*(uint32_t *)&float_value);
        hex_value = stream.str();
    }*/

    void float_to_hex(const float a_value, uint8_t* a_out, const int start_pos)
    {
        uint32_t floatHex;
        memcpy(&floatHex, &a_value, sizeof(a_value));
        sprintf((char*)(a_out + start_pos), "%08X", floatHex); // Convert to hexadecimal string
    }

    void write_float_to_serial(char channel_name, const std::vector<float> &float_values)
    {
        uint8_t hex_values[10 * 8] = {0}; // Initialize to 0
        for (size_t i = 0; i < float_values.size(); ++i)
        {
            float_to_hex(float_values[i], hex_values, i * 8);
        }
        // Send data over serial
        std::string message = std::string(1, channel_name) + std::string(reinterpret_cast<char*>(hex_values), sizeof(hex_values)) + "\n";
        ssize_t n = write(serial_fd_, message.c_str(), message.length());
        if (n < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "write_float_to_serial: Error writing to serial");
        }
        else
        {
            //RCLCPP_WARN(this->get_logger(), "Sending: %s", message.c_str());
            //std::cout << "Sending: " << channel_name << std::string(reinterpret_cast<char*>(hex_values), sizeof(hex_values)) << std::endl;
            //std::cout.flush();
            std::this_thread::sleep_for(1ms);
        }
    }


    /*void write_float_to_serial(char channel_name, const std::vector<float> &float_values)
    {
        std::string hex_values;
        for (float float_value : float_values)
        {
            std::string hex;
            float_to_hex(float_value, hex);
            hex_values += hex;
        }
        
        // Pad to 10 values
        hex_values.resize(10 * 8, '0');
        std::string message = std::string(1, channel_name) + hex_values + "\n";
        write(serial_fd_, message.c_str(), message.length());
        RCLCPP_WARN(this->get_logger(), "Sending: %c%s", channel_name, hex_values.c_str());
        RCLCPP_WARN_STREAM(this->get_logger(), "Sending: " << channel_name << std::hex << hex_values.c_str());
        std::cout << "Sending: " << channel_name << hex_values << std::endl;
        std::cout.flush();
        std::this_thread::sleep_for(1ms);
    }*/

    void get_float(const std::string &a_string, int a_beginning, float &float_value)
    {
        
        std::string hex_value = a_string.substr(a_beginning, 8); // Extract hexadecimal value
        std::cout << hex_value << std::endl<< std::flush;
        uint32_t float_hex = strtoul(hex_value.c_str(), NULL, 16); // Convert hexadecimal string to unsigned long
        //float_value = 0.f;
        //uint32_t float_hex = std::stoul(hex_value, nullptr, 16); // Convert hexadecimal string to integer
        float_value = *(float *)&float_hex;                       // Convert integer to float
    }

    std::string read_line_from_serial()
    {
        std::string line;
        char buffer;
        ssize_t n;

        do
        {
            n = read(serial_fd_, &buffer, 1);
            if (n > 0)
            {
                line += buffer;
            }
        } while (n > 0 && buffer != '\n');

        return line;
    }

    void read_serial()
    {
        /*char buf[1024];
        ssize_t n = read(serial_fd_, buf, sizeof(buf));
        if (n == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Received nothing");
        }
        else
        {
            buf[n] = '\0';
            //RCLCPP_INFO(this->get_logger(), "Received: %s", buf);
            //std::cout << "Received: " << buf << std::endl;
            //std::cout.flush(); 

            std::string line(buf);*/



            std::string line = read_line_from_serial();
            if (line.length() >= 2 + 8 * 5 + 2 && line.substr(0, 2) == "o:") // Ensure CRC is included
            {
                //RCLCPP_WARN(this->get_logger(), "Received: %s", line.c_str());
                
                // Extract CRC from the received line
                std::string crc_str = line.substr(line.length() - 3, 2); // Assuming CRC is two bytes long
                uint16_t received_crc = 0;
                memcpy(&received_crc, crc_str.c_str(), sizeof(uint16_t));

                // Compute CRC for the received data
                uint16_t computed_crc = calculate_crc(reinterpret_cast<const uint8_t*>(line.c_str()), line.length() - 3); // Excluding CRC bytes
                
                // Validate CRC
                if (received_crc == computed_crc)
                {
                    krabi_msgs::msg::OdomLighter odom_msg;
                    std::string hex_values = line.substr(2, line.length() - 3); // Remove "o:" and CRC
                    int i = 0;
                    get_float(hex_values, i, odom_msg.pose_x);
                    i += 8;
                    get_float(hex_values, i, odom_msg.pose_y);
                    i += 8;
                    get_float(hex_values, i, odom_msg.angle_rz);
                    i += 8;
                    get_float(hex_values, i, odom_msg.speed_vx);
                    i += 8;
                    get_float(hex_values, i, odom_msg.speed_wz);
                    i += 8;
                    odom_pub_->publish(odom_msg);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "CRC Error! Received: %s", line.c_str());
                    RCLCPP_ERROR_STREAM(this->get_logger(), received_crc << " != " << computed_crc << std::endl);
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Error! Received: %s", line.c_str());
            }

            /*if (line.substr(0, 2) == "e:" && line.length() >= 2 + 8 * 2)
            {
                krabi_msgs::msg::Encoders encoders_msg;
                std::string hex_values = line.substr(2 + 8 * 2);
                int i = 0;
                get_float(hex_values, i, encoders_msg.encoder_right);
                i += 8;
                get_float(hex_values, i, encoders_msg.encoder_left);
                i += 8;
                encoders_pub_->publish(encoders_msg);
            }*/
        //}
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel_;
    rclcpp::Subscription<krabi_msgs::msg::MotorsCmd>::SharedPtr subscription_motors_cmd_;
    rclcpp::Subscription<krabi_msgs::msg::MotorsParameters>::SharedPtr subscription_motors_parameters_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_enable_motor_;

    rclcpp::Publisher<krabi_msgs::msg::OdomLighter>::SharedPtr odom_pub_;
    rclcpp::Publisher<krabi_msgs::msg::Encoders>::SharedPtr encoders_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    int serial_fd_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleBrokerSTM32>());
    rclcpp::shutdown();
    return 0;
}
