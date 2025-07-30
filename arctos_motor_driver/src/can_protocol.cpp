#include "arctos_motor_driver/can_protocol.hpp"
#include "arctos_motor_driver/motor_types.hpp"
#include <cmath>
#include <chrono>
#include <thread>

namespace arctos_motor_driver {

CANProtocol::CANProtocol(rclcpp::Node::SharedPtr node) : node_(node) {
    can_pub_ = node->create_publisher<can_msgs::msg::Frame>("/to_motor_can_bus", 10);
}

void CANProtocol::sendFrame(uint8_t motor_id, const std::vector<uint8_t>& data) {
    auto start_time = node_->get_clock()->now();
    while (can_pub_->get_subscription_count() == 0) {
        if ((node_->get_clock()->now() - start_time).seconds() > 2.0) {
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for subscriber to /to_motor_can_bus");
            break;
        }
        RCLCPP_INFO(node_->get_logger(), "Waiting for subscriber to /to_motor_can_bus...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    auto msg = std::make_shared<can_msgs::msg::Frame>();
    msg->id = motor_id;
    msg->dlc = static_cast<uint8_t>(data.size() + 1);  // +1 for CRC byte
    
    // Initialize array with zeros
    std::fill(msg->data.begin(), msg->data.end(), 0);
    
    // Copy command data
    std::copy(data.begin(), data.end(), msg->data.begin());
    
    // Calculate CRC: ID + all data bytes
    uint16_t crc = motor_id;  // Start with motor ID
    for (size_t i = 0; i < data.size(); i++) {
        crc += data[i];
    }
    crc &= 0xFF;  // Keep only lower byte
    
    // Add CRC as last byte
    msg->data[data.size()] = static_cast<uint8_t>(crc);
    
    RCLCPP_INFO(node_->get_logger(), "[sendFrame] TX ID:%d CMD:0x%02X (len %zu)", motor_id, data.empty()?0:data[0], data.size());
    can_pub_->publish(*msg);
}

double CANProtocol::decodeInt48(const std::vector<uint8_t>& data) {
    if (data.size() != 6) {
        throw std::runtime_error("Data size must be 6 bytes for int48_t decoding");
    }

    // Combine the 6 bytes into a 48-bit integer
    int64_t addition_value = (static_cast<int64_t>(data[0]) << 40) |
                             (static_cast<int64_t>(data[1]) << 32) |
                             (static_cast<int64_t>(data[2]) << 24) |
                             (static_cast<int64_t>(data[3]) << 16) |
                             (static_cast<int64_t>(data[4]) << 8) |
                             static_cast<int64_t>(data[5]);

    // Apply two's complement to handle signed 48-bit values
    if (addition_value & 0x800000000000) {  // Check if the 47th bit (sign bit) is set
        addition_value |= 0xFFFF000000000000;  // Sign-extend to 64 bits
    }

    // Convert addition value to degrees
    double motor_angle_deg = (addition_value * MotorConstants::DEGREES_PER_REVOLUTION) / MotorConstants::ENCODER_STEPS;

    return motor_angle_deg;
}

double CANProtocol::decodeVelocityToRPM(const std::vector<uint8_t>& data) {
    if (data.size() != 2) {
        throw std::runtime_error("Invalid data size for velocity decoding. Expected 2 bytes.");
    }
    
    // RPM data is sent as a 16-bit signed integer in big-endian format
    int16_t speed = static_cast<int16_t>((data[0] << 8) | data[1]);
    return static_cast<double>(speed);
}

double CANProtocol::encoderToRadians(double encoder_value) {
    // Convert encoder degrees to radians
    return encoder_value * MotorConstants::DEG_TO_RAD;
}

double CANProtocol::radiansToEncoder(double radians) {
    // Convert radians to encoder degrees
    return radians * MotorConstants::RAD_TO_DEG;
}

double CANProtocol::rpmToRadPS(double rpm) {
    // Convert RPM to radians per second
    return rpm * MotorConstants::RPM_TO_RADPS;
}

double CANProtocol::radPSToRPM(double rad_ps) {
    // Convert radians per second to RPM
    return rad_ps * MotorConstants::RADPS_TO_RPM;
}

} // namespace arctos_motor_driver