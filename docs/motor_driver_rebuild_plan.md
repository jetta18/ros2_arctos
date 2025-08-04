# MKS Servo Motor Driver Rebuild Plan

## Project Overview
Complete rebuild of the MKS Servo 42D/57D motor driver implementation to ensure reliable CAN communication and proper joint state feedback for ROS2 control.

## Current Issues to Solve
1. ❌ Joint positions remain zero despite valid encoder data
2. ❌ CAN message processing routing problems  
3. ❌ Encoder data not flowing correctly through the system
4. ❌ Inconsistent message parsing and data extraction

## Phase 1: Clean CAN Protocol Foundation 🔧

### 1.1 Create Simple CAN Message Classes
```cpp
// Simple, focused CAN message structures
struct CANFrame {
    uint32_t id;
    uint8_t dlc;
    std::array<uint8_t, 8> data;
    
    uint8_t calculateCRC() const;
    bool isValid() const;
};

struct CANCommand {
    uint8_t cmd;
    std::vector<uint8_t> payload;
    
    CANFrame toFrame(uint32_t motor_id) const;
};
```

### 1.2 Implement Core CAN Commands
Focus on **essential commands only**:
- ✅ **0x31**: Read encoder (primary position feedback)
- ✅ **0x34**: Read IO status (limit switches)
- ✅ **0xF3**: Enable/disable motor
- ✅ **0x92**: Set zero position
- ✅ **0x82**: Set work mode to SR_vFOC

### 1.3 Clean CAN Protocol Class
```cpp
class CANProtocol {
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
    
public:
    // Simple, reliable methods
    bool sendCommand(uint32_t motor_id, const CANCommand& cmd);
    CANFrame readEncoder(uint32_t motor_id);
    CANFrame readIOStatus(uint32_t motor_id);
    bool enableMotor(uint32_t motor_id, bool enable);
    bool setZeroPosition(uint32_t motor_id);
};
```

## Phase 2: Robust Message Processing 📨

### 2.1 Clear Message Routing
```cpp
class CANMessageProcessor {
private:
    std::unordered_map<uint8_t, std::function<void(const CANFrame&)>> handlers_;
    
public:
    void registerHandler(uint8_t cmd, std::function<void(const CANFrame&)> handler);
    void processMessage(const can_msgs::msg::Frame::SharedPtr& msg);
    
private:
    bool validateFrame(const CANFrame& frame);
    CANFrame convertRosFrame(const can_msgs::msg::Frame::SharedPtr& msg);
};
```

### 2.2 Dedicated Response Handlers
```cpp
// Separate handler for each command type
void handleEncoderResponse(const CANFrame& frame);
void handleIOResponse(const CANFrame& frame);
void handleStatusResponse(const CANFrame& frame);
```

### 2.3 Data Validation & Extraction
```cpp
struct EncoderData {
    int64_t raw_value;      // 48-bit signed value from 0x31
    double angle_degrees;   // Converted to degrees
    bool is_valid;
    
    static EncoderData fromCANFrame(const CANFrame& frame);
};

struct IOStatus {
    bool limit_left;
    bool limit_right;
    bool out1;
    bool out2;
    bool is_valid;
    
    static IOStatus fromCANFrame(const CANFrame& frame);
};
```

## Phase 3: Simplified Motor Driver 🎯

### 3.1 Clean Motor State Management
```cpp
struct MotorState {
    // Essential state only
    uint32_t motor_id;
    std::string joint_name;
    
    // Position data
    double position_rad;
    double velocity_rad_s;
    rclcpp::Time last_update;
    
    // Motor parameters
    double gear_ratio;
    bool inverted;
    
    // Status
    bool enabled;
    bool limit_left;
    bool limit_right;
    
    void updateFromEncoder(const EncoderData& data);
    void updateFromIO(const IOStatus& status);
};
```

### 3.2 Focused Motor Driver Interface
```cpp
class MotorDriver {
private:
    std::unique_ptr<CANProtocol> can_protocol_;
    std::unique_ptr<CANMessageProcessor> message_processor_;
    std::unordered_map<std::string, MotorState> motors_;
    
public:
    // Core functionality only
    bool addMotor(const std::string& joint_name, uint32_t motor_id, double gear_ratio, bool inverted);
    bool initializeMotor(const std::string& joint_name);
    
    // State queries
    double getJointPosition(const std::string& joint_name) const;
    double getJointVelocity(const std::string& joint_name) const;
    bool isMotorEnabled(const std::string& joint_name) const;
    
    // Commands
    bool enableMotor(const std::string& joint_name, bool enable);
    bool setZeroPosition(const std::string& joint_name);
    void requestStateUpdate(const std::string& joint_name);
    
private:
    void onCANMessage(const can_msgs::msg::Frame::SharedPtr& msg);
    MotorState* findMotor(const std::string& joint_name);
};
```

## Phase 4: Integration & Testing 🧪

### 4.1 Step-by-Step Testing
1. **CAN Communication Test**: Verify basic send/receive
2. **Single Motor Test**: Test one motor encoder reading
3. **Multi-Motor Test**: Test all 6 motors simultaneously  
4. **ROS Integration Test**: Verify joint_states publication
5. **Movement Test**: Test position commands and feedback

### 4.2 Debug Tools
```cpp
class CANDebugger {
public:
    void logFrame(const CANFrame& frame, const std::string& direction);
    void logEncoderData(const EncoderData& data);
    void logMotorState(const MotorState& state);
    
    // Statistics
    void printStatistics();
    size_t getFrameCount(uint8_t cmd);
    double getAverageLatency();
};
```

## Phase 5: Hardware Interface Integration 🔗

### 5.1 Clean Hardware Interface
```cpp
// Simplified read() method
return_type ArctosInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // Update motor states (throttled to reasonable rate)
    if (shouldUpdateStates(time)) {
        motor_driver_->requestAllStates();
    }
    
    // Copy current positions to ROS arrays
    for (size_t i = 0; i < info_.joints.size(); i++) {
        const std::string& joint_name = info_.joints[i].name;
        joint_position_[i] = motor_driver_->getJointPosition(joint_name);
        joint_velocities_[i] = motor_driver_->getJointVelocity(joint_name);
    }
    
    return return_type::OK;
}
```

## Implementation Strategy 📋

### Week 1: Foundation
- [ ] Create MKS reference document ✅
- [ ] Implement basic CANFrame and CANCommand classes
- [ ] Create simple CANProtocol with essential commands
- [ ] Test basic CAN send/receive

### Week 2: Message Processing  
- [ ] Implement CANMessageProcessor with clear routing
- [ ] Create EncoderData and IOStatus extraction
- [ ] Test encoder reading with single motor
- [ ] Validate data extraction accuracy

### Week 3: Motor Driver
- [ ] Implement simplified MotorState management
- [ ] Create focused MotorDriver interface
- [ ] Test multi-motor state management
- [ ] Integrate with existing hardware interface

### Week 4: Integration & Polish
- [ ] Full system integration testing
- [ ] Performance optimization
- [ ] Documentation and cleanup
- [ ] Deployment and validation

## Success Criteria ✅

1. **Reliable Encoder Reading**: All 6 motors report accurate positions
2. **Consistent Data Flow**: Joint states update continuously at expected rate
3. **Proper Message Routing**: No more command misrouting or parsing errors
4. **Clean Architecture**: Code is maintainable and well-documented
5. **ROS Integration**: Seamless integration with ros2_control framework

## Key Design Principles

1. **Simplicity First**: Start with minimal working implementation
2. **Clear Separation**: Distinct layers for CAN, processing, and motor management
3. **Robust Validation**: Validate all incoming data before processing
4. **Comprehensive Logging**: Debug-friendly with detailed logging
5. **Testable Components**: Each component can be tested independently

## Files to Create/Modify

### New Files:
- `include/arctos_motor_driver/can_frame.hpp`
- `include/arctos_motor_driver/can_command.hpp` 
- `include/arctos_motor_driver/can_message_processor.hpp`
- `include/arctos_motor_driver/motor_state.hpp`
- `src/can_frame.cpp`
- `src/can_command.cpp`
- `src/can_message_processor.cpp`
- `src/motor_state.cpp`

### Modified Files:
- `include/arctos_motor_driver/can_protocol.hpp` (simplify)
- `src/can_protocol.cpp` (rebuild)
- `include/arctos_motor_driver/motor_driver.hpp` (simplify)
- `src/motor_driver.cpp` (rebuild)

This plan focuses on creating a solid, testable foundation that can be built incrementally while maintaining the existing ROS2 integration points.
