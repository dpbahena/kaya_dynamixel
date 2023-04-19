#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

using namespace std::chrono_literals;

// Control table address for AX & MX(1.0)
#define ADDR_TORQUE_ENABLE      24
#define ADDR_GOAL_POSITION      30
#define ADDR_PRESENT_POSITION   36
#define ADDR_CCW_ANGLE_LIMIT     8
#define ADDR_MOVING_SPEED       32
#define ADDR_GOAL_ACCELERATION  73

// Protocol version
#define PROTOCOL_VERSION 1.0  // Default Protocol version of DYNAMIXEL AX & MX.
// Default setting
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL MX series is 57600
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"






class DynamixelWheel : public rclcpp::Node{
public:
    DynamixelWheel(int speed, int accel);
    virtual ~DynamixelWheel();
    
    

private:
    
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    uint8_t dxl_error = 0;
    uint32_t goal_position = 0;
    uint16_t speed, accel;
    int dxl_comm_result = COMM_TX_FAIL;


    void setUpDynamixelAsWheel(uint8_t dxl_id);
    void testWheel(uint8_t dxl_id);
    inline int checkErrors(int result){
        if(result != COMM_SUCCESS){
            RCLCPP_ERROR(this->get_logger()," He HE HECCW angle limit error %s", packetHandler->getTxRxResult(result));
            assert(result == COMM_SUCCESS);
        }else if(dxl_error != 0){
            RCLCPP_ERROR(this->get_logger()," horrores %s", packetHandler->getRxPacketError(dxl_error));
            assert(dxl_error == 0);
        }
        return result;
    }
    

    
    


};

DynamixelWheel::DynamixelWheel(int speed, int accel) : Node("wheel_node"){
    
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    this->speed = speed;
    this->accel = accel;
    /* Open Serial Port */
    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
        exit(1);
    }
    /* Set the baudrate of the serial port (use DYNAMIXEL Baudrate) */
    dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    if (dxl_comm_result == false){
        RCLCPP_ERROR(this->get_logger(), " Failed to set baudrate! ");
        exit(1);
    }
    
    setUpDynamixelAsWheel(BROADCAST_ID); // All wheels!
    rclcpp::sleep_for(1s);
    testWheel(2);



}

DynamixelWheel::~DynamixelWheel(){
    
    /* Exiting: set torque back to 0 */

    checkErrors(packetHandler->write1ByteTxRx(portHandler, BROADCAST_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error));
   
}

void DynamixelWheel::setUpDynamixelAsWheel(uint8_t dxl_id){
    
    
    /* Disable torque */
    checkErrors(packetHandler->write1ByteTxRx(portHandler, BROADCAST_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error));
    /* Set wheel mode */
    checkErrors(packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_CCW_ANGLE_LIMIT, (uint16_t)0, &dxl_error));
    /* Enable torque */
    checkErrors(packetHandler->write1ByteTxRx(portHandler, BROADCAST_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error));

    /* Set wheel mode */
    // /* Set torque to 0 */
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BROADCAST_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // if(dxl_comm_result != COMM_SUCCESS){
    //     RCLCPP_ERROR(this->get_logger()," Failed to enable torque! : %S", packetHandler->getRxPacketError(dxl_error));
    //     exit(1);
    // }else if(dxl_error != 0){
    //     RCLCPP_ERROR(this->get_logger()," %s", packetHandler->getRxPacketError(dxl_error));

    // }
    
    // /* Set wheel mode */
    // dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_CCW_ANGLE_LIMIT, (uint16_t)0, &dxl_error);
    // if(dxl_comm_result != COMM_SUCCESS){
    //     RCLCPP_ERROR(this->get_logger()," CCW angle limit error %s", packetHandler->getTxRxResult(dxl_comm_result));
    //     exit(1);
    // }else if(dxl_error != 0){
    //     RCLCPP_ERROR(this->get_logger()," %s", packetHandler->getRxPacketError(dxl_error));
    // }

    // /* Set torque */
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // if(dxl_comm_result != COMM_SUCCESS){
    //     RCLCPP_ERROR(this->get_logger()," Failed to enable torque! : %S", packetHandler->getRxPacketError(dxl_error));
    //     exit(1);
    // }else if(dxl_error != 0){
    //     RCLCPP_ERROR(this->get_logger()," %s", packetHandler->getRxPacketError(dxl_error));
    // }
}


void DynamixelWheel::testWheel(uint8_t dxl_id){

    /* Set acceleration */
    checkErrors(packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_GOAL_ACCELERATION, (uint8_t)accel, &dxl_error));
    /* Set wheel speed */
    checkErrors(packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_MOVING_SPEED, (uint16_t)speed, &dxl_error));
    std::this_thread::sleep_for(5s);
    /* Stop the motor */
    checkErrors(packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_MOVING_SPEED, (uint16_t)1024, &dxl_error));
    
    // /* Set acceleration */
    // dxl_comm_result =packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_GOAL_ACCELERATION, (uint8_t)accel, &dxl_error);
    // if(dxl_comm_result != COMM_SUCCESS){
    //     RCLCPP_ERROR(this->get_logger()," moving speed error %s", packetHandler->getTxRxResult(dxl_comm_result));
    //     exit(1);
    // }else if(dxl_error != 0){
    //     RCLCPP_ERROR(this->get_logger()," Acceleration error %s", packetHandler->getRxPacketError(dxl_error));
    // }

    //  /* Set wheel speed */
    // dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_MOVING_SPEED, (uint16_t)speed, &dxl_error);
    // if(dxl_comm_result != COMM_SUCCESS){
    //     RCLCPP_ERROR(this->get_logger()," moving speed error %s", packetHandler->getTxRxResult(dxl_comm_result));
    //     exit(1);
    // }else if(dxl_error != 0){
    //     RCLCPP_ERROR(this->get_logger()," %s", packetHandler->getRxPacketError(dxl_error));
    // }
    // std::this_thread::sleep_for(5s);

    // /* stop the motor */
    // dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_MOVING_SPEED, (uint16_t)1024, &dxl_error);
    // if(dxl_comm_result != COMM_SUCCESS){
    //     RCLCPP_ERROR(this->get_logger()," moving speed error %s", packetHandler->getTxRxResult(dxl_comm_result));
    //     exit(1);
    // }else if(dxl_error != 0){
    //     RCLCPP_ERROR(this->get_logger()," %s", packetHandler->getRxPacketError(dxl_error));
    // }
}
int main(int argc, char **argv){

    

    rclcpp::init(argc, argv);
    int speed, accel;
    if(argc == 3){
        speed = atoi(argv[1]);
        accel = atoi(argv[2]);
    }else{
        speed = 1800;
        accel = 60;
    }
    rclcpp::spin(std::make_shared<DynamixelWheel>(speed, accel));
    rclcpp::shutdown();

    return 0;


}

