#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"  // for relative speeds

// Control table address for AX & MX(1.0)
#define ADDR_TORQUE_ENABLE      24
#define ADDR_GOAL_POSITION      30
#define ADDR_PRESENT_POSITION   36
#define ADDR_CCW_ANGLE_LIMIT     8
#define ADDR_MOVING_SPEED       32
#define LEN_MX_MOVING           2
#define ADDR_GOAL_ACCELERATION  73

// Protocol version
#define PROTOCOL_VERSION 1.0  // Default Protocol version of DYNAMIXEL AX & MX.
// Default setting
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL MX series is 57600
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"
#define ACCELERATION  10
#define MOTOR_1     1
#define MOTOR_2     2
#define MOTOR_3     3





using std::placeholders::_1;
using namespace std::chrono_literals;

class MotorWheelControl : public rclcpp::Node{
public:

    MotorWheelControl();
    virtual ~MotorWheelControl();

private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    bool dxl_addparam_result = false;               // addParam result
    uint8_t param_speed_value[2];

    

    float m1_rs, m2_rs, m3_rs;   //holds relative speeds for each motor

    /* publishers and subscribers declarations */
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr relative_speeds_sub_;

    

    void setDxl(uint8_t id);  // set dynamixels M-12 as wheel mode
    void dxlSetAcceleration(uint8_t id);
    void dxlSetSpeed(uint8_t id, double speed);
    
    void dxlMotorsOn();
    

    /* callback functions */
    void relativeSpeeds_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);


    /* check for errors during transmissions */
    inline int checkErrors(int result){
        if(result != COMM_SUCCESS){
            RCLCPP_ERROR(this->get_logger(),"%s", packetHandler->getTxRxResult(result));
            assert(result == COMM_SUCCESS);
        }else if(dxl_error != 0){
            RCLCPP_ERROR(this->get_logger(),"%s", packetHandler->getRxPacketError(dxl_error));
            assert(dxl_error == 0);
        }
        return result;
    }


};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorWheelControl>());
    rclcpp::shutdown();

    return 0;
}

MotorWheelControl::MotorWheelControl() : Node("motorcontrollernode"){
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    
    
    /* Open Serial Port */
    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
        assert(dxl_comm_result == true);
  }
    dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    if (dxl_comm_result == false){
        RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate");
        assert(dxl_comm_result == true);
    }
    
    /* susbcriber definition */
    relative_speeds_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("relative_speeds", 10, std::bind(&MotorWheelControl::relativeSpeeds_callback, this, _1) );
    
    setDxl(BROADCAST_ID); // all motors: wheel mode
    dxlSetAcceleration(BROADCAST_ID);  // all wheels

    
}


void MotorWheelControl::dxlMotorsOn(){

    /**
     * Wheel Mode It is a moving speed to Goal direction. 0~2047 (0X7FF) can be used, and the unit is about 0.916rpm.
     * If a value in the range of 0~1023 is used, it is stopped by setting to 0 while rotating to CCW direction. 
     * If a value in the range of 1024~2047 is used, it is stopped by setting to 1024 while rotating to CW direction. 
     * That is, the 10th bit becomes the direction bit to control the direction.
     *   
     *                        
     * (Front left ID: 1) O--------O (front right ID: 3)
     *                     \      /
     *                      \    /
     *                       \  /
     *                         O (rear wheel ID: 2)
    */
   
    float s1{},s2{},s3{}; // holds speeds 
    /* Initialize Group synchronize instant */
    dynamixel::GroupSyncWrite groupSpeedSyncWrite(portHandler, packetHandler, ADDR_MOVING_SPEED, LEN_MX_MOVING);
    
    
    /* Set speeds for each motor */
    /* Group synchronization */

    // Allocate speed value into byte array

    /* Motor 1 */  

    if(m1_rs >= 0){
        s1 = m1_rs * 1024 + 1024;  // CW
        param_speed_value[0] = DXL_LOBYTE(s1);
        param_speed_value[1] = DXL_HIBYTE(s1);
        dxl_addparam_result = groupSpeedSyncWrite.addParam(MOTOR_1, param_speed_value);
        if(dxl_addparam_result != true){
            RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupSyncWrite addparam failed", MOTOR_1);
            assert(dxl_addparam_result == true);
        }

    }else{
        s1 =  -m1_rs * 1024 + 0 ;  // CCW
        param_speed_value[0] = DXL_LOBYTE(s1);
        param_speed_value[1] = DXL_HIBYTE(s1);
        dxl_addparam_result = groupSpeedSyncWrite.addParam(MOTOR_1, param_speed_value);
        if(dxl_addparam_result != true){
            RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupSyncWrite addparam failed", MOTOR_1);
            assert(dxl_addparam_result == true);
        }
    }

    /* Motor 2 */
    if(m2_rs >= 0){
        s2 = m2_rs * 1024 + 1024;  // CW
        param_speed_value[0] = DXL_LOBYTE(s2);
        param_speed_value[1] = DXL_HIBYTE(s2);
        dxl_addparam_result = groupSpeedSyncWrite.addParam(MOTOR_2, param_speed_value);
        if(dxl_addparam_result != true){
            RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupSyncWrite addparam failed", MOTOR_2);
            assert(dxl_addparam_result == true);
        }

    }else{
        s2 =  -m2_rs * 1024 + 0 ;  // CCW
        param_speed_value[0] = DXL_LOBYTE(s2);
        param_speed_value[1] = DXL_HIBYTE(s2);
        dxl_addparam_result = groupSpeedSyncWrite.addParam(MOTOR_2, param_speed_value);
        if(dxl_addparam_result != true){
            RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupSyncWrite addparam failed", MOTOR_2);
            assert(dxl_addparam_result == true);
        }
    }    
        
    /* Motor 3 */
    if(m3_rs >= 0){
        s3 = m3_rs * 1024 + 1024;  // CW
        param_speed_value[0] = DXL_LOBYTE(s3);
        param_speed_value[1] = DXL_HIBYTE(s3);
        dxl_addparam_result = groupSpeedSyncWrite.addParam(MOTOR_3, param_speed_value);
        if(dxl_addparam_result != true){
            RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupSyncWrite addparam failed", MOTOR_3);
            assert(dxl_addparam_result == true);
        }

    }else{
        s3 =  -m3_rs * 1024 + 0 ;  // CCW
        param_speed_value[0] = DXL_LOBYTE(s3);
        param_speed_value[1] = DXL_HIBYTE(s3);
        dxl_addparam_result = groupSpeedSyncWrite.addParam(MOTOR_3, param_speed_value);
        if(dxl_addparam_result != true){
            RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupSyncWrite addparam failed", MOTOR_3);
            assert(dxl_addparam_result == true);
        }
    } 

    /* Syncwrite moving speed */

    checkErrors( dxl_comm_result = groupSpeedSyncWrite.txPacket() );
    

    // Clear syncwrite parameter storage
    groupSpeedSyncWrite.clearParam();



    /* No group synchronization */
    // if(m1_rs>= 0){
    // 	s1 = m1_rs * 1024 + 1024;
    //     dxlSetSpeed(MOTOR_1, s1);  // CW
    // }else{
    //     s1 =  -m1_rs * 1024 + 0 ;  // CCW
    // 	dxlSetSpeed(MOTOR_1, s1);
    // }
    // if(m2_rs >= 0){
    //     s2 = m2_rs * 1024 + 1024;  // CW
    // 	dxlSetSpeed(MOTOR_2, s2);
    // }else{
    //     s2 = -m2_rs * 1024 + 0;   // CCW
    // 	dxlSetSpeed(MOTOR_2, s2);
    // }
    // if(m3_rs>= 0){
    //     s3 = m3_rs * 1024 + 1024;  // CW
    // 	dxlSetSpeed(MOTOR_3, s3);
    // }else{
    //     s3 = -m3_rs * 1024 + 0;   // CCW
    // 	dxlSetSpeed(MOTOR_3, s3);
    // }

    
}


void MotorWheelControl::relativeSpeeds_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg){

    m1_rs = msg->data[0];  // motor 1
    m2_rs = msg->data[1];  // motor 2
    m3_rs = msg->data[2];  // motor 3

    dxlMotorsOn();

}

void MotorWheelControl::setDxl(uint8_t id){

    /* Disable torque */
    checkErrors(packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, &dxl_error));
    /* Set wheel mode */
    checkErrors(packetHandler->write2ByteTxRx(portHandler, id, ADDR_CCW_ANGLE_LIMIT, (uint16_t) 0, &dxl_error));  // 0 : no limit
    /* Enable torque */
    checkErrors(packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, &dxl_error));

}

void MotorWheelControl::dxlSetAcceleration(uint8_t id){

    /* Set acceleration */
    checkErrors(packetHandler->write1ByteTxRx(portHandler, id, ADDR_GOAL_ACCELERATION, (uint8_t)ACCELERATION, &dxl_error));

}

void MotorWheelControl::dxlSetSpeed(uint8_t id, double speed){

    /* Set wheel speed */
    checkErrors(packetHandler->write2ByteTxRx(portHandler, id, ADDR_MOVING_SPEED, (uint16_t)speed, &dxl_error));

}

MotorWheelControl::~MotorWheelControl(){

}