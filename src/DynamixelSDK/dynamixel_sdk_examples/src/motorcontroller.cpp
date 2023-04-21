#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"  // for relative speeds
#include "std_msgs/msg/bool.hpp"  // for checking motor off/on

// Control table address for AX & MX(1.0)
#define ADDR_TORQUE_ENABLE      24
#define ADDR_GOAL_POSITION      30
#define ADDR_PRESENT_POSITION   36
#define ADDR_CCW_ANGLE_LIMIT     8
#define ADDR_MOVING_SPEED       32
#define LEN_MX_MOVING            2   // 2 bytes 
#define LEN_MX_MOVING_STATUS         1   // 1 byte
#define ADDR_GOAL_ACCELERATION  73
#define ADDR_MOVING_STATUS      46

// Protocol version
#define PROTOCOL_VERSION 1.0  // Default Protocol version of DYNAMIXEL AX & MX.
// Default setting
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL MX series is 57600
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"
#define ACCELERATION  10
#define MOTOR_1     1
#define MOTOR_2     2
#define MOTOR_3     3
#define ON          1
#define OFF         0


/* USAGE EXAMPLE - to start motors once 
*
*  ros2 topic pub -1 /relative_speeds std_msgs/msg/Float32MultiArray "{data: {0.58, .3, -0.58, 0}}"  
* 
*/


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
    bool dxl_addparam_result = false;    // addParam result
    bool motors_status = OFF;
    uint8_t param_speed_value[2];

    int maxTime;  // max number of seconds the motors untir motors on same command will be shutdown.

    

    float m1_rs, m2_rs, m3_rs;   //holds relative speeds for each motor
    float s1{},s2{},s3{}; // hold actual speeds

    /* publishers and subscribers declarations */
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr relative_speeds_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motorstatus_pub_;
    

    /* timers */
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;
    double start, end; // measure time intervals for length of time motors are working on last command
    

    void setDxl(uint8_t id);  // set dynamixels M-12 as wheel mode
    void dxlSetAcceleration(uint8_t id);
    void dxlSetSpeed(uint8_t id, double speed);
    
    void dxlStartMotors();
    void dxlStopMotors();
    bool areMotorsOn();
    
    

    /* callback functions */
    void relativeSpeeds_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);
    void motors_status_callback();
    void elapsedTime_callback();


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

    this->declare_parameter("maxtime", 3);
    this->get_parameter("maxtime", maxTime);  // maxTime gets its value fro the parameter "maxtime"
    
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

    /* publisher definition */
    motorstatus_pub_ = this->create_publisher<std_msgs::msg::Bool>("motor_status", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&MotorWheelControl::motors_status_callback, this) );

    /* measure elapsed time only when motors are ON */
    timer2_ = this->create_wall_timer(100ms, [&](){if(motors_status == ON) elapsedTime_callback();});

    setDxl(BROADCAST_ID); // all motors: wheel mode
    dxlSetAcceleration(BROADCAST_ID);  // all wheels

    
}
/**
 * @brief Publish the status of the motors ON/OFF
 * 
 */
void MotorWheelControl::motors_status_callback(){
    
    std_msgs::msg::Bool msg;
    msg.data = areMotorsOn();
    motorstatus_pub_->publish(msg);
        
}
/**
 * @brief Stop all the motors
 * 
 */
void MotorWheelControl::dxlStopMotors(){
    /* Initialize Group synchronize instant */
    dynamixel::GroupSyncWrite groupSpeedSyncWrite(portHandler, packetHandler, ADDR_MOVING_SPEED, LEN_MX_MOVING);

    /**
     * (uint16_t)s1 << 10) & 1  checks if the 10th bit is set. If set to 1 then is CW otherwise is CCW
     *  Value of 0 stops motors CCW
     *  value of 1024 stops motors CW
     */
    if( ((uint16_t)s1 >> 10) & 1){   
        s1 = 1024; //CW
    } else{
        s1 = 0;   // CCW
    }
    if( ((uint16_t)s2 >> 10) & 1){
        s2 = 1024; //CW
    } else{
        s2 = 0;   // CCW
    }
    if( ((uint16_t)s3 >> 10) & 1){
        s3 = 1024; //CW
    } else{
        s3 = 0;   // CCW
    }

    param_speed_value[0] = DXL_LOBYTE(s1);
    param_speed_value[1] = DXL_HIBYTE(s1);
    dxl_addparam_result = groupSpeedSyncWrite.addParam(MOTOR_1, param_speed_value);
    if(dxl_addparam_result != true){
        RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupSyncWrite addparam failed", MOTOR_1);
        assert(dxl_addparam_result == true);
    }
    
    param_speed_value[0] = DXL_LOBYTE(s2);
    param_speed_value[1] = DXL_HIBYTE(s2);
    dxl_addparam_result = groupSpeedSyncWrite.addParam(MOTOR_2, param_speed_value);
    if(dxl_addparam_result != true){
        RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupSyncWrite addparam failed", MOTOR_2);
        assert(dxl_addparam_result == true);
    }
    param_speed_value[0] = DXL_LOBYTE(s3);
    param_speed_value[1] = DXL_HIBYTE(s3);
    dxl_addparam_result = groupSpeedSyncWrite.addParam(MOTOR_3, param_speed_value);
    if(dxl_addparam_result != true){
        RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupSyncWrite addparam failed", MOTOR_3);
        assert(dxl_addparam_result == true);
    }


     /* Syncwrite moving (stop) speed */

    checkErrors( groupSpeedSyncWrite.txPacket() );
    

    // Clear syncwrite parameter storage
    groupSpeedSyncWrite.clearParam();
    s1 = 0;
    s2 = 0;
    s3 = 0;



}
void MotorWheelControl::dxlStartMotors(){

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

    /* start timer */
    start = this->get_clock()->now().seconds();

    /* Syncwrite moving speed */

    checkErrors( groupSpeedSyncWrite.txPacket() );
    

    // Clear syncwrite parameter storage
    groupSpeedSyncWrite.clearParam();

    motors_status = areMotorsOn();  // check if motors are actually on


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

/**
 * @brief  Checks the status of each motor: ON , OFF
 * 
 * @return true 
 * @return false 
 */
bool MotorWheelControl::areMotorsOn(){

    bool dxl_getdata_result = false;
    bool state_m1, state_m2, state_m3;

    /* Initialize GroupBulkRead Instance */
    dynamixel::GroupBulkRead groupReadMotorStatus(portHandler, packetHandler);

    /* Add parameter storage for Dynamixel#1 present motor status */
     dxl_addparam_result = groupReadMotorStatus.addParam(MOTOR_1, ADDR_MOVING_STATUS, LEN_MX_MOVING_STATUS);
        if(dxl_addparam_result != true){
            RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupBulkRead addparam failed", MOTOR_1);
            assert(dxl_addparam_result == true);
        }
     dxl_addparam_result = groupReadMotorStatus.addParam(MOTOR_2, ADDR_MOVING_STATUS, LEN_MX_MOVING_STATUS);
        if(dxl_addparam_result != true){
            RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupBulkRead addparam failed", MOTOR_2);
            assert(dxl_addparam_result == true);
        }
     dxl_addparam_result = groupReadMotorStatus.addParam(MOTOR_3, ADDR_MOVING_STATUS, LEN_MX_MOVING_STATUS);
        if(dxl_addparam_result != true){
            RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupBulkRead addparam failed", MOTOR_3);
            assert(dxl_addparam_result == true);
        }


    /* Bulkread present position and moving status */
    checkErrors(groupReadMotorStatus.txRxPacket());
    checkErrors(groupReadMotorStatus.getError(MOTOR_1, &dxl_error));
    checkErrors(groupReadMotorStatus.getError(MOTOR_2, &dxl_error));
    checkErrors(groupReadMotorStatus.getError(MOTOR_3, &dxl_error));

    dxl_getdata_result = groupReadMotorStatus.isAvailable(MOTOR_1, ADDR_MOVING_STATUS, LEN_MX_MOVING_STATUS);
    if(dxl_getdata_result != true){
        RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupBulkRead getdata failed", MOTOR_1);
        assert(dxl_getdata_result == true);
    }
    dxl_getdata_result = groupReadMotorStatus.isAvailable(MOTOR_2, ADDR_MOVING_STATUS, LEN_MX_MOVING_STATUS);
    if(dxl_getdata_result != true){
        RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupBulkRead getdata failed", MOTOR_2);
        assert(dxl_getdata_result == true);
    }
    dxl_getdata_result = groupReadMotorStatus.isAvailable(MOTOR_3, ADDR_MOVING_STATUS, LEN_MX_MOVING_STATUS);
    if(dxl_getdata_result != true){
        RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupBulkRead getdata failed", MOTOR_3);
        assert(dxl_getdata_result == true);
    }
    
    /* Get the moving Status Value */
    state_m1 = groupReadMotorStatus.getData(MOTOR_1, ADDR_MOVING_STATUS, LEN_MX_MOVING_STATUS);
    state_m2 = groupReadMotorStatus.getData(MOTOR_2, ADDR_MOVING_STATUS, LEN_MX_MOVING_STATUS);
    state_m3 = groupReadMotorStatus.getData(MOTOR_3, ADDR_MOVING_STATUS, LEN_MX_MOVING_STATUS);

    groupReadMotorStatus.clearParam();

    if(state_m1 || state_m2 || state_m3){  // if either motor is on then the status is ON
        return ON;
    }

    return OFF;
}
void MotorWheelControl::relativeSpeeds_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg){

    m1_rs = msg->data.at(0);  // motor 1
    m2_rs = msg->data.at(1);  // motor 2
    m3_rs = msg->data.at(2);  // motor 3
    
    dxlStartMotors();
    

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

    /* Disable torque */
    checkErrors(packetHandler->write1ByteTxRx(portHandler, BROADCAST_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error)); // all motors
    
}

void MotorWheelControl::elapsedTime_callback(){
    auto checkTime = this->get_clock()->now().seconds();
    auto timeElapsed = checkTime - start;

    if(timeElapsed >= maxTime){
        dxlStopMotors();
        motors_status = areMotorsOn();
    }
}