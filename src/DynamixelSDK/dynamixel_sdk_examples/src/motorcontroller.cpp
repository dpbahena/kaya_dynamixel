#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_relative_speed.hpp"  // for relative speeds
#include "std_msgs/msg/bool.hpp"  // for checking motor off/on





// Control table address for AX & MX(1.0)
#define ADDR_TORQUE_ENABLE      24
#define ADDR_GOAL_POSITION      30
#define ADDR_PRESENT_POSITION   36
#define ADDR_CCW_ANGLE_LIMIT     8
#define ADDR_MOVING_SPEED       32
#define LEN_MX_MOVING            2   // 2 bytes 
#define LEN_MX_MOVING_STATUS     1   // 1 byte
#define ADDR_GOAL_ACCELERATION  73
#define ADDR_MOVING_STATUS      46
#define ADDR_PRESENT_SPEED      38
#define ADDR_TORQUE_LIMIT       34


// Protocol version
#define PROTOCOL_VERSION 1.0  // Default Protocol version of DYNAMIXEL AX & MX.
// Default setting
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL MX series is 57600
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"
#define ACCELERATION  50
#define MOTOR_1     1
#define MOTOR_2     2
#define MOTOR_3     3
#define ON          1
#define OFF         0



/* USAGE EXAMPLE - to start motors once 
*
*  ros2 topic pub -1 /relative_speeds dynamixel_sdk_custom_interfaces/msg/SetRelativeSpeed "{m1: -0.5, m2: 0.4, m3: .5}"  
*  ros2 topic pub -t 3 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
* 
*  change parameter in command line (to change maxtime wheels running on last command)
*  
*  ros2 run dynamixel_sdk_examples motorcontroller --ros-args -p maxtime:=4 -p accel:=5 -p torque:=512
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
    
    /**
     * @brief To calculate the velocity factor for the MX-12 Dynamixel motor, divide the maximum Dynamixel goal velocity value (1023) by the maximum RPM of the motors
     *          constant used to convert the calculated wheel velocities to the appropriate unit or scale expected by the Dynamixel motors.
     */
    float velocityFactor; 
    /**
     * @brief  Acceleration It can be used from 0~254(0xFE) and the unit is approximately 8.583 [° / sec2].
     *          When it is set to 0, there is no control over acceleration and moves with the maximum acceleration of the motor.
     *          When it is set to 254, it becomes 2,180 [° / sec2].
     *          For example, the present speed of DYNAMIXEL is 0, and Goal Acceleration is 10.
     *          The speed of DYNAMIXEL after 1 second will be 14.3 [RPM].
    */
    int8_t acceleration;

    /**
     * @brief  It is the value of the maximum torque limit.
     *         0 ~ 1,023(0x3FF) is available, and the unit is about 0.1%.
     *         For example, if the value is 512, it is about 50%; that means only 50% of the maximum torque will be used.
     *         When the power is turned on, Torque Limit(34) is reset to the value of Max Torque(14).
    */
    int16_t torqueLimit;


    float m1_rs, m2_rs, m3_rs;   //holds relative speeds for each motor
    float s1{},s2{},s3{}; // hold actual speeds

    /* publishers and subscribers declarations */
    rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed>::SharedPtr relative_speeds_sub_;
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
    void relativeSpeeds_callback(const dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed::ConstSharedPtr msg);
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

   
    this->declare_parameter("accel", 0);
    this->get_parameter("accel", acceleration);  

    this->declare_parameter("torque", 512);
    this->get_parameter("torque", torqueLimit);
  

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
    relative_speeds_sub_ = this->create_subscription<dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed>("relative_speeds", 10, std::bind(&MotorWheelControl::relativeSpeeds_callback, this, _1) );

    /* publisher definition */
    motorstatus_pub_ = this->create_publisher<std_msgs::msg::Bool>("motor_status", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&MotorWheelControl::motors_status_callback, this) );

    /* measure elapsed time only when motors are ON */
    timer2_ = this->create_wall_timer(10ms, [&](){if(motors_status == ON) elapsedTime_callback();});

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
     * (uint16_t)s1 >> 10) & 1  checks if the 10th bit is set. If set to 1 then is CW otherwise is CCW
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
    RCLCPP_INFO(this->get_logger(),"Elapset time %0.3f\n",  this->get_clock()->now().seconds() - start );
    //std::this_thread::sleep_for(2s); // give time for wheels stop moving after stop command issued
    motors_status = OFF;    // or you can simple change the status to OFF by uncommenting this line and commenting both the above and below line
    //motors_status = areMotorsOn();
    

    // Clear syncwrite parameter storage
    groupSpeedSyncWrite.clearParam();
    
    /* reset speeds */
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
        s1 = m1_rs + 1024;  // CW
        if(s1 > 2047) s1 = 2047; // max value
        //RCLCPP_INFO(this->get_logger(),"CW %f\t", s1);
        param_speed_value[0] = DXL_LOBYTE(s1);
        param_speed_value[1] = DXL_HIBYTE(s1);
        dxl_addparam_result = groupSpeedSyncWrite.addParam(MOTOR_1, param_speed_value);
        if(dxl_addparam_result != true){
            RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupSyncWrite addparam failed", MOTOR_1);
            assert(dxl_addparam_result == true);
        }

    }else{
        s1 =  -m1_rs + 0 ;  // CCW
        if(s1 > 1023) s1 = 1023;  // max value
        //RCLCPP_INFO(this->get_logger(),"CCW %f\t", s1);
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
        s2 = m2_rs + 1024;  // CW
        if(s2 > 2047) s2 = 2047;
        //RCLCPP_INFO(this->get_logger(),"CW %f\t", s2);
        param_speed_value[0] = DXL_LOBYTE(s2);
        param_speed_value[1] = DXL_HIBYTE(s2);
        dxl_addparam_result = groupSpeedSyncWrite.addParam(MOTOR_2, param_speed_value);
        if(dxl_addparam_result != true){
            RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupSyncWrite addparam failed", MOTOR_2);
            assert(dxl_addparam_result == true);
        }

    }else{
        s2 =  -m2_rs + 0 ;  // CCW
        if(s2 > 1023) s2 = 1023;
        //RCLCPP_INFO(this->get_logger(),"CCW %f\t", s2);
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
        s3 = m3_rs + 1024;  // CW
        if(s3 > 2047) s3 = 2047;
        //RCLCPP_INFO(this->get_logger(),"CW %f\n", s3);
        param_speed_value[0] = DXL_LOBYTE(s3);
        param_speed_value[1] = DXL_HIBYTE(s3);
        dxl_addparam_result = groupSpeedSyncWrite.addParam(MOTOR_3, param_speed_value);
        if(dxl_addparam_result != true){
            RCLCPP_ERROR(this->get_logger(),"[ID:%03d] groupSyncWrite addparam failed", MOTOR_3);
            assert(dxl_addparam_result == true);
        }

    }else{
        s3 =  -m3_rs + 0 ;  // CCW
        if(s3 > 1023) s3 = 1023;
        //RCLCPP_INFO(this->get_logger(),"CCW %f\n", s3);
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
void MotorWheelControl::relativeSpeeds_callback(const dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed::ConstSharedPtr msg){

    m1_rs = msg->m1;  // motor 1
    m2_rs = msg->m2;  // motor 2
    m3_rs = msg->m3;  // motor 3
    
    dxlStartMotors();
    

}

void MotorWheelControl::setDxl(uint8_t id){

    /* Disable torque */
    checkErrors(packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, &dxl_error));
    /* Set wheel mode */
    checkErrors(packetHandler->write2ByteTxRx(portHandler, id, ADDR_CCW_ANGLE_LIMIT, (uint16_t) 0, &dxl_error));  // 0 : no limit
    /* Enable torque */
    checkErrors(packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, &dxl_error));

    /* Limit max-torque */
    checkErrors(packetHandler->write2ByteTxRx(portHandler, id, ADDR_TORQUE_LIMIT, torqueLimit, &dxl_error));
    
    
}

void MotorWheelControl::dxlSetAcceleration(uint8_t id){

    /* Set acceleration */
    checkErrors(packetHandler->write1ByteTxRx(portHandler, id, ADDR_GOAL_ACCELERATION, (int8_t)acceleration, &dxl_error));

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
    uint16_t s1{},s2{},s3{};
    if(timeElapsed >= maxTime){
        /* Check the present speed of each wheel 0 - 1023*/
        checkErrors(packetHandler->read2ByteTxRx(portHandler, MOTOR_3,ADDR_PRESENT_SPEED, &s3, &dxl_error));
        
        checkErrors(packetHandler->read2ByteTxRx(portHandler, MOTOR_2,ADDR_PRESENT_SPEED, &s2, &dxl_error));
        checkErrors(packetHandler->read2ByteTxRx(portHandler, MOTOR_1,ADDR_PRESENT_SPEED, &s1, &dxl_error));
        RCLCPP_INFO(this->get_logger(),"speeds %d, %d, %d\n", s1%1024,s2%1024,s3%1024);
        RCLCPP_INFO(this->get_logger(),"RPM %f %f, %f\n", (s1%1024)*0.916,(s2%1024)*0.916,(s3%1024)*0.916);

        dxlStopMotors();
        // motors_status = areMotorsOn();
    }
}