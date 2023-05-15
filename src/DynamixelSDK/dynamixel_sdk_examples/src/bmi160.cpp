#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "DFRobot_BMI160.h"

#define BMI160_ADDR    UINT8_C(0x69)


using namespace std::chrono_literals;
using std::placeholders::_1;


class AccelGyro : public rclcpp::Node{
public:
    AccelGyro() :  Node("accelgyronode"){
    bmi160 = new DFRobot_BMI160(BMI160_ADDR);   
    Init_hardware();
    initI2c();
    
    
    // init publisher
    imuPub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu",10);
    timer_ = this->create_wall_timer(100ms, std::bind(&AccelGyro::imu_callback, this));
    }

    ~AccelGyro(){
        delete bmi160;
    }
private:
   
    //DFRobot_BMI160 bmi160(0x68);
    DFRobot_BMI160 *bmi160;
    
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void imu_callback(){
        
        auto msg = sensor_msgs::msg::Imu();
        int rslt;
        
        int16_t accelgyro[6]={0};
        rslt = bmi160->getAccelGyroData(accelgyro);
        if(rslt == 0){
            msg.angular_velocity.x = accelgyro[0]*M_PI/180.0;
            msg.angular_velocity.y = accelgyro[1]*M_PI/180.0;
            msg.angular_velocity.z = accelgyro[2]*M_PI/180.0;
            msg.linear_acceleration.x = accelgyro[3]/16384.0;
            msg.linear_acceleration.y = accelgyro[4]/16384.0;
            msg.linear_acceleration.z = accelgyro[5]/16384.0;
        }
        
        imuPub_->publish(msg);
    }

    void initI2c(){
        
        
        /* Set and init the bmi160 12c address */
        if(bmi160->I2cInit()!= BMI160_OK){
            RCLCPP_ERROR(this->get_logger(),"Init false");
            while(1);
        }else{
            printf("I2C init OK! \n");
        }
    }

    void Init_hardware(){

         /* Init the hardware bmin160 */ 
        if(bmi160->softReset() != BMI160_OK){
            RCLCPP_ERROR(this->get_logger(),"Reset false");
            while(1);
        }else{
            printf("Soft Reset OK! \n");
        }
    }
};


int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AccelGyro>());
    rclcpp::shutdown();
    
    return 0;
}
