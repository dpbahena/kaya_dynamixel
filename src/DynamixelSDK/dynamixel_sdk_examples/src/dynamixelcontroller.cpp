#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "geometry_msgs/msg/twist.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_relative_speed.hpp"


/**
 * @brief Testing commands
 * max linear velocity about 1.15 m/s
 * ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}"
 */

// const double WHEEL1_ANGLE = 0.0;
// const double WHEEL2_ANGLE = 2.0 * M_PI/3.0;
// const double WHEEL3_ANGLE = 4.0 * M_PI/3.0;

const double WHEEL1_ANGLE = 5.0 * M_PI/6.0;
const double WHEEL2_ANGLE = 3.0 * M_PI/2.0;
const double WHEEL3_ANGLE = 1.0 * M_PI/6.0;
const double WHEEL_RADIUS = .0415;  // meters
const double WHEEL_BASE_RADIUS = 0.1016/sqrt(3);   // 0.1016 meters is half the the triangle side formed by the 3 omniwheel car which is 0.2032 meters
/**
 *  Wheel Mode It is a moving speed to Goal direction. 0~2047 (0X7FF) can be used, and the unit is about 0.916rpm. 
 *  If a value in the range of 0~1023 is used, it is stopped by setting to 0 while rotating to CCW direction.
 *  If a value in the range of 1024~2047 is used, it is stopped by setting to 1024 while rotating to CW direction. 
 *  That is, the 10th bit becomes the direction bit to control the direction.  
 *  Factor is calculated by inverse of .916 rpm per unit 
*/
const double VELOCITY_FACTOR = 1.4; //1.0/0.916;   



class OmniWheelRelativeSpeeds : public rclcpp::Node{
public:
    OmniWheelRelativeSpeeds() : Node("omniwheel_node"){
        // initialize subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&OmniWheelRelativeSpeeds::cmd_vel_callback, this, std::placeholders::_1));
        // initialize publisher
        relative_speeds_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed>("relative_speeds", 10);

    }
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed>::SharedPtr relative_speeds_pub_;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg){
        
        /* Extract linear and angular velocities from cmd_vel messages */
        double linear_x = msg->linear.x;
        double linear_y = msg->linear.y;
        double angular_z = msg->angular.z;
        
        double angular_adjustment = 1.5;

        if(angular_z > 0) angular_z += angular_adjustment;  // adjust correct number of radians for this car configuration
        else if(angular_z < 0) angular_z -= angular_adjustment;
        

        
        /* Calculate the actual wheels RPM */
        
        double wheel_rpm_x = (linear_x * 60)/(2 * M_PI * WHEEL_RADIUS);
        double wheel_rpm_y = (linear_y * 60)/(2 * M_PI * WHEEL_RADIUS);
        double wheel_rpm_z = (angular_z * 60)/(2 * M_PI);

        /* Calculate the individual wheel velocities based on robot's linear and angular velocities */
        
        double wh1_vel = wheel_rpm_x * cos(WHEEL1_ANGLE) + wheel_rpm_y * sin(WHEEL1_ANGLE) + wheel_rpm_z * WHEEL_BASE_RADIUS / WHEEL_RADIUS;
        double wh2_vel = wheel_rpm_x * cos(WHEEL2_ANGLE) + wheel_rpm_y * sin(WHEEL2_ANGLE) + wheel_rpm_z * WHEEL_BASE_RADIUS / WHEEL_RADIUS;
        double wh3_vel = wheel_rpm_x * cos(WHEEL3_ANGLE) + wheel_rpm_y * sin(WHEEL3_ANGLE) + wheel_rpm_z * WHEEL_BASE_RADIUS / WHEEL_RADIUS;
        
        /* Calculate the wheel velocities
        * 
        * (Front left wheel 1) O--------O (front right ID: 3)
        * 150 deg-5pi/6 rads    \      /   30 deg or pi/6 rads
        *                        \    /
        *                         \  /
        *                           O (rear wheel ID: 2)
        *                               270 deg or 3pi/2 rads
        */

     
        auto goal_velocity = dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed();
      

        goal_velocity.m1 = wh1_vel * VELOCITY_FACTOR;
        goal_velocity.m2 = wh2_vel * VELOCITY_FACTOR;
        goal_velocity.m3 = wh3_vel * VELOCITY_FACTOR;
        RCLCPP_INFO(this->get_logger(),"speeds %f %f %f \n", wh1_vel * VELOCITY_FACTOR, wh2_vel * VELOCITY_FACTOR, wh3_vel*VELOCITY_FACTOR);

        relative_speeds_pub_->publish(goal_velocity);

    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OmniWheelRelativeSpeeds>());
    rclcpp::shutdown();

    return 0;
}

