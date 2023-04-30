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
        printf("speeds %f %f %f \n", wh1_vel, wh2_vel, wh3_vel);

        relative_speeds_pub_->publish(goal_velocity);

    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OmniWheelRelativeSpeeds>());
    rclcpp::shutdown();

    return 0;
}

/**
 * @brief CALCULATE THE GOAL SPEED BASED ON THE m/s and the rpm of the actual wheel
 * 
 * wheel radius = .1305 meters
 * wheel_rmp = (linear_velocity.x * 60)/(2 * pi * wheel_radius)
 * 
 * CCW is 0-1023 with 0.916 rpm per unit.
 * To determine the number of units (0-1023), we will divide the wheel_rmp by 0.916
 * 
 * Example:   wheel_rmp = (1 * 60)/(2 * pi * 0.1305) = 36.58734324
 * 
 * Divide    36.58734324/0.916 = 39.94251445 units out of 0-1023 .... maybe not...and just use the wheel_rpm
 * 
 * Use that number to multiply above in the code. e.g:
 * 
 *  double wh1_vel = linear_x * cos(WHEEL1_ANGLE) + linear_y * sin(WHEEL1_ANGLE) + angular_z;
 * 
 * double wheel1_rpm = ...; // RPM for wheel 1
double wheel2_rpm = ...; // RPM for wheel 2
double wheel3_rpm = ...; // RPM for wheel 3

double wheel1_rad_per_sec = (wheel1_rpm * 2 * M_PI) / 60;
double wheel2_rad_per_sec = (wheel2_rpm * 2 * M_PI) / 60;
double wheel3_rad_per_sec = (wheel3_rpm * 2 * M_PI) / 60;
 */


// /**
//  * @brief This was before calculations of radians per second which work exept for the angular z
//  * 
//  *   Calculate the actual wheels RPM */
        
//         double wheel_rpm_x = ((linear_x * 60)/(2 * M_PI * WHEEL_RADIUS))/0.916;
//         double wheel_rpm_y = ((linear_y * 60)/(2 * M_PI * WHEEL_RADIUS))/0.916;
//         double wheel_rpm_z = ((angular_z * 60)/(2 * M_PI * WHEEL_RADIUS))/0.916;

//          Get radians per second for angular_z */

       
        
//         /* Calculate the wheel velocities
//         * 
//         * (Front left wheel 1) O--------O (front right ID: 3)
//         * 150 deg-5pi/6 rads    \      /   30 deg or pi/6 rads
//         *                        \    /
//         *                         \  /
//         *                           O (rear wheel ID: 2)
//         *                               270 deg or 3pi/2 rads
//         */

//         // double wh1_vel = linear_x * cos(WHEEL1_ANGLE) + linear_y * sin(WHEEL1_ANGLE) + angular_z;
//         // double wh2_vel = linear_x * cos(WHEEL2_ANGLE) + linear_y * sin(WHEEL2_ANGLE) + angular_z;
//         // double wh3_vel = linear_x * cos(WHEEL3_ANGLE) + linear_y * sin(WHEEL3_ANGLE) + angular_z;
// 
//         double wh1_vel = wheel_rpm_x * cos(WHEEL1_ANGLE) + wheel_rpm_y * sin(WHEEL1_ANGLE) + wheel_rpm_z;
//         double wh2_vel = wheel_rpm_x * cos(WHEEL2_ANGLE) + wheel_rpm_y * sin(WHEEL2_ANGLE) + wheel_rpm_z;
//         double wh3_vel = wheel_rpm_x * cos(WHEEL3_ANGLE) + wheel_rpm_y * sin(WHEEL3_ANGLE) + wheel_rpm_z;

//   
//  

// this is bad with the angular velocity calculations
 /* Calculate the actual wheels RPM */
        
        // double wheel_rpm_x = (linear_x * 60)/(2 * M_PI * WHEEL_RADIUS);
        // double wheel_rpm_y = (linear_y * 60)/(2 * M_PI * WHEEL_RADIUS);
        // double wheel_rpm_z = (angular_z * 60)/(2 * M_PI * WHEEL_RADIUS);

        // /* Get radians per second for angular_z */

       
        
        // /* Calculate the wheel velocities
        // * 
        // * (Front left wheel 1) O--------O (front right ID: 3)
        // * 150 deg-5pi/6 rads    \      /   30 deg or pi/6 rads
        // *                        \    /
        // *                         \  /
        // *                           O (rear wheel ID: 2)
        // *                               270 deg or 3pi/2 rads
        // */

        // // double wh1_vel = linear_x * cos(WHEEL1_ANGLE) + linear_y * sin(WHEEL1_ANGLE) + angular_z;
        // // double wh2_vel = linear_x * cos(WHEEL2_ANGLE) + linear_y * sin(WHEEL2_ANGLE) + angular_z;
        // // double wh3_vel = linear_x * cos(WHEEL3_ANGLE) + linear_y * sin(WHEEL3_ANGLE) + angular_z;

        // double wh1_vel = wheel_rpm_x * cos(WHEEL1_ANGLE) + wheel_rpm_y * sin(WHEEL1_ANGLE) + (wheel_rpm_z * 2 * M_PI) / 60;
        // double wh2_vel = wheel_rpm_x * cos(WHEEL2_ANGLE) + wheel_rpm_y * sin(WHEEL2_ANGLE) + (wheel_rpm_z * 2 * M_PI) / 60;
        // double wh3_vel = wheel_rpm_x * cos(WHEEL3_ANGLE) + wheel_rpm_y * sin(WHEEL3_ANGLE) + (wheel_rpm_z * 2 * M_PI) / 60;

    //     void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg){
        
    //     /* Extract linear and angular velocities from cmd_vel messages */
    //     double linear_x = msg->linear.x;
    //     double linear_y = msg->linear.y;
    //     double angular_z = msg->angular.z;

        
    //     /* Calculate the actual wheels RPM */
        
    //     // double wheel_rpm_x = ((linear_x * 60)/(2 * M_PI * WHEEL_RADIUS);
    //     // double wheel_rpm_y = ((linear_y * 60)/(2 * M_PI * WHEEL_RADIUS);
    //     // double wheel_rpm_z = ((angular_z * 60)/(2 * M_PI * WHEEL_RADIUS);

    //     /* Calculate the individual wheel velocities based on robot's linear and angular velocities */
    //     double wheel1_rad_per_sec = (linear_x * cos(WHEEL1_ANGLE) + linear_y * sin(WHEEL1_ANGLE) + angular_z * WHEEL_RADIUS);
    //     double wheel2_rad_per_sec = (linear_x * cos(WHEEL2_ANGLE) + linear_y * sin(WHEEL2_ANGLE) + angular_z * WHEEL_RADIUS);
    //     double wheel3_rad_per_sec = (linear_x * cos(WHEEL3_ANGLE) + linear_y * sin(WHEEL3_ANGLE) + angular_z * WHEEL_RADIUS);      
        
    //     /* Calculate the wheel velocities
    //     * 
    //     * (Front left wheel 1) O--------O (front right ID: 3)
    //     * 150 deg-5pi/6 rads    \      /   30 deg or pi/6 rads
    //     *                        \    /
    //     *                         \  /
    //     *                           O (rear wheel ID: 2)
    //     *                               270 deg or 3pi/2 rads
    //     */

    //     // double wh1_vel = linear_x * cos(WHEEL1_ANGLE) + linear_y * sin(WHEEL1_ANGLE) + angular_z;
    //     // double wh2_vel = linear_x * cos(WHEEL2_ANGLE) + linear_y * sin(WHEEL2_ANGLE) + angular_z;
    //     // double wh3_vel = linear_x * cos(WHEEL3_ANGLE) + linear_y * sin(WHEEL3_ANGLE) + angular_z;

    //     // double wh1_vel = wheel_rpm_x * cos(WHEEL1_ANGLE) + wheel_rpm_y * sin(WHEEL1_ANGLE) + wheel_rpm_z;
    //     // double wh2_vel = wheel_rpm_x * cos(WHEEL2_ANGLE) + wheel_rpm_y * sin(WHEEL2_ANGLE) + wheel_rpm_z;
    //     // double wh3_vel = wheel_rpm_x * cos(WHEEL3_ANGLE) + wheel_rpm_y * sin(WHEEL3_ANGLE) + wheel_rpm_z;

    //        // Convert wheel velocities in rad/s to RPM
    //     double wheel1_rpm = (wheel1_rad_per_sec * 60) / (2 * M_PI);
    //     double wheel2_rpm = (wheel2_rad_per_sec * 60) / (2 * M_PI);
    //     double wheel3_rpm = (wheel3_rad_per_sec * 60) / (2 * M_PI);

    //     auto relative_speeds = dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed();
    //     // relative_speeds.m1 = wh1_vel;
    //     // relative_speeds.m2 = wh2_vel;
    //     // relative_speeds.m3 = wh3_vel;

    //     relative_speeds.m1 = wheel1_rpm;
    //     relative_speeds.m2 = wheel2_rpm;
    //     relative_speeds.m3 = wheel3_rpm;
    //     printf("speeds %f %f %f \n", wheel1_rpm, wheel2_rpm, wheel3_rpm);

    //     relative_speeds_pub_->publish(relative_speeds);

    // }