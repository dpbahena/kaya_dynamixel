#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "geometry_msgs/msg/twist.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_relative_speed.hpp"




// const double WHEEL1_ANGLE = 0.0;
// const double WHEEL2_ANGLE = 2.0 * M_PI/3.0;
// const double WHEEL3_ANGLE = 4.0 * M_PI/3.0;

const double WHEEL1_ANGLE = 5.0 * M_PI/6.0;
const double WHEEL2_ANGLE = 3.0 * M_PI/2.0;
const double WHEEL3_ANGLE = 1.0 * M_PI/6.0;



class OmniWheelRelativeSpeeds : public rclcpp::Node{
public:
    OmniWheelRelativeSpeeds() : Node("omniwheel_node"){
        // initialize subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&OmniWheelRelativeSpeeds::cmd_vel_callback, this, std::placeholders::_1));
        // initialize publisher
        relative_speeds_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed>("relative_speeds", 1);

    }
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed>::SharedPtr relative_speeds_pub_;
    void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg){
        /* Extract linear and angular velocities from cmd_vel messages */
        double linear_x = msg->linear.x;
        double linear_y = msg->linear.y;
        double angular_z = msg->angular.z;

        
        /* Calculate the wheel velocities
        * 
        * (Front left wheel 1) O--------O (front right ID: 3)
        * 150 deg-5pi/6 rads    \      /   30 deg or pi/6 rads
        *                        \    /
        *                         \  /
        *                           O (rear wheel ID: 2)
        *                               270 deg or 3pi/2 rads
        */

        double wh1_vel = linear_x * cos(WHEEL1_ANGLE) + linear_y * sin(WHEEL1_ANGLE) + angular_z;
        double wh2_vel = linear_x * cos(WHEEL2_ANGLE) + linear_y * sin(WHEEL2_ANGLE) + angular_z;
        double wh3_vel = linear_x * cos(WHEEL3_ANGLE) + linear_y * sin(WHEEL3_ANGLE) + angular_z;


        auto relative_speeds = dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed();
        relative_speeds.m1 = wh1_vel;
        relative_speeds.m2 = wh2_vel;
        relative_speeds.m3 = wh3_vel;

        relative_speeds_pub_->publish(relative_speeds);

    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OmniWheelRelativeSpeeds>());
    rclcpp::shutdown();

    return 0;
}