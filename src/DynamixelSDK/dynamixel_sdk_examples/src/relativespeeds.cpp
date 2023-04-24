#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_relative_speed.hpp"
#include "Eigen/Dense"

using namespace Eigen;
using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief  Command to publish Vel command test
 * ros2 topic pub -1 /vel_cmds geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}" 
 * 
 */



class RelativeSpeeds : public rclcpp::Node{
public:
    RelativeSpeeds();

private:
    float x_comp{}, y_comp{}, w_comp{};  // componets of the direction vector
    bool velocity_received;
    float average_velocity;  // from linear.x, linear.y and angular.z   
    //float velocity;
    bool isLinear = false;
    bool isAngular = false;    //velocity received is either linear or angular 
    float m1, m2, m3;  // receives the relative speeds for each motor
    const int maxPwm = 440;
    const float maxLVelocity = 2.0; // feet per seconds
    const float maxAVelocity = 3.0; // radians per second

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed>::SharedPtr relative_speeds_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void init_subscribers();
    void init_publishers();

    void velocity_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
    void relative_speeds_callback();
    /* Set relative speeds for each motor */
    void set_relative_speeds(); 
    //void set_direction_components(int direction);
    float mapVelocity(float v);
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelativeSpeeds>());
    rclcpp::shutdown();

    return 0;
}

RelativeSpeeds::RelativeSpeeds() : Node("relativespeeds_node"){
    init_subscribers();
    init_publishers();  
}

void RelativeSpeeds::init_subscribers(){
    velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&RelativeSpeeds::velocity_callback, this, _1));
}
void RelativeSpeeds::init_publishers(){
    relative_speeds_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed>("relative_speeds", 1);
    timer_ = this->create_wall_timer(100ms, std::bind(&RelativeSpeeds::relative_speeds_callback, this));
}

void RelativeSpeeds::velocity_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg){
    
    if(!msg->linear.x && !msg->linear.y && !msg->angular.z){
        x_comp = 0;
        y_comp = 0;
        w_comp = 0;
        velocity_received = false;
        return;
    }

    auto angle = atan(msg->linear.x/msg->linear.y);
    float bearing{};
    
    average_velocity = (abs(msg->linear.x) + abs(msg->linear.y) + abs(msg->angular.z))/3;
    
    if (msg->angular.z != 0)
        isAngular = true;
  
        
    if (msg->linear.x > 0 && msg->linear.y == 0){  // moved up (North) only
        /* x_comp = 0;
        y_comp = 1;
        w_comp = 0; */
        bearing = M_PI/2;
        isLinear = true;
        //std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " UP" << "  Bearing = " << bearing <<  std::endl;

    }
    else if (msg->linear.x < 0 && msg->linear.y == 0){  //moved down (South) only
        /* x_comp = 0;
        y_comp = -1;
        w_comp = 0; */
        bearing = 3*M_PI/2;
        isLinear = true;
        //std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " DOWN" <<  "  Bearing = " << bearing << std::endl;
    }

    else if(msg->linear.y > 0  && msg->linear.x == 0){   // moved to the right (East) only
        /* x_comp = 1;
        y_comp = 0;
        w_comp = 0; */
        bearing = 0;
        isLinear = true;
        //std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " RIGHT" <<  "  Bearing = " << bearing << std::endl;
    }

    else if(msg->linear.y < 0 && msg->linear.x == 0 ){   // moved to the left (West) only
        /* x_comp = -1;
        y_comp = 0;
        w_comp = 0; */
        bearing = M_PI;
        isLinear = true;
        //std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " LEFT" <<  "  Bearing = " << bearing << std::endl;
    }
    else{   // get the bearing angle
        if(msg->linear.x > 0 && msg->linear.y > 0){   // Quadrant 1
            bearing = M_PI/2 - angle;
            std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " Quadrant I" <<  std::endl;
        }
        if(msg->linear.x < 0 && msg->linear.y > 0){   // Quadrant II
            bearing = M_PI/2 - angle;
            std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " Quadrant II" <<  std::endl;
        }
        if(msg->linear.x < 0 && msg->linear.y < 0){   // Quadrant III
            bearing = 3*M_PI/2 - angle;
            std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " Quadrant III" <<  std::endl;
        }
        if(msg->linear.x > 0 && msg->linear.y < 0){   // Quadrant IV
            bearing = 3*M_PI/2 - angle;
            std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " Quadrant IV" <<  std::endl;
        }
        isLinear = true;
    }

    // if(msg->linear.x || msg->linear.y){   // If this axes were moved then do the calculations. Just in case only rotation was activated
    //     x_comp = cos(bearing);
    //     y_comp = sin(bearing);
    // }

    x_comp = cos(bearing);
    y_comp = sin(bearing);
    w_comp = msg->angular.z;
    
    set_relative_speeds();
    velocity_received = true;

    
}

void RelativeSpeeds::set_relative_speeds(){
    
    //std::cout << "hi" << std::endl;
    Matrix3f m;
    m << -0.33,0.58,0.33,
         -0.33,-0.58,0.33,
          0.67, 0, 0.33;
    
    // Matrix3f m {
    // {-0.33,0.58,0.33},
    // {-0.33,-0.58,0.33},
    // {0.67, 0, 0.33},
    // };
    //std::cout << "ariana" << m << std::endl;

    //std::cout << m << std::endl;

    MatrixXf n(3,1);
    n << x_comp, y_comp, w_comp;

    // Matrix3f n {
    //     {x_comp},
    //     {y_comp},
    //     {w_comp},
    // };
   
    MatrixXf motorSpeeds = m*n;
    
    std::cout << "Components: {"<< x_comp <<", " << y_comp << ", " << w_comp << "}" << std::endl;

    // extract each indiviual motor speed from the matrix and publish it.
    m1 = motorSpeeds.coeff(0,0);
    m2 = motorSpeeds.coeff(1,0);
    m3 = motorSpeeds.coeff(2,0);
    std::cout << "motorSpeeds :" << m1 << " and " << m2 << " and " << m3 << std::endl;
}

void RelativeSpeeds::relative_speeds_callback(){

    /* variable to be published */
    dynamixel_sdk_custom_interfaces::msg::SetRelativeSpeed motorspeeds; // holds the relative speeds of each motor in an array

    //auto pwm = mapVelocity(average_velocity);
    //auto pwm = mapVelocity(velocity);
    /*
    * (Front left ID: 1) O--------O (front right ID: 3)
    *                     \      /
    *                      \    /
    *                       \  /
    *                         O (rear wheel ID: 2)
    */
    
    // motorspeeds.m1 = m2;  
    // motorspeeds.m2 = m3;   // USING ABOVE CONFIG, switch m1/m3 m2/m1 m3/m2
    // motorspeeds.m3 = m1;
    
    motorspeeds.m1 = m1;  
    motorspeeds.m2 = m2;   
    motorspeeds.m3 = m3;

    /* 4th element of array is the adjusted pwm */
    //motorspeeds.data.push_back(pwm);      

    if(velocity_received){
        relative_speeds_pub_->publish(motorspeeds);
        velocity_received = false;
        
    }
        
}