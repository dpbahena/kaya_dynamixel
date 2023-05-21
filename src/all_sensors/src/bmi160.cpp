#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "bmi160/bmi160-i2c.h"
#include <deque>

#define BMI160_I2C_ADDR  UINT8_C(0x69)

using namespace std::chrono_literals;

class KalmanFilter {
private:
    double q; // process noise covariance
    double r; // measurement noise covariance
    double p; // estimation error covariance
    double x; // estimated value
    double k; // kalman gain

public:
    KalmanFilter(double initial_q, double initial_r, double initial_p, double initial_x)
        : q(initial_q), r(initial_r), p(initial_p), x(initial_x)
    {
    }

    double update(double measurement)
    {
        // prediction update
        p = p + q;

        // measurement update
        k = p / (p + r);
        x = x + k * (measurement - x);
        p = (1 - k) * p;

        return x;
    }
};


class MovingAverage {
private:
    size_t window_size;
    std::deque<double> samples;
    double sum;

public:
    MovingAverage(size_t size) : window_size(size), sum(0) {}

    double compute(double sample) {
        if (samples.size() >= window_size) {
            sum -= samples.front();
            samples.pop_front();
        }

        samples.push_back(sample);
        sum += sample;

        return sum / samples.size();
    }
}; 

class AccelGyro : public rclcpp::Node{
public:
    AccelGyro() : Node("accegyro_node"){
        bmi160 = new Bmi160_I2C(BMI160_I2C_ADDR);
        //filter1 = new MovingAverage(15);
        kfilter1 = new KalmanFilter(0.003,0.2,1.0,3.0);
        //filter2 = new MovingAverage(10);
        kfilter2 = new KalmanFilter(0.003,0.2,1.0,2.0);
        //filter3 = new MovingAverage(10);
        kfilter3 = new KalmanFilter(0.003,0.2,1.0,2.0);
        
        bmi160->autoCalibrateGyroOffset();
        imuPub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu",10);
        timer_ = this->create_wall_timer(100ms, std::bind(&AccelGyro::sensor,this));
    
    }
    ~AccelGyro(){

        delete bmi160;
        delete kfilter1;
        delete kfilter2;
        delete kfilter3;
    }

   
    
private:
    
    Bmi160_I2C *bmi160;
    //MovingAverage *filter1, *filter2, *filter3;
    KalmanFilter *kfilter1, *kfilter2, *kfilter3;
    

    /* publisher */
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
    rclcpp::TimerBase::SharedPtr timer_;
    


    void sensor(){
        auto msg = sensor_msgs::msg::Imu();
        int16_t data[6];
        bmi160->getMotion_6DOF(data);
        msg.angular_velocity.x = kfilter1->update(data[0]);  // radians/s  // degrees per second *M_PI/180.0;
        msg.angular_velocity.y = kfilter2->update(data[1]);                //*M_PI/180.0;
        msg.angular_velocity.z = kfilter3->update(data[2]);                //*M_PI/180.0;
        msg.linear_acceleration.x = data[3]/16384.0;
        msg.linear_acceleration.y = data[4]/16384.0;
        msg.linear_acceleration.z = data[5]/16384.0;
        
        imuPub_->publish(msg);
    }
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AccelGyro>());
    rclcpp::shutdown();

    return 0;
}