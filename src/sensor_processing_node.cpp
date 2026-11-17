
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

class SensorProcessingNode : public rclcpp::Node
{
public:
    SensorProcessingNode() : Node("sensor_processing_node")
    {
        //============================================================================================
        //Isto cria 3 publisher que envia mensagens para os tópicos /tsb/surge  /tsb/heading /tsb/yaw_rate
        surge_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/tsb/surge", 10
        );
        heading_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/tsb/heading", 10
        );
        yaw_rate_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/tsb/yaw_rate", 10
        );
        //============================================================================================

        // Subscribe to ODOM
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                double surge = msg->twist.twist.linear.x;
                
                //=====================================================================================
                //Aqui criamos uma mensagem, atribuímos-lhe o valor da velocidade medida e publicamo-la.
                std_msgs::msg::Float64 surge_msg;
                surge_msg.data = surge;

                surge_pub_->publish(surge_msg);
                //======================================================================================

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Surge velocity: %.4f m/s",
                    surge
                );
            }
        );

        // Subscribe to IMU
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                // Read the orientation quaternion
                double x = msg->orientation.x;
                double y = msg->orientation.y;
                double z = msg->orientation.z;
                double w = msg->orientation.w;

                // Convert quaternion to heading (yaw)
                double heading = std::atan2(
                    2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z)
                );

                // Read yaw rate
                double yaw_rate = msg->angular_velocity.z;

                std_msgs::msg::Float64 heading_msg;
                heading_msg.data = heading;
                heading_pub_->publish(heading_msg);
                
                std_msgs::msg::Float64 yaw_rate_msg;
                yaw_rate_msg.data = yaw_rate;
                yaw_rate_pub_->publish(yaw_rate_msg);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Heading: %.4f rad | Yaw rate: %.4f rad/s",
                    heading, yaw_rate
                );
            }
        );

    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr surge_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SensorProcessingNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
