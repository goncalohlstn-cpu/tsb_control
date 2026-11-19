#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <functional>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>
#include <chrono>
#include <algorithm>
#include "geometry_msgs/msg/twist.hpp"

class PIDNode : public rclcpp::Node
{
public:
    PIDNode() : Node("pid_node")
    {
        // Parametros do controlador de surge
        kp_surge_ = this->declare_parameter<double>("kp_surge", 1.0);
        ki_surge_ = this->declare_parameter<double>("ki_surge", 0.2);
        kd_surge_ = this->declare_parameter<double>("kd_surge", 0.05);

        // Parametros do controlador de heading
        kp_heading_ = this->declare_parameter<double>("kp_heading", 1.0);
        ki_heading_ = this->declare_parameter<double>("ki_heading", 0.1);
        kd_heading_ = this->declare_parameter<double>("kd_heading", 0.2); 
        
        //Parametros de Limites de anti-windup
        surge_integral_limit_ = this->declare_parameter<double>(
        "surge_integral_limit", 1.0);

        heading_integral_limit_ = this->declare_parameter<double>(
        "heading_integral_limit", 1.0);

        //Parametros de Limites de saturacao dos comandos
        max_surge_cmd_ = this->declare_parameter<double>(
        "max_surge_cmd", 0.15);

        max_yaw_rate_cmd_ = this->declare_parameter<double>(
        "max_yaw_rate_cmd", 1.0);
        
        // Subscribe to /tsb/surge
        surge_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/tsb/surge",10,
        std::bind(&PIDNode::surge_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "PID Node iniciado!");

        // Subscribe to /tsb/heading
        heading_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/tsb/heading",10,
        std::bind(&PIDNode::heading_callback, this, std::placeholders::_1)
        );

        // Subscribe to /tsb/yaw_rate
        yaw_rate_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/tsb/yaw_rate",10,
        std::bind(&PIDNode::yaw_rate_callback, this, std::placeholders::_1)
        );

        // Subscribe to /tsb/reference
        reference_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/tsb/reference",10,
        std::bind(&PIDNode::reference_callback,this,std::placeholders::_1)
        );


        //========================================================================
        //Publicar os comandos
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10
        );

        //========================================================================
        // Cria um timer que executa control_callback() a cada 100 ms (10 Hz).
        control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PIDNode::control_callback, this)
        );
    }
private:
    void surge_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        if (!std::isfinite(msg->data))
        {
            has_surge_ = false;
            RCLCPP_WARN(this->get_logger(), "Surge invalido!");
            return;
        }

        surge_ = msg->data;
        has_surge_ = true;
        last_surge_time_ = std::chrono::steady_clock::now();

    }
    void heading_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        if (!std::isfinite(msg->data))
        {
            has_heading_ = false;
            RCLCPP_WARN(this->get_logger(), "Heading invalido!");
            return;
        }

        heading_ = msg->data;
        has_heading_ = true;
        last_heading_time_ = std::chrono::steady_clock::now();
     }
    void yaw_rate_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        if (!std::isfinite(msg->data))
        {
            has_yaw_rate_ = false;
            RCLCPP_WARN(this->get_logger(), "Yaw rate invalido!");
            return;
        }

        yaw_rate_ = msg->data;
        has_yaw_rate_ = true;
        last_yaw_rate_time_ = std::chrono::steady_clock::now();
    }
    void reference_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 2)
        {
            has_reference_ = false;
            RCLCPP_WARN(this->get_logger(),"A referencia deve conter 2 valores!");
            return;
        }
        if (!std::isfinite(msg->data[0]) ||
            !std::isfinite(msg->data[1]))
        {
            has_reference_ = false;
            RCLCPP_WARN(this->get_logger(), "Referencia invalida!");
            return;
        }

        has_reference_ = true;

        surge_ref_ = msg->data[0];
        heading_ref_ = msg->data[1];

        RCLCPP_INFO(this->get_logger(),"Surge ref: %.3f m/s | Heading ref: %.3f rad",
                    surge_ref_, heading_ref_);
    }

    void control_callback()
    {
        //garantir que temos referncias e dados dos sensores inicialmente
        if (!has_surge_ || !has_heading_ ||
            !has_yaw_rate_ || !has_reference_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "A aguardar dados dos sensores e referencias..."
            );

            publish_stop();

            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(1);

        //garantir que os sensores não deixaram de publicar
        if ((now - last_surge_time_ > timeout) ||
            (now - last_heading_time_ > timeout) ||
            (now - last_yaw_rate_time_ > timeout))
        {
            surge_integral_ = 0.0;
            heading_integral_ = 0.0;
            surge_error_initialized_ = false;

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Sensor timeout! Controlador suspenso."
            );

            publish_stop();

            return;
        }
        double surge_error = surge_ref_ - surge_;
        double surge_p = kp_surge_ * surge_error;
        
       surge_integral_ += surge_error * dt_;
       
       //antiwindup do controlador integral de surge
       surge_integral_ = std::max(
            -surge_integral_limit_,
            std::min(surge_integral_, surge_integral_limit_)
        );

        double surge_i = ki_surge_ * surge_integral_;

        double surge_d = 0.0;
        if (surge_error_initialized_)
        {
            surge_d = kd_surge_ *
                    (surge_error - previous_surge_error_) / dt_;
        }

        previous_surge_error_ = surge_error;
        surge_error_initialized_ = true;

        double surge_pid = surge_p + surge_i + surge_d;




        double heading_error = heading_ref_ - heading_;

        heading_error = std::atan2(
            std::sin(heading_error),
            std::cos(heading_error)
        );

        double heading_p = kp_heading_ * heading_error;
        double heading_d = -kd_heading_ * yaw_rate_;
        
        heading_integral_ += heading_error * dt_;

        //anti-windup do controlador integral de heading
        heading_integral_ = std::max(
            -heading_integral_limit_,
            std::min(heading_integral_, heading_integral_limit_)
        );

        double heading_i = ki_heading_ * heading_integral_;

        double heading_pid = heading_p + heading_i + heading_d;

        //criar a variavel para gerar os comandos
        geometry_msgs::msg::Twist cmd;
        //comando de surge com feedforward
        cmd.linear.x = surge_ref_ + surge_pid;
        //comando de heading rate a partir do erro de heading
        cmd.angular.z = heading_pid;

        //Saturação dos comandos
        cmd.linear.x = std::clamp(
        cmd.linear.x, -max_surge_cmd_, max_surge_cmd_);

        cmd.angular.z = std::clamp(
        cmd.angular.z, -max_yaw_rate_cmd_, max_yaw_rate_cmd_);

        //Publicar os comandos no topico /cmd_vel atraves do publicador
        // cmd_vel_pub_
        cmd_vel_pub_->publish(cmd);



        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Surge | P: %.3f | I: %.3f | D: %.3f | PID: %.3f"
            " ||Heading | P: %.3f | I: %.3f | D: %.3f | PID: %.3f",
            surge_p, surge_i, surge_d, surge_pid,
            heading_p, heading_i, heading_d, heading_pid
        );

        if (std::abs(surge_d) > 1e-6)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Componente derivativa: %.4f",
                surge_d
            );
        }

    }

    //Esta função garante que se os dados do sensor deixarem de ser
    //publicados enviamos ordem de parar ao turtlebot
    void publish_stop()
    {
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
    }


    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr surge_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_rate_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr reference_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::chrono::steady_clock::time_point last_surge_time_;
    std::chrono::steady_clock::time_point last_heading_time_;
    std::chrono::steady_clock::time_point last_yaw_rate_time_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;


    double yaw_rate_ = 0.0;
    double heading_ = 0.0;
    double surge_ = 0.0;
    double surge_ref_ = 0.0;
    double heading_ref_ = 0.0;

    double kp_surge_ = 1.0;
    double ki_surge_ = 0.2;
    double surge_integral_ = 0.0;
    double dt_ = 0.1;
    double kd_surge_ = 0.05;
    double previous_surge_error_ = 0.0;
    bool surge_error_initialized_ = false;


    double kp_heading_ = 1.0;
    double kd_heading_ = 0.2;
    double ki_heading_ = 0.1;
    double heading_integral_ = 0.0;

    double surge_integral_limit_ = 1.0;
    double heading_integral_limit_ = 1.0;

    bool has_surge_ = false;
    bool has_heading_ = false;
    bool has_yaw_rate_ = false;
    bool has_reference_ = false;

    double max_surge_cmd_ = 0.15;
    double max_yaw_rate_cmd_ = 1.0;
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PIDNode>());

    rclcpp::shutdown();
    return 0;
}
