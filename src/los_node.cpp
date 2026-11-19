#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

struct Waypoint
{
    double x;
    double y;
};

class LOSNode : public rclcpp::Node
{
public:
    LOSNode() : Node("los_node")
    {
        position_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tsb/position", 10,
            std::bind(&LOSNode::position_callback, this, std::placeholders::_1));

        reference_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/tsb/reference", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LOSNode::control_callback, this));

        RCLCPP_INFO(this->get_logger(),
                    "LOS iniciado: loiter continuo com %zu waypoints.",
                    waypoints_.size());
    }

private:
    void position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 2 ||
            !std::isfinite(msg->data[0]) || !std::isfinite(msg->data[1]))
        {
            has_position_ = false;
            RCLCPP_WARN(this->get_logger(), "Posicao invalida: esperado [x, y].");
            return;
        }

        x_ = msg->data[0];
        y_ = msg->data[1];
        has_position_ = true;
        last_position_time_ = std::chrono::steady_clock::now();
    }

    void publish_reference(double speed, double heading)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {speed, heading};
        reference_pub_->publish(msg);
    }

    void control_callback()
    {
        // Nao comandar movimento se nao houver posicao recente.
        if (!has_position_ ||
            std::chrono::steady_clock::now() - last_position_time_ >
                std::chrono::seconds(1))
        {
            publish_reference(0.0, last_heading_ref_);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "A aguardar posicao atualizada.");
            return;
        }

        // O ultimo segmento liga o ultimo waypoint ao primeiro.
        // Quando chega a um waypoint, avanca de forma circular e nunca termina.
        for (std::size_t attempts = 0; attempts < waypoints_.size(); ++attempts)
        {
            const Waypoint & start = waypoints_[segment_index_];
            const Waypoint & end = waypoints_[(segment_index_ + 1) % waypoints_.size()];

            const double dx = end.x - start.x;
            const double dy = end.y - start.y;
            const double length = std::hypot(dx, dy);
            const double distance_to_end = std::hypot(end.x - x_, end.y - y_);
            const double along_track = ((x_ - start.x) * dx +
                                        (y_ - start.y) * dy) / length;
            const double cross_track = ((x_ - start.x) * (-dy) +
                                        (y_ - start.y) * dx) / length;

            // Avancar por proximidade ou quando ultrapassamos o fim do segmento.
            if (distance_to_end > waypoint_tolerance_ &&
                !(along_track >= length && std::abs(cross_track) <= lookahead_))
            {
                break;
            }

            segment_index_ = (segment_index_ + 1) % waypoints_.size();
            RCLCPP_INFO(this->get_logger(), "Novo segmento: %zu de %zu.",
                        segment_index_ + 1, waypoints_.size());
        }

        const Waypoint & start = waypoints_[segment_index_];
        const Waypoint & end = waypoints_[(segment_index_ + 1) % waypoints_.size()];

        // Angulo do segmento no referencial da odometria.
        const double chi_p = std::atan2(end.y - start.y, end.x - start.x);

        // Erro transversal positivo a esquerda do segmento.
        const double e = -(x_ - start.x) * std::sin(chi_p)
                         + (y_ - start.y) * std::cos(chi_p);

        // Lei Line-of-Sight (LOS).
        const double psi_d = chi_p - std::atan2(e, lookahead_);
        last_heading_ref_ = std::atan2(std::sin(psi_d), std::cos(psi_d));

        publish_reference(desired_speed_, last_heading_ref_);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Loiter | segmento %zu/%zu | erro lateral %.3f m | heading ref %.3f rad",
            segment_index_ + 1, waypoints_.size(), e, last_heading_ref_);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr reference_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Valores fixos
    const double lookahead_ = 0.70;    
    const double desired_speed_ = 0.10;
    const double waypoint_tolerance_ = 0.20;

    // Loiter circular aproximado por 8 segmentos
    const std::vector<Waypoint> waypoints_ = {
        {0.000,  0.000},
        {0.439, -1.061},
        {1.500, -1.500},
        {2.561, -1.061},
        {3.000,  0.000},
        {2.561,  1.061},
        {1.500,  1.500},
        {0.439,  1.061}
    };

    std::size_t segment_index_ = 0;
    double x_ = 0.0;
    double y_ = 0.0;
    double last_heading_ref_ = 0.0;
    bool has_position_ = false;
    std::chrono::steady_clock::time_point last_position_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LOSNode>());
    rclcpp::shutdown();
    return 0;
}
