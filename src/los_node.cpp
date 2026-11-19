#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
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
        // Trajetoria de reserva: o YAML substitui este valor ao iniciar o no.
        // Formato: [x0, y0, x1, y1, ...], em metros (referencial de /odom).
        const std::vector<double> default_waypoints = {
            0.000,  0.000,
            0.439, -1.061,
            1.500, -1.500,
            2.561, -1.061,
            3.000,  0.000,
            2.561,  1.061,
            1.500,  1.500,
            0.439,  1.061
        };

        // O parametro e lido ao iniciar; para alterar a trajetoria em execucao,
        // publicar no topico /tsb/waypoints.
        const auto initial_waypoints =
            this->declare_parameter<std::vector<double>>(
                "waypoints", default_waypoints);

        if (!set_waypoints(initial_waypoints))
        {
            throw std::runtime_error("Parametro 'waypoints' inicial invalido.");
        }

        position_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tsb/position", 10,
            std::bind(&LOSNode::position_callback, this, std::placeholders::_1));

        waypoints_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tsb/waypoints", 10,
            std::bind(&LOSNode::waypoints_callback, this, std::placeholders::_1));

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
    // Recebe coordenadas planas e valida a trajetoria completa antes de a aceitar.
    bool set_waypoints(const std::vector<double> & data)
    {
        // Minimo de 3 pontos para um loiter poligonal fechado.
        if (data.size() < 6 || data.size() % 2 != 0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Waypoints invalidos: indicar pelo menos 3 pares [x,y].");
            return false;
        }

        std::vector<Waypoint> candidate;
        candidate.reserve(data.size() / 2);

        for (std::size_t i = 0; i < data.size(); i += 2)
        {
            if (!std::isfinite(data[i]) || !std::isfinite(data[i + 1]))
            {
                RCLCPP_WARN(this->get_logger(),
                            "Waypoints invalidos: coordenadas nao finitas.");
                return false;
            }
            candidate.push_back({data[i], data[i + 1]});
        }

        // Evita segmentos de comprimento zero, incluindo o ultimo -> primeiro.
        for (std::size_t i = 0; i < candidate.size(); ++i)
        {
            const Waypoint & a = candidate[i];
            const Waypoint & b = candidate[(i + 1) % candidate.size()];
            if (std::hypot(b.x - a.x, b.y - a.y) < 1e-6)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Waypoints invalidos: existem pontos consecutivos iguais.");
                return false;
            }
        }

        waypoints_.swap(candidate);
        segment_index_ = 0;  // A nova trajetoria comeca no segmento P1 -> P2.
        RCLCPP_INFO(this->get_logger(),
                    "Trajetoria aceite: %zu waypoints; segmento reiniciado.",
                    waypoints_.size());
        return true;
    }

    void waypoints_callback(
        const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Se invalida, mantem a ultima trajetoria valida sem interrupcao.
        if (!set_waypoints(msg->data))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Nova trajetoria rejeitada; a anterior foi mantida.");
        }
    }

    void position_callback(
        const std_msgs::msg::Float64MultiArray::SharedPtr msg)
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
        // Nao comandar avancar se nao houver posicao recente.
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
        for (std::size_t attempts = 0; attempts < waypoints_.size(); ++attempts)
        {
            const Waypoint & start = waypoints_[segment_index_];
            const Waypoint & end =
                waypoints_[(segment_index_ + 1) % waypoints_.size()];

            const double dx = end.x - start.x;
            const double dy = end.y - start.y;
            const double length = std::hypot(dx, dy);
            const double distance_to_end = std::hypot(end.x - x_, end.y - y_);
            const double along_track = ((x_ - start.x) * dx +
                                        (y_ - start.y) * dy) / length;
            const double cross_track = ((x_ - start.x) * (-dy) +
                                        (y_ - start.y) * dx) / length;

            // Avancar por proximidade ou se ultrapassou o fim do segmento.
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
        const Waypoint & end =
            waypoints_[(segment_index_ + 1) % waypoints_.size()];

        const double chi_p = std::atan2(end.y - start.y, end.x - start.x);
        const double e = -(x_ - start.x) * std::sin(chi_p)
                         + (y_ - start.y) * std::cos(chi_p);
        const double psi_d = chi_p - std::atan2(e, lookahead_);
        last_heading_ref_ = std::atan2(std::sin(psi_d), std::cos(psi_d));

        publish_reference(desired_speed_, last_heading_ref_);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Loiter | segmento %zu/%zu | erro lateral %.3f m | heading ref %.3f rad",
            segment_index_ + 1, waypoints_.size(), e, last_heading_ref_);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr waypoints_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr reference_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Ganhos geometricos e velocidade fixos em C++ (nao sao parametros ROS).
    const double lookahead_ = 0.70;           // m
    const double desired_speed_ = 0.10;       // m/s
    const double waypoint_tolerance_ = 0.20;  // m

    std::vector<Waypoint> waypoints_;
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
