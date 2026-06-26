#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include <algorithm>
#include <vector>

#include "unitree_nav_bridge/ros2_sport_client.h"

/**
 * @brief Puente de navegación Nav2 <-> Unitree (Go2).
 *
 * Traduce los comandos de velocidad estándar de ROS 2 (geometry_msgs/Twist en
 * /cmd_vel, publicados por Nav2) a la API de movimiento propietaria de Unitree
 * (unitree_api/Request vía SportClient::Move).
 *
 * El SDK de Unitree expone la primitiva Move(), pero no incluye la integración
 * con el stack de navegación ni la lógica de seguridad. Este nodo añade:
 *   - Watchdog: si Nav2 deja de publicar, el robot se detiene.
 *   - Verificación de modo: solo se mueve si el cuadrúpedo está en un modo de
 *     locomoción válido (evita enviar comandos con los motores en damping).
 *   - Saturación de velocidades dentro de los límites del robot.
 *   - Loop de control a frecuencia fija, desacoplado de la llegada de comandos.
 *
 * Todos los valores operativos son parámetros de ROS 2 (ver config/bridge_params.yaml).
 */
class NavToUnitreeBridge : public rclcpp::Node {
public:
    NavToUnitreeBridge() : Node("nav_to_unitree_bridge") {
        // --- Parámetros configurables (con valores por defecto para el Go2) ---
        cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        state_topic_   = declare_parameter<std::string>("state_topic", "lf/sportmodestate");

        control_frequency_ = declare_parameter<double>("control_frequency", 50.0);
        cmd_timeout_       = declare_parameter<double>("cmd_timeout", 0.3);

        // Modos de SportModeState en los que es seguro mover el robot.
        // Por defecto: 0 (desconocido/arranque), 7 y 9 (de pie / caminando).
        // 12 normalmente es Damping (motores apagados). Ajustar según firmware.
        valid_modes_ = declare_parameter<std::vector<int64_t>>(
            "valid_locomotion_modes", std::vector<int64_t>{0, 7, 9});

        // Límites de velocidad (m/s y rad/s) acordes al hardware.
        max_vx_   = declare_parameter<double>("max_vx", 1.5);
        min_vx_   = declare_parameter<double>("min_vx", -1.0);
        max_vy_   = declare_parameter<double>("max_vy", 0.5);
        max_vyaw_ = declare_parameter<double>("max_vyaw", 1.0);

        sport_client_ = std::make_shared<SportClient>(this);

        const auto qos = rclcpp::SensorDataQoS();

        // 1. Suscriptor a Nav2 (cmd_vel)
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, qos,
            std::bind(&NavToUnitreeBridge::cmdVelCallback, this, std::placeholders::_1));

        // 2. Suscriptor al estado del robot (modo de locomoción actual)
        state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
            state_topic_, 1,
            std::bind(&NavToUnitreeBridge::stateCallback, this, std::placeholders::_1));

        // 3. Loop de control a frecuencia fija
        const auto period = std::chrono::duration<double>(1.0 / control_frequency_);
        control_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&NavToUnitreeBridge::controlLoop, this));

        last_cmd_time_ = now();

        RCLCPP_INFO(get_logger(),
                    "Puente Nav2-Unitree iniciado | cmd_vel='%s' estado='%s' | "
                    "control=%.0f Hz watchdog=%.2fs",
                    cmd_vel_topic_.c_str(), state_topic_.c_str(),
                    control_frequency_, cmd_timeout_);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_twist_ = *msg;
        last_cmd_time_ = now();
    }

    // Guarda el modo de locomoción actual reportado por el cuadrúpedo.
    void stateCallback(const unitree_go::msg::SportModeState::SharedPtr msg) {
        robot_mode_ = msg->mode;
    }

    bool isLocomotionModeValid() const {
        return std::find(valid_modes_.begin(), valid_modes_.end(),
                         static_cast<int64_t>(robot_mode_)) != valid_modes_.end();
    }

    void controlLoop() {
        unitree_api::msg::Request req;

        // SEGURIDAD 1: si el robot no está en un modo de locomoción válido,
        // ignorar a Nav2 por completo para no forzar el hardware.
        if (!isLocomotionModeValid()) {
            return;
        }

        // SEGURIDAD 2: watchdog. Si Nav2 dejó de publicar, detener el robot.
        const double dt = (now() - last_cmd_time_).seconds();
        if (dt > cmd_timeout_) {
            sport_client_->Move(req, 0.0f, 0.0f, 0.0f);
            return;
        }

        // Mapeo y saturación de Twist -> Move
        const float vx = std::clamp(static_cast<float>(current_twist_.linear.x),
                                    static_cast<float>(min_vx_), static_cast<float>(max_vx_));
        const float vy = std::clamp(static_cast<float>(current_twist_.linear.y),
                                    static_cast<float>(-max_vy_), static_cast<float>(max_vy_));
        const float vyaw = std::clamp(static_cast<float>(current_twist_.angular.z),
                                      static_cast<float>(-max_vyaw_), static_cast<float>(max_vyaw_));

        sport_client_->Move(req, vx, vy, vyaw);
    }

    // --- ROS interfaces ---
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::shared_ptr<SportClient> sport_client_;

    // --- Estado ---
    geometry_msgs::msg::Twist current_twist_;
    rclcpp::Time last_cmd_time_;
    int robot_mode_ = 0;  // estado desconocido al arranque

    // --- Parámetros ---
    std::string cmd_vel_topic_;
    std::string state_topic_;
    double control_frequency_;
    double cmd_timeout_;
    std::vector<int64_t> valid_modes_;
    double max_vx_, min_vx_, max_vy_, max_vyaw_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavToUnitreeBridge>();

    // MultiThreadedExecutor: hay varios suscriptores y un timer de alta
    // frecuencia que conviene no serializar tras un callback lento.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
