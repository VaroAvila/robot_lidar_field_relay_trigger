#include <rclcpp/rclcpp.hpp>
#include "rclcpp/node_interfaces/node_parameters.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <neo_srvs2/srv/relay_board_set_relay.hpp>
#include <neo_msgs2/msg/relay_board_v2.hpp>
#include <cmath>
/*#include <yaml>*/


class LaserFieldRelayControl : public rclcpp::Node
{
public:
    LaserFieldRelayControl() : Node("laser_field_relay_control")
    {
        // Command velocity subscriber
        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 
            10, 
            std::bind(&LaserFieldRelayControl::cmd_vel_callback, this, std::placeholders::_1)
            );
        // Relay states subscriber (set of arrays and other info - we only want to use the relay arrays)
        relay_states_sub = create_subscription<neo_msgs2::msg::RelayBoardV2>(
            "/state",
            10,
            std::bind(&LaserFieldRelayControl::relay_states_callback, this, std::placeholders::_1)
        );
        // Client to execute the SetRelay service from the Relay Board
        client = create_client<neo_srvs2::srv::RelayBoardSetRelay>("/set_relay");

        declare_parameter("speed_threshold_1", 0.12);
        declare_parameter("speed_threshold_2", 0.3);
        declare_parameter("speed_threshold_3", 0.5);

        get_parameter("speed_threshold_1", m_speed_threshold_1);
        get_parameter("speed_threshold_2", m_speed_threshold_2);
        get_parameter("speed_threshold_3", m_speed_threshold_3);

    }

private:
    rclcpp::Client<neo_srvs2::srv::RelayBoardSetRelay>::SharedPtr client;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<neo_msgs2::msg::RelayBoardV2>::SharedPtr relay_states_sub;
    geometry_msgs::msg::Twist current_cmd_vel;
    std::array<bool,4> current_relay_states;

    // Declaration of the speed_threshold parameters stored on the .yaml config file:
    double m_speed_threshold_1; 
    double m_speed_threshold_2;
    double m_speed_threshold_3;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      current_cmd_vel = *msg;
      call_service_based_on_cmd_vel(current_cmd_vel);
    }


    void relay_states_callback(const neo_msgs2::msg::RelayBoardV2::SharedPtr msg) 
    {
        current_relay_states = msg->relay_states; std::cout<<"R2:"<<current_relay_states[2]<<" / "<<"R3:"<<current_relay_states[3]<<std::endl;
    }

    void call_service_based_on_cmd_vel(geometry_msgs::msg::Twist m_current_cmd_speed) {
        // Extract cmd_vel values
        double linear_x = m_current_cmd_speed.linear.x;
        double linear_y = m_current_cmd_speed.linear.y;
        double angular_z = m_current_cmd_speed.angular.z;

        // Calculate the absolute linear speed
        double linear_speed_abs = std::sqrt(linear_x * linear_x + linear_y * linear_y);

        // Find the higher speed between linear and angular
        double max_speed = std::max(linear_speed_abs, std::abs(angular_z));

        // Define the threshold values for different speed ranges
        // double m_speed_threshold_1 = 0.12; 
        // double m_speed_threshold_2 = 0.3;
        // double m_speed_threshold_3 = 0.5;


        // Determine the desired relay states based on speed
        bool relay2_state = current_relay_states[2];
        bool relay3_state = current_relay_states[3];

        // Freerun (0.1)
        if (max_speed < m_speed_threshold_1) {
            relay2_state = false;
            relay3_state = true;
        } 
        // Docking (0.15-2)
        else if (max_speed < m_speed_threshold_2) {
            relay2_state = true;
            relay3_state = true;
        } 
        // Stand (0.4)
        else if (max_speed < m_speed_threshold_3) {
            relay2_state = false;
            relay3_state = false;
        } 
        // Max velocity (0.8)
        else if (max_speed >= m_speed_threshold_3) {
          relay2_state = true;
          relay3_state = false;
        }

        // Call the service only if the relay states have changed
        if (current_relay_states[2] != relay2_state) {
            call_service_relay(2, relay2_state);
        }

        if (current_relay_states[3] != relay3_state) {
            call_service_relay(3, relay3_state);
        }
    }

    void call_service_relay(int id, bool state) 
    {
        auto request = std::make_shared<neo_srvs2::srv::RelayBoardSetRelay::Request>();
        request->id = id;
        request->state = state;

        auto future = client->async_send_request(request);
        rclcpp::spin_until_future_complete(shared_from_this(), future);

    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto response = future.get();
            RCLCPP_INFO(get_logger(), "Service response: %d", response->success);
        } else {
            RCLCPP_ERROR(get_logger(), "Service call failed.");
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserFieldRelayControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


