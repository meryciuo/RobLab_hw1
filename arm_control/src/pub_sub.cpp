#include <functional>
#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

class ArmController : public rclcpp::Node
{
public:
  ArmController() : Node("arm_controller_node")
  {
    subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&ArmController::jointStateCallback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/position_controller/commands", 10);
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&ArmController::publishCommand, this));
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    current_joint_positions_ = {msg->position[0], msg->position[1], msg->position[2], msg->position[3]};
    RCLCPP_INFO(this->get_logger(), "joint_positions: [%f, %f, %f, %f]",
                  current_joint_positions_[0], current_joint_positions_[1],
                  current_joint_positions_[2], current_joint_positions_[3]);
  }

  void publishCommand()
  {
    auto message = std_msgs::msg::Float64MultiArray();
    message.data = {1.0, 1.0, 1.0, 1.0}; 
    RCLCPP_INFO(this->get_logger(), "Publishing: [%f, %f, %f, %f]",
                message.data[0], message.data[1], message.data[2], message.data[3]);
    publisher_->publish(message);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<double> current_joint_positions_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmController>());
  rclcpp::shutdown();
  return 0;
}

