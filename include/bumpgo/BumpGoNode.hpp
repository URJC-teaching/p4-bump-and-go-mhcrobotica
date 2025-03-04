#ifndef BUMPGO_BUMPGONODE_HPP_
#define BUMPGO_BUMPGONODE_HPP_

#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace bumpgo
{

using namespace std::chrono_literals;  // NOLINT

class BumpGoNode : public rclcpp::Node
{
public:
  BumpGoNode();

private:
  void bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg);
  void control_cycle();

  static const int FORWARD = 0;
  static const int BACK = 1;
  static const int TURN = 2;
  int state_;
  rclcpp::Time state_ts_;

  void go_state(int new_state);
  bool check_forward_2_back();
  bool check_back_2_turn();
  bool check_turn_2_forward();

  const rclcpp::Duration TURNING_TIME {2s};
  const rclcpp::Duration BACKING_TIME {2s};

  static constexpr float SPEED_LINEAR = 0.3f;
  static constexpr float SPEED_ANGULAR = 0.3f;

  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  kobuki_ros_interfaces::msg::BumperEvent::UniquePtr last_bump_;
};

}  // namespace bumpgo

#endif  // BUMPGO_BUMPGONODE_HPP_
