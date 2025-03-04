#include <utility>
#include "bumpgo/BumpGoNode.hpp"

#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace bumpgo
{

using namespace std::chrono_literals;
using std::placeholders::_1;

BumpGoNode::BumpGoNode()
: Node("bump_go"),
  state_(FORWARD)
{
  bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
    "/events/bumper", 10, std::bind(&BumpGoNode::bumper_callback, this, _1));

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = create_wall_timer(20ms, std::bind(&BumpGoNode::control_cycle, this));

  state_ts_ = now();

  last_bump_ = std::make_unique<kobuki_ros_interfaces::msg::BumperEvent>();
  last_bump_->state = kobuki_ros_interfaces::msg::BumperEvent::RELEASED;
}

void
BumpGoNode::bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg)
{
  last_bump_ = std::move(msg);
}

void
BumpGoNode::control_cycle()
{
  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case FORWARD:
      out_vel.linear.x = SPEED_LINEAR;

      if (check_forward_2_back()) {
        go_state(BACK);
      }
      break;
    case BACK:
      out_vel.linear.x = -SPEED_LINEAR;

      if (check_back_2_turn()) {
        go_state(TURN);
      }
      break;
    case TURN:
        // 0 izquierda, 1 centro, 2 derecha
        if (last_bump_->bumper == 2 || last_bump_->bumper == 1) {
            out_vel.angular.z = SPEED_ANGULAR * 3;
        } else {
            out_vel.angular.z = -SPEED_ANGULAR * 3;
        }

      if (check_turn_2_forward()) {
        go_state(FORWARD);
      }

      break;
  }

  vel_pub_->publish(out_vel);
}

void
BumpGoNode::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = now();
}

bool
BumpGoNode::check_forward_2_back()
{
  return last_bump_->state == kobuki_ros_interfaces::msg::BumperEvent::PRESSED;
}

bool
BumpGoNode::check_back_2_turn()
{
  // Going back for 2 seconds
  return (now() - state_ts_) > BACKING_TIME;
}

bool
BumpGoNode::check_turn_2_forward()
{
  // Turning for 2 seconds
  return (now() - state_ts_) > TURNING_TIME;
}

}  // namespace fsm_bumpgo