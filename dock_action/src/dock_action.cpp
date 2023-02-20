#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "action_interfaces/action/dock.hpp"

#include "irobot_create_msgs/action/drive_distance.hpp"
#include "irobot_create_msgs/action/rotate_angle.hpp"
#include "sensor_msgs/msg/image.hpp"


// irobot_create_msgs::action:DriveDistance;

namespace dock_action_cpp
{

class DockActionServer : public rclcpp::Node
{
public:
  using Dock = action_interfaces::action::Dock;
  using GoalHandleDock = rclcpp_action::ServerGoalHandle<Dock>;

  explicit DockActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("dock_turtle_action_server",options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Dock>(
      this,
      "dock_turtle",
      std::bind(&DockActionServer::handle_goal, this,_1,_2),
      std::bind(&DockActionServer::handle_cancel, this,_1),
      std::bind(&DockActionServer::handle_accepted, this,_1));

    // Subscribe to image topic
    image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
      "/color/preview/image",
      1,
      std::bind(&DockActionServer::image_callback, this, _1));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Process the image
    // ...
  }


  rclcpp_action::Server<Dock>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Dock::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleDock> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDock> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DockActionServer::execute, this, _1), goal_handle}.detach();
  }

void execute(const std::shared_ptr<GoalHandleDock> goal_handle)
  {
    auto result = std::make_shared<Dock::Result>();




    // Check if goal is done
    if (rclcpp::ok()) {
      result->finished = 1;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class DockActionServer
}

RCLCPP_COMPONENTS_REGISTER_NODE(dock_action_cpp::DockActionServer)