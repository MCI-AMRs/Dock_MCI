#include <functional>
#include <memory>
#include <thread>
#include <sstream>
#include <iostream>
#include <cmath>
#include <string>

#include <chrono> // time
#include <opencv2/opencv.hpp> 
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp> 
#include <cv_bridge/cv_bridge.h>

// ROS2 needed for Action
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// turtlebot4 needed includes
#include "action_interfaces/action/dock.hpp"
#include "action_interfaces/action/poseerr.hpp"
#include "irobot_create_msgs/action/drive_distance.hpp"
#include "irobot_create_msgs/action/rotate_angle.hpp"
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <control_toolbox/pid.hpp>

int MarkerID_;
geometry_msgs::msg::Twist pose_error_;

class CamDockActionServer : public rclcpp::Node
{
public:
  using Dock = action_interfaces::action::Dock;
  using GoalHandleDock = rclcpp_action::ServerGoalHandle<Dock>;
  using PoseError = action_interfaces::action::Poseerr;
  using GoalHandlePoseError = rclcpp_action::ClientGoalHandle<PoseError>;
  using Twist = geometry_msgs::msg::Twist;

  explicit CamDockActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("cam_dock_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Dock>(
      this,
      "cam_docking",
      std::bind(&CamDockActionServer::handle_goal, this, _1, _2),
      std::bind(&CamDockActionServer::handle_cancel, this, _1),
      std::bind(&CamDockActionServer::handle_accepted, this, _1));

    this->client_ptr_ = rclcpp_action::create_client<PoseError>(
      this,
      "cam_pose_error_estimator");

    cmd_vel_publisher_ = this->create_publisher<Twist>("/robot2/cmd_vel",10);
  }

  void send_goal()
  {
    using namespace std::placeholders;
    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = PoseError::Goal();
    goal_msg.id = MarkerID_;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<PoseError>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&CamDockActionServer::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&CamDockActionServer::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&CamDockActionServer::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Server<Dock>::SharedPtr action_server_;  
  rclcpp_action::Client<PoseError>::SharedPtr client_ptr_;
  uint64_t last_time_;

   rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Dock::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal to get pose of MarkerID %d", goal->id);
    (void)uuid;
    MarkerID_ = goal->id;
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
    std::thread{std::bind(&CamDockActionServer::execute, this, _1), goal_handle}.detach();
  }

  void goal_response_callback(const GoalHandlePoseError::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandlePoseError::SharedPtr,
    const std::shared_ptr<const PoseError::Feedback> feedback)
  {
    pose_error_ = feedback->error;
  }

  void result_callback(const GoalHandlePoseError::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    rclcpp::shutdown();
  }

  void execute(const std::shared_ptr<GoalHandleDock> goal_handle)
  {
    auto result = std::make_shared<Dock::Result>();
    Twist cmd_vel;
    pid_y_error.initPid(0.1, 0.001, 0.001,0.3, 0.01); // 0.31, 0.05 m/s max vel
    pid_angle.initPid(0.1, 0.001, 0.001, 0.4, 0,01); // 1.9, 0.4 rad/s max vel 

    if (pose_error_.linear.z < 0.2)
        return;

    uint64_t time = this->now().nanoseconds();

    if(abs(pose_error_.linear.y) > 0.01){
        cmd_vel.angular.z = pid_y_error.computeCommand(pose_error_.linear.y, time - last_time_);
        if (pose_error_.linear.y > 0.0){
            cmd_vel.angular.z = cmd_vel.angular.z * -1.0;
        }
        // guard to prohibit too large angles do not lose marker
        if (pose_error_.angular.z > 0.6){
            cmd_vel.angular.z = (std::signbit(cmd_vel.angular.z) ? -1 : 1) * 0.1;
        }
    }
    else{
        cmd_vel.angular.z = pid_angle.computeCommand(pose_error_.angular.z, time - last_time_);
        if (pose_error_.angular.z > 0){
            cmd_vel.angular.z = -cmd_vel.angular.z;
        }
    }

    cmd_vel.linear.x = 0.001;                 
    last_time_ = time;

    std::cout << "cmd_vel.linear.x: " << cmd_vel.linear.x << std::endl;
    std::cout << "cmd_vel.angular.z: " << cmd_vel.angular.z << std::endl;

    cmd_vel_publisher_->publish(cmd_vel);

    if(rclcpp::ok()){
        result->finished = true;
        goal_handle->succeed(result);
    }
  }
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
  control_toolbox::Pid pid_y_error;
  control_toolbox::Pid pid_angle;
};  // class CamDockActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<CamDockActionServer> action_server = std::make_shared<CamDockActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
