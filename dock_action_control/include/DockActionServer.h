#pragma once

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
#include "irobot_create_msgs/action/drive_distance.hpp"
#include "irobot_create_msgs/action/rotate_angle.hpp"
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control_toolbox/pid.hpp"

using namespace std::placeholders;

std::string name = "/robot2";

class DockActionServer : public rclcpp::Node
{
public:
  using Dock = action_interfaces::action::Dock;
  using GoalHandleDock = rclcpp_action::ServerGoalHandle<Dock>;
  using RotateAngle = irobot_create_msgs::action::RotateAngle;
  using GoalHandleRotate = rclcpp_action::ClientGoalHandle<RotateAngle>;
  using DriveDistance = irobot_create_msgs::action::DriveDistance;
  using GoalHandleDistance = rclcpp_action::ClientGoalHandle<DriveDistance>;
  using NavigateToPosition = irobot_create_msgs::action::NavigateToPosition;
  using GoalHandlePosition = rclcpp_action::ClientGoalHandle<NavigateToPosition>;
  using Twist = geometry_msgs::msg::Twist;
  using ImageComp = sensor_msgs::msg::CompressedImage;
  using Info = sensor_msgs::msg::CameraInfo;

  explicit DockActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("dock_turtle_action_server",options) // Class Constructor
  {
    //std::string test = this->get_name();
    // create the action server
    std::string server_name = name + "/dock_turtle";
    std::cout << server_name << std::endl;
    this->action_server_ = rclcpp_action::create_server<Dock>(
      this,
      server_name,
      std::bind(&DockActionServer::handle_goal, this,_1,_2),
      std::bind(&DockActionServer::handle_cancel, this,_1),
      std::bind(&DockActionServer::handle_accepted, this,_1));
    
    // create rotate angle client
    std::string rotate_name = name + "/rotate_angle";
    this->rotate_angle_ = rclcpp_action::create_client<RotateAngle>(
      this,
      rotate_name
      );

    // create drive distance client
    std::string drive_name = name + "/drive_distance";
    this->drive_distance_ = rclcpp_action::create_client<DriveDistance>(
      this,
      drive_name
      );

    callback_group1_ = this->create_callback_group(
                        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group2_ = this->create_callback_group(
                        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group1_;
    rclcpp::SubscriptionOptions options2;
    options2.callback_group = callback_group2_;

    std::string cam_name = name + "/oakd/rgb/image_raw/compressed";
    image_subscriber_ = this->create_subscription<ImageComp>(
    cam_name,1,std::bind(&DockActionServer::image_callback, this, std::placeholders::_1), options1);

    std::string cam_i_name = name + "/oakd/rgb/camera_info";
    calibration_subscriber_ = this->create_subscription<Info>(
    cam_i_name,1,std::bind(&DockActionServer::calib_callback, this, std::placeholders::_1),options2);
    
    std::string cmd_name = name + "/cmd_vel";
    cmd_vel_publisher_ = this->create_publisher<Twist>(cmd_name,10);
  }

  bool isNavigating;

  struct Pose {
    float x_error;
    float y_error;
    float angle_error;
    bool image;
  };
  
  ////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////// callbacks ///////////////////////////////////////////

  void callback_turn_goal_response(const GoalHandleRotate::SharedPtr & goal_handle);
  void callback_turn_result(const GoalHandleRotate::WrappedResult & result);
  void callback_drivedist_goal_response(const GoalHandleDistance::SharedPtr & goal_handle);
  void callback_drivedist_result(const GoalHandleDistance::WrappedResult & result);

private:
  double angle_error;
  double x_error;
  double y_error;
  cv_bridge::CvImagePtr cv_ptr_;
  Info::SharedPtr calibData;
  bool image_received_ = false;
  double angle_threshold = 0.035; // ^= 2°
  double y_threshold = 0.015; // 1 cm
  bool gotImage = false;
  ImageComp::SharedPtr image_global;
  uint64_t last_time_;

  void image_callback(const ImageComp::SharedPtr msg);
  void calib_callback(const Info::SharedPtr msg);
  void send_goal(std::string angle_or_dist ,double speed, double rad_or_m );

  //finding median of ungrouped data
  float median(std::vector<double> vec);

  cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);
  
  int pose_estimation(cv_bridge::CvImagePtr img);

//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////action response functions ///////////////////////////////////////
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const Dock::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDock> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleDock> goal_handle);

  void rough_dock(double y_err, double phi_err);

  void fineDock(float travel_dist, float y_error, float phi_err);
  
  void PIDdock(float x_err, float y_err, float phi_err);

  void execute(const std::shared_ptr<GoalHandleDock> goal_handle);
 
  rclcpp_action::Server<Dock>::SharedPtr action_server_;
  rclcpp_action::Client<RotateAngle>::SharedPtr rotate_angle_;
  rclcpp_action::Client<DriveDistance>::SharedPtr drive_distance_;
  rclcpp_action::Client<NavigateToPosition>::SharedPtr navigate_to_position_;
  rclcpp::Subscription<ImageComp>::SharedPtr image_subscriber_;
  rclcpp::Subscription<Info>::SharedPtr calibration_subscriber_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_group1_;
  rclcpp::CallbackGroup::SharedPtr callback_group2_;
  control_toolbox::Pid pid_y_error;
  control_toolbox::Pid pid_angle;
};  // class DockActionServerTestActionServer
