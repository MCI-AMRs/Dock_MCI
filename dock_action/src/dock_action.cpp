#include <functional>
#include <memory>
#include <thread>
#include <sstream>
#include <iostream>
#include <cmath>

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
#include "sensor_msgs/msg/image.hpp"

#define DEBUG
// #define NOROB // uncomment do not use the move commands
#define PI 3.1415

using namespace std::placeholders;
// irobot_create_msgs::action:DriveDistance;

// define global image variable
bool gotImage = false;
sensor_msgs::msg::Image::SharedPtr image_global;

class DockActionServer : public rclcpp::Node
{
public:
  using Dock = action_interfaces::action::Dock;
  using GoalHandleDock = rclcpp_action::ServerGoalHandle<Dock>;
  using RotateAngle = irobot_create_msgs::action::RotateAngle;
  using DriveDistance = irobot_create_msgs::action::DriveDistance;
  using NavigateToPosition = irobot_create_msgs::action::NavigateToPosition;

  explicit DockActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("dock_turtle_action_server",options) // Class Constructor
  {
    // create the action server
    this->action_server_ = rclcpp_action::create_server<Dock>(
      this,
      "dock_turtle",
      std::bind(&DockActionServer::handle_goal, this,_1,_2),
      std::bind(&DockActionServer::handle_cancel, this,_1),
      std::bind(&DockActionServer::handle_accepted, this,_1));
    
    // create rotate angle client
    this->rotate_angle_ = rclcpp_action::create_client<RotateAngle>(
      this,
      "rotate_angle"
      );

    // create drive distance client
    this->drive_distance_ = rclcpp_action::create_client<DriveDistance>(
      this,
      "drive_distance"
      );

    // create navigating to position client
    this->navigate_to_pose_ = rclcpp_action::create_client<NavigateToPosition>(
      this,
      "navigate_to_position"
      );
  }
  double angle_error;
  double vertical_error;
  double horizontal_error;
  double search_angle = 0.35; // ^= 20° search angle to find AR Tag
  int count = 1;
  bool turn = 1;
  double scale = 1;

  struct Quaternion_z
  {
    double w,z;
  };
  
  struct Plane_xy
  {
    double x,y;
  };
  
  void send_position(Quaternion_z q, Plane_xy pos)
  {
    if (!this->navigate_to_pose_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }
    auto goal_pos = NavigateToPosition::Goal();
    goal_pos.goal_pose.pose.position.x = pos.x;
    goal_pos.goal_pose.pose.position.y = pos.y;
    goal_pos.goal_pose.pose.orientation.w = q.w;
    goal_pos.goal_pose.pose.orientation.z = q.z;
    goal_pos.max_translation_speed = 0.2;
    goal_pos.max_rotation_speed = 0.1;

    RCLCPP_ERROR(this->get_logger(), "Sending position action to Robot!");
    this->navigate_to_pose_->async_send_goal(goal_pos);
  }

  void send_goal(std::string angle_or_dist ,double speed, double rad_or_m )
    {
    if (angle_or_dist == "angle")
    {
      if (!this->rotate_angle_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
      }

      auto goal_msg = RotateAngle::Goal();
      goal_msg.max_rotation_speed = speed;
      goal_msg.angle = rad_or_m;

      RCLCPP_INFO(this->get_logger(), "Sending goal");
      this->rotate_angle_->async_send_goal(goal_msg);
     }
    else if (angle_or_dist == "dist")
    {
      if (!this->drive_distance_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
      }

      auto goal_msg = DriveDistance::Goal();
      goal_msg.max_translation_speed = speed;
      goal_msg.distance = rad_or_m;

      RCLCPP_INFO(this->get_logger(), "Sending goal");
      this->drive_distance_->async_send_goal(goal_msg);
    }

  }

  int pose_estimation(cv_bridge::CvImagePtr img){
    //float mtx[9] = {1586.84790, 0.0, 664.805767,0.0, 1583.30397, 421.826172,0.0, 0.0, 1.0};
    //float dist[5] = {0.17910684, -0.81931715, 0.0033601, 0.00745101, 1.44824592};

    float mtx[9] = {1546.14811,0.0,547.135385,0.0,1545.10239,399.614371,0.0,0.0,1.0};
    float dist[5] = {0.149987357,-0.681876074,0.000866810576,-0.000552686996,0.934266444};

    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, mtx);
    cv::Mat distCoeffs = cv::Mat(1, 5, CV_32F, dist);
    cv::Mat imageCopy;
    img->image.copyTo(img->image);
    float markerLength = 0.1;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(img->image,dictionary,corners,ids);
    // If at least one marker detected
    if (ids.size() > 0) {
      cv::aruco::drawDetectedMarkers(img->image, corners, ids);
      // Calculate pose for marker
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners,markerLength,cameraMatrix,distCoeffs,rvecs,tvecs);
      // Draw axis for marker
      cv::aruco::drawAxis(img->image, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1);
      std::cout << "Translation vector: " << std::endl;
      for(int i=0;i<3;i++)
        std::cout << tvecs.at(0)[i] << std::endl;

      std::cout << "Rotation vector: " << std::endl;
      for(int i=0;i<3;i++)
        std::cout << rvecs.at(0)[i]*180/PI << "°" << std::endl;

      
      angle_error = abs(rvecs.at(0)[2]);
      horizontal_error = abs(tvecs.at(0)[0]);
      vertical_error = tvecs.at(0)[2]+0.05-0.232; //offset of camera
      return 0;
    }
    else
    {
      return -1;
    }
}

  void search_for_tag()
  {
    if(turn)
    {
      send_goal("angle",0.1,search_angle*scale);
      count += 1;
      scale += 0.25;
    }
    else
    {
      send_goal("angle",0.1,search_angle*-scale);
      count -= 1;
      scale += 0.25;
    }

    if (count == 3)
    {
      send_goal("angle",0.1,search_angle*(scale+1)); // return to start
      count = -1;
      turn = 0;
      scale = 1;
    }
    else if (count == -3)
    {
      send_goal("angle",0.1,search_angle*(-(scale+1))); // return to start
      count = 1;
      turn = 1;
      scale = 1;
    }
    else if (count == 1 || count == -1)
    {
      scale = 0.25; // set scale to 0.25 to make smaller steps
    }
  }

  Quaternion_z euler_to_quaternion(double angle_z)
  {
    Quaternion_z q;
    q.w = cos(angle_z*0.5);
    q.z = sin(angle_z*0.5);
    return q;
  }

private:
  rclcpp_action::Server<Dock>::SharedPtr action_server_;
  rclcpp_action::Client<RotateAngle>::SharedPtr rotate_angle_;
  rclcpp_action::Client<DriveDistance>::SharedPtr drive_distance_;
  rclcpp_action::Client<NavigateToPosition>::SharedPtr navigate_to_pose_;
  cv_bridge::CvImagePtr cv_ptr_;
  bool image_received_ = false;
  double angle_threshold = 0.08; // ^= 5°
  double horizontal_threshold = 0.005; // 5 mm

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const Dock::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %i",goal->goal);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDock> goal_handle)
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
    #ifdef DEBUG
      std::cout << "received goal" << std::endl;
    #endif
  }

void execute(const std::shared_ptr<GoalHandleDock> goal_handle)
  {
    bool goal_reached = false;
    auto result = std::make_shared<Dock::Result>();
    int marker = -1;

      while (!goal_reached){ // as long as goal not reached
        if(gotImage){
          try
          { 
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr_ = cv_bridge::toCvCopy(image_global,sensor_msgs::image_encodings::BGR8);
          }
          catch (cv_bridge::Exception& e)
          {
            RCLCPP_INFO(this->get_logger(),"cv_bridge exception: %s", e.what());
            return;
          }
            // estimate the pose
            marker = pose_estimation(cv_ptr_); 
            std::cout << "Angle error: " << angle_error*180/PI << " °" << std::endl;
            std::cout << "Translation error: " << vertical_error << " m" << std::endl;
            // Update GUI Window
            cv::imshow("image_stream", cv_ptr_->image);
            cv::waitKey(1);
            if(marker == 0){
               if(angle_error > angle_threshold || horizontal_error > horizontal_threshold)
               {
                // Drive 20 cm infront of ArUco Tag
                #ifdef NOROB
                Plane_xy pos;
                pos.y = vertical_error - 0.2;
                pos.x = horizontal_error;
                Quaternion_z q;
                q = euler_to_quaternion(angle_error);
                send_position(q,pos); // Drive to the pose!
                #endif
                RCLCPP_INFO(this->get_logger(), "Driving infront of the Dock!");
                
               }
               else
               {
                // Drive towards dock with reduced vel
                #ifdef NOROB
                send_goal("dist",0.1,vertical_error);
                #endif 
                RCLCPP_INFO(this->get_logger(), "Docking the BOT!");
                goal_reached = true;
               }
               
            }
            else
            {
              // turn around to find the AR Tag
              #ifdef NOROB
              search_for_tag();
              #endif
            }
        gotImage = false; // wait for new image
        }  
        else
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }
      
      // Check if goal is done
      if (rclcpp::ok()) {
        result->finished = 1;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
    }
};  // class DockActionServerTestActionServer

class ImageSubscriber : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;

  public:
    ImageSubscriber() : Node("image_subscriber")
    {
      // Subscribe to image topic
      image_subscriber_ = this->create_subscription<Image>(
        "/color/preview/image",1,std::bind(&ImageSubscriber::image_callback, this, _1));
    }

  private:
    void image_callback(const Image::SharedPtr msg)const
    {
      if(!gotImage){
        image_global = msg;
        gotImage = true;
      }
      std::cout << "now" << std::endl;
    }
    rclcpp::Subscription<Image>::SharedPtr image_subscriber_;
};





int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<DockActionServer> action_server = std::make_shared<DockActionServer>();
  std::shared_ptr<ImageSubscriber> image_subscriber = std::make_shared<ImageSubscriber>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(image_subscriber);

  executor.add_node(action_server);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}