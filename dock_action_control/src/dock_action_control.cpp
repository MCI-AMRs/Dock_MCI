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

#define DEBUG
// #define NOROB // uncomment to not use the move commands
#define PI 3.1415
#define OAK_OFFS 0.17 // exact dist oak_bumper would be 0.232 but turtle should drive underneath
#define MARKER_LENGTH 0.092
#define MARKER_ID 20

using namespace std::placeholders;

bool gotImage = false;
sensor_msgs::msg::CompressedImage::SharedPtr image_global;
uint64_t last_time_;
std::string name = "";

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
  ////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////// callbacks ///////////////////////////////////////////

  void callback_turn_goal_response(const GoalHandleRotate::SharedPtr & goal_handle) {
    if(!goal_handle)
    {
        RCLCPP_INFO(get_logger(), "[nav2] Turn: goal rejected!");
        isNavigating = false;
    } else {
        RCLCPP_INFO(get_logger(), "[nav2] Turn: goal accepted!");
    }
  }

  void callback_turn_result(const GoalHandleRotate::WrappedResult & result) {
      switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(get_logger(), "[nav2] Turn Goal succeeded");
              isNavigating = false;
              break;
          case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(this->get_logger(), "[nav2] Turn Goal was aborted");
              isNavigating = false;
              return;
          case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_ERROR(this->get_logger(), "[nav2] Turn Goal was canceled");
              isNavigating = false;
              return;
          default:
              RCLCPP_ERROR(this->get_logger(), "[nav2] Turn Unknown result code");
              isNavigating = false;
              return;
      }
  }

  void callback_drivedist_goal_response(const GoalHandleDistance::SharedPtr & goal_handle){
      if(!goal_handle) {
          RCLCPP_INFO(get_logger(), "[nav2] DriveDist: goal rejected!");
          isNavigating = false;
      }
      else {
          RCLCPP_INFO(get_logger(), "[nav2] DriveDist: goal accepted!");
      }
  }

  void callback_drivedist_result(const GoalHandleDistance::WrappedResult & result) {
      switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(get_logger(), "[nav2] Goal succeeded");
              isNavigating = false;
              break;
          case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(this->get_logger(), "[nav2] Goal was aborted");
              isNavigating = false;
              return;
          case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_ERROR(this->get_logger(), "[nav2] Goal was canceled");
              isNavigating = false;
              return;
          default:
              RCLCPP_ERROR(this->get_logger(), "[nav2] Unknown result code");
              isNavigating = false;
              return;
      }
  }

private:
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
  double angle_error;
  double x_error;
  double y_error;
  cv_bridge::CvImagePtr cv_ptr_;
  Info::SharedPtr calibData;
  bool image_received_ = false;
  double angle_threshold = 0.035; // ^= 2°
  double y_threshold = 0.015; // 1 cm

  void image_callback(const ImageComp::SharedPtr msg)
  {
      if(!gotImage){
          image_global = msg;
          gotImage = true;
      }
  }

  void calib_callback(const Info::SharedPtr msg)
  {
      calibData = msg;
      RCLCPP_INFO(get_logger(), "Unsubscribing from Calibration");
      calibration_subscriber_.reset();
  }


  void send_goal(std::string angle_or_dist ,double speed, double rad_or_m ) {
    using namespace std::placeholders;
    if (angle_or_dist == "angle") {
      if (!this->rotate_angle_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
      }

      auto goal_msg = RotateAngle::Goal();
      goal_msg.max_rotation_speed = speed;
      goal_msg.angle = rad_or_m;

      isNavigating = true;
      // set callbacks
      auto send_goal_options_turn = rclcpp_action::Client<RotateAngle>::SendGoalOptions();
      send_goal_options_turn.goal_response_callback = std::bind(&DockActionServer::callback_turn_goal_response, this,_1);
      send_goal_options_turn.result_callback = std::bind(&DockActionServer::callback_turn_result, this,_1);

      // send goal
      this->rotate_angle_->async_send_goal(goal_msg, send_goal_options_turn);
      
      // wait for future to complete
      size_t counter = 0;
      while(isNavigating){
          if((counter % 25) == 0)
          RCLCPP_INFO(get_logger(), "[nav2] navigating...");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          counter++;
      }
      }
    else if (angle_or_dist == "dist"){
      if (!this->drive_distance_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
      }

      auto goal_msg = DriveDistance::Goal();
      goal_msg.max_translation_speed = speed;
      goal_msg.distance = rad_or_m;
      isNavigating = true;
      // set callbacks
      auto send_goal_options_dist = rclcpp_action::Client<DriveDistance>::SendGoalOptions();
      send_goal_options_dist.goal_response_callback = std::bind(&DockActionServer::callback_drivedist_goal_response, this,_1);
      send_goal_options_dist.result_callback = std::bind(&DockActionServer::callback_drivedist_result, this,_1);

      // send goal
      this->drive_distance_->async_send_goal(goal_msg, send_goal_options_dist);
            
      // wait for future to complete
      size_t counter = 0;
      while(isNavigating) {
          if((counter % 25) == 0)
          RCLCPP_INFO(get_logger(), "[nav2] navigating...");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          counter++;
      }
    }
  }

  //finding median of ungrouped data
  float median(std::vector<double> vec){
      // Sort the array 
      std::sort(vec.begin(), vec.end());

      if (vec.size() % 2 == 0) { // even number
          int n = vec.size()/2;
          return ((vec.at(n-1)+vec.at(n))*0.5);
      }
      else { // odd number
          int n = std::floor(vec.size()*0.5);
          return (vec.at(n));
      }
  }

  cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R){
    // https://learnopencv.com/rotation-matrix-to-euler-angles/
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
  }
  
  int pose_estimation(cv_bridge::CvImagePtr img){ 
      double mtx[9] = {calibData->k[0], calibData->k[1],calibData->k[2],calibData->k[3],calibData->k[4],calibData->k[5],calibData->k[6],calibData->k[7],calibData->k[8]};
      double dist[8] = {calibData->d[0], calibData->d[1],calibData->d[2],calibData->d[3],calibData->d[4],calibData->d[5],calibData->d[6],calibData->d[7]};

      cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, mtx);
      cv::Mat distCoeffs = cv::Mat(1, 8, CV_64F, dist);

      cv::Mat imageCopy;
      img->image.copyTo(imageCopy);

      cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
      
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
      params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX; // CORNER_REFINE_CONTOUR CORNER_REFINE_APRILTAG CORNER_REFINE_SUBPIX

      cv::aruco::detectMarkers(imageCopy,dictionary,corners,ids,params);
      

      // If at least one marker detected
      if (ids.size() > 0) {
          cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
          // Calculate pose for marker
          int nMarkers = corners.size();
          std::vector<cv::Vec3d> rvecs, tvecs;
              
          if(std::find(ids.begin(),ids.end(),MARKER_ID) != ids.end()) { // check if the expected marker can be seen
              for(int i = 0; i < nMarkers; i++) {  // check if the robot is infront of the expected marker to dock to the right machine
                  if(ids.at(i) == MARKER_ID)
                      cv::aruco::estimatePoseSingleMarkers(corners,MARKER_LENGTH,cameraMatrix,distCoeffs,rvecs,tvecs);
              } 
          }
          else {
              return -1;
          }
          // Draw axis for marker
          cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs.at(0), tvecs.at(0), 0.05);

          // new rodrigues to euler    
          cv::Mat cam_aruco_rot_mat, inv_tvec;
          cv::Vec3f rot_vec;
          cv::Mat tvecs_mat = (cv::Mat_<double>(3, 1) << tvecs.at(0)[0], tvecs.at(0)[1], tvecs.at(0)[2]);
          cv::Rodrigues(rvecs.at(0), cam_aruco_rot_mat); // convert rotation vector to rotation matrix

          rot_vec = rotationMatrixToEulerAngles(cam_aruco_rot_mat);
          angle_error = rot_vec[1];
          inv_tvec = cam_aruco_rot_mat.t()*tvecs_mat; // transposed and multiplied with the transl. vector - to get horizontal error in the aruco_cam coordinate system robotics_condensed p.19 

          //std::cout << "angle error: " << (angle_error * 180 / PI) << "°" << std::endl;

          if (abs(angle_error) > 0.08){ // angle error > ~5°
              y_error = inv_tvec.at<double>(0,0);
              x_error = (abs(inv_tvec.at<double>(0,2))-OAK_OFFS); //offset of camera
          }
          else {
              y_error = tvecs.at(0)[0];
              x_error = (abs(tvecs.at(0)[2])-OAK_OFFS); //offset of camera
          }

          // orientation of the robot towards the aruco
          //std::cout << "z error: " << x_error << std::endl;
          //std::cout << "y error aruco: " << inv_tvec.at<double>(0,0) << std::endl;
          //std::cout << "y error cam: " << tvecs.at(0)[0]<< std::endl;
          //std::cout << "choosedn y_error: " << y_error << std::endl;
          
          img->image = imageCopy;
          return 0;
      }
      else {
          return -1;
      }
  }

  void search_for_tag(int count){
    double angle = 0.1745; // ^10°

    if(count < 4)
      send_goal("angle",0.1,angle);
    else if (count == 4)
      send_goal("angle",0.1, -angle*4); // turn in opposite direction +1
    else
      send_goal("angle",0.1,-angle);
  }


//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////action response functions ///////////////////////////////////////
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const Dock::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %i",goal->id);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDock> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDock> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DockActionServer::execute, this, _1), goal_handle}.detach();
  }

  void rough_dock(double y_err, double phi_err){
    if(abs(y_err) > y_threshold) { // reduce horizontal error
      RCLCPP_INFO_STREAM(this->get_logger(),"Horizontal error correction " << y_err << "...!");
      // turn to reduce error
      // if horizontal error is positive turn into positive direction 
      int sign = std::signbit(y_err) ? 1 : -1; 

      send_goal("angle",0.05,(0.5 * PI - abs(phi_err)) * sign); 
      // reduce horizontal error
      send_goal("dist",0.05,abs(y_err));
      // turn back
      send_goal("angle",0.05,0.5 * PI * (-sign));
    } 
    else if (abs(phi_err) > angle_threshold) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Angle correction " << phi_err*180/PI << "° ...!");                  
      int sign = std::signbit(phi_err) ? -1 : 1;
      send_goal("angle",0.05,phi_err * (-sign)); // correct angle in the opposite direction
    }
  }
  
  void PIDdock(float x_err, float y_err, float phi_err){
    float guard = 0.5;
    Twist cmd_vel;
    if (y_err < 0.03){
        guard = 0.2;
    }

    std::cout << "median y: " << y_err << std::endl;
    std::cout << "median x: " << x_err << std::endl;
    std::cout << "median phi: " << phi_err*180/PI << std::endl;

    // Init PID's
    pid_y_error.initPid(0.005, 0.0001, 0.0,0.3, 0.01); // 0.31, 0.05 m/s max vel
    pid_angle.initPid(0.05, 0.001, 0.001, 0.4, 0,01); // 1.9, 0.4 rad/s max vel 

    uint64_t time = this->now().nanoseconds();

    if(std::round(abs(y_err) * 1000) / 1000 > y_threshold){ // round to two decimals
        cmd_vel.angular.z = pid_y_error.computeCommand(y_err, time - last_time_);
        std::cout << "reduce y_error" << std::endl;

        if (y_err > 0.0){
            cmd_vel.angular.z = cmd_vel.angular.z * -1.0;
        }
        // guard to prohibit too large angles do not lose marker
        if (std::round(abs(phi_err) * 100) / 100  > guard){
            cmd_vel.angular.z = (cmd_vel.angular.z < 0) ? 0.01 : -0.01;
            std::cout << "guard" << std::endl;
        }

        cmd_vel.linear.x = 0.001;
    }
    else{
        cmd_vel.angular.z = pid_angle.computeCommand(phi_err, time - last_time_);
        if (std::round(abs(phi_err) * 100) / 100 > 0.04){
            std::cout << phi_err << std::endl;
            // if phi_err + cmd_vel -
            cmd_vel.angular.z = phi_err < 0 ? -cmd_vel.angular.z : cmd_vel.angular.z;

            cmd_vel.linear.x = 0.0;
            std::cout << "reduce angle error" << std::endl;

        }
        else{
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = 0.001;
        }
    }
                    
    last_time_ = time;

    // guard to set max_velocity
    if(abs(cmd_vel.angular.z) > 0.04){
        cmd_vel.angular.z = (std::signbit(cmd_vel.angular.z) ? -1 : 1) * 0.04;
    }

    std::cout << "cmd_vel.linear.x: " << cmd_vel.linear.x << std::endl;
    std::cout << "cmd_vel.angular.z: " << cmd_vel.angular.z << std::endl;

    cmd_vel_publisher_->publish(cmd_vel);
  }


  void execute(const std::shared_ptr<GoalHandleDock> goal_handle) {
    bool goal_reached = false;
    auto result = std::make_shared<Dock::Result>();
    int countImage = 0;
    int ThresholdImage = 5;
    bool firstIter = true;
    bool firstPIDdock = true;
    std::vector<double> y_err_vec, x_err_vec, phi_err_vec;


    while (!goal_reached){ // as long as goal not reacheds
      if(gotImage){
        try {
          cv_ptr_ = cv_bridge::toCvCopy(image_global,sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e) {
          RCLCPP_INFO(this->get_logger(),"cv_bridge exception: %s", e.what());
          return;
        }
        // estimate the pose
        int marker = pose_estimation(cv_ptr_); 

        cv::imshow("image_stream", cv_ptr_->image);
        cv::waitKey(1);

        // start docking algorithm
        if (marker == 0){
          if (countImage < ThresholdImage) {               
            y_err_vec.push_back(y_error);
            x_err_vec.push_back(x_error);
            phi_err_vec.push_back(angle_error);
            countImage++;
          }
          else {
            countImage = 0; // reset counter
            float x_err = median(x_err_vec);
            float y_err = median(y_err_vec);
            float phi_err = median(phi_err_vec);

            std::cout << "x_err" << x_err << std::endl;
            std::cout << "y_err" << y_err << std::endl;
            std::cout << "phi_err" << phi_err << std::endl;
            // reset vectors
            y_err_vec.clear();
            x_err_vec.clear();
            phi_err_vec.clear();

            // do rough estimation in the first iteration
            if(firstIter){
              std::cout << "rough dock" << std::endl;
              #ifdef NOROB
              rough_dock(y_err, phi_err);
              #endif
              firstIter = false;
            }

          if (x_err < 0.15){
              // Check if goal is done
              goal_reached = true;
              if (rclcpp::ok()) {
                RCLCPP_INFO(this->get_logger(), "Goal succeeded! Turtle docked.");
                result->finished = 1;
                goal_handle->succeed(result);
              }
          }
          else {
            if(firstPIDdock){
              last_time_ = this->now().nanoseconds();
              firstPIDdock = false;
            }
              std::cout << "PIDdock" << std::endl;
              #ifdef NOROB
              PIDdock(x_err,y_err,phi_err);
              #endif
          }
          }
          gotImage = false; // wait for new image
        } // if marker 
        else {
          std::this_thread::sleep_for(std::chrono::milliseconds(200));   
          gotImage = false; // wait for new image      
        }
      } // if gotImage
      else { // wait for Imagestream
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    } 
  }
};  // class DockActionServerTestActionServer

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<DockActionServer> Node = std::make_shared<DockActionServer>();

  rclcpp::Node::make_shared("Dock_Action_Node");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(Node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
