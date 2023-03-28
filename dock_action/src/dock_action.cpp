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
#include "sensor_msgs/msg/image.hpp"

#define DEBUG
#define NOROB // uncomment to not use the move commands
#define PI 3.1415
#define OAK_OFFS 0.20 // exact dist oak_bumper would be 0.232 but turtle should drive underneath
#define MARKER_LENGTH 0.092
#define MARKER_ID 20

using namespace std::placeholders;
// irobot_create_msgs::action:DriveDistance;
// define global image variable
bool gotImage = false;
sensor_msgs::msg::CompressedImage::SharedPtr image_global;
cv::Mat camP;
cv::Mat orientationP;

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

  explicit DockActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("dock_turtle_action_server",options) // Class Constructor
  {
    //std::string test = this->get_namespace();
    std::string name = "/robot3";
    std::cout << name << std::endl;
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
  }

  double angle_error;
  double z_error;
  double x_error;
  bool isNavigating;

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

    //finding mode of ungrouped data
  float mode(float arr[], int n){
      // Sort the array 
      std::sort(arr, arr + n);
      int n_center = 0;

      if (n % 2 == 0) { // even number
        n_center = n/2;
        return (arr[n_center-1]+arr[n_center]) * 0.5;
      }
      else {
        n_center = std::floor(n*0.5);
        return arr[n_center];
      }
  }

  int pose_estimation(cv_bridge::CvImagePtr img){  
    // Cam from Fake Turtlebot
    // float mtx[9] = {1024.147705078125, 0.0, 647.973876953125,0.0, 1024.147705078125, 363.7773132324219,0.0, 0.0, 1.0};
    // float dist[14] = {9.57563591003418, -92.45447540283203, 0.0016312601510435343, 0.0018333167536184192, 308.990478515625, 9.401731491088867, -91.41809844970703, 305.3674621582031, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Cam from Robot3
    float mtx[9] = {1028.708740234375, 0.0, 641.5645751953125, 0.0, 1028.708740234375, 362.7433776855469, 0.0, 0.0, 1.0};
    float dist[14] = {10.559211730957031, -81.07833862304688, -0.00018250872381031513, -0.00033414774225093424, 299.4360656738281, 10.359108924865723, -80.04523468017578, 294.8573913574219, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Cam from Robot2
    // float mtx[9] = {1029.14794921875, 0.0, 647.3345947265625, 0.0, 1029.14794921875, 373.95074462890625, 0.0, 0.0, 1.0};
    // float dist[14] = {12.819296836853027, -113.50406646728516, -2.672206210263539e-05, 9.265074368158821e-06, 401.9082336425781, 12.615724563598633, -112.17804718017578, 396.441162109375, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Cam from Robot1
    // float mtx[9] = {1025.4049072265625, 0.0, 643.5555419921875, 0.0, 1025.4049072265625, 371.5435791015625, 0.0, 0.0, 1.0};
    // float dist[14] = {18.74028778076172, -179.54446411132812, 0.002264645416289568, 0.0020573034416884184, 681.7216186523438, 18.51045799255371, -177.75823974609375, 673.6657104492188, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-MARKER_LENGTH/2.f, MARKER_LENGTH/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(MARKER_LENGTH/2.f, MARKER_LENGTH/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(MARKER_LENGTH/2.f, -MARKER_LENGTH/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-MARKER_LENGTH/2.f, -MARKER_LENGTH/2.f, 0);

    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, mtx);
    cv::Mat distCoeffs = cv::Mat(1, 14, CV_32F, dist);
    //cv::Mat distCoeffs = cv::Mat(1, 5, CV_32F, dist);
    cv::Mat imageCopy;
    img->image.copyTo(img->image);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    cv::aruco::detectMarkers(img->image,dictionary,corners,ids,params);

    // If at least one marker detected
    if (ids.size() > 0) {
      cv::aruco::drawDetectedMarkers(img->image, corners, ids);
      // Calculate pose for marker
      int nMarkers = corners.size();
      std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
        
      if(std::find(ids.begin(),ids.end(),20) != ids.end()) { // check if the expected marker can be seen
        for(int i = 0; i < nMarkers; i++) {  // check if the robot is infront of the expected marker to dock to the right machine
          if(ids.at(i) == 20){
            cv::solvePnP(objPoints,corners.at(i),cameraMatrix,distCoeffs,rvecs.at(0),tvecs.at(0));
          }
        }
      }
      else {
        return -1;
      }
      // Draw axis for marker
      cv::aruco::drawAxis(img->image, cameraMatrix, distCoeffs, rvecs.at(0), tvecs.at(0), 0.1);

      // new rodrigues to euler    
      cv::Mat cam_aruco_rot_mat, inv_tvec;
      cv::Vec3f rot_vec;
      cv::Mat tvecs_mat = (cv::Mat_<double>(3, 1) << tvecs.at(0)[0], tvecs.at(0)[1], tvecs.at(0)[2]);
      cv::Rodrigues(rvecs.at(0), cam_aruco_rot_mat); // convert rotation vector to rotation matrix
      
      rot_vec = rotationMatrixToEulerAngles(cam_aruco_rot_mat);
      float rot_y = rot_vec[1];
      inv_tvec = cam_aruco_rot_mat.t()*tvecs_mat; // transposed and multiplied with the transl. vector - to get horizontal error in the aruco_cam coordinate system robotics_condensed p.19 

      angle_error = rot_y;
      std::cout << "angle error: " << angle_error * 180 / PI << "°" << std::endl;
      // x_error = tvecs.at(0)[0];

      if (abs(angle_error) > 0.08){ // angle error > ~5°
        x_error = inv_tvec.at<double>(0,0);
        z_error = abs(inv_tvec.at<double>(0,2))-OAK_OFFS; //offset of camera
      }
      else {
        x_error = tvecs.at(0)[0];
        z_error = abs(tvecs.at(0)[2])-OAK_OFFS; //offset of camera
      }

      // orientation of the robot towards the aruco
      std::cout << "z error: " << z_error << std::endl;
      std::cout << "x error aruco: " << inv_tvec.at<double>(0,0) << std::endl;
      std::cout << "x error cam: " << tvecs.at(0)[0] << std::endl;

      return 0;
    }
    else {
      std::cout << "return -1" << std::endl;
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

  cv_bridge::CvImagePtr cv_ptr_;
  bool image_received_ = false;
  double angle_threshold = 0.035; // ^= 2°
  double x_threshold = 0.01; // 1 cm

//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////action response functions ///////////////////////////////////////
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const Dock::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %i",goal->goal);
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
    #ifdef DEBUG
      std::cout << "received goal" << std::endl;
    #endif
  }

  void execute(const std::shared_ptr<GoalHandleDock> goal_handle) {
      bool goal_reached = false;
      auto result = std::make_shared<Dock::Result>();
      int countImage = 0;
      int cam_threshold = 20;
      float angle[20] = {0.0};
      float verti[20] = {0.0};
      float horiz[20] = {0.0};
      int count_noTag = 0; // counter how many times the tag is not found


        while (!goal_reached){ // as long as goal not reached
          if(gotImage){
            try {
              cv_bridge::CvImagePtr cv_ptr;
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
                if (countImage < cam_threshold) {               
                  angle[countImage] = angle_error;
                  verti[countImage] = z_error;
                  horiz[countImage] = x_error;
                  countImage++;
                }
                else {
                count_noTag = 0; // reset counter no Tag
                countImage = 0;
                angle_error = mode(angle,cam_threshold); // calculate modus of the used images
                z_error = mode(verti,cam_threshold);
                x_error = mode(horiz,cam_threshold);

                  if(abs(x_error) > x_threshold) { // reduce horizontal error
                    RCLCPP_INFO_STREAM(this->get_logger(),"Horizontal error correction " << x_error << "...!");
                    #ifdef NOROB
                    // turn to reduce error
                    std::cout << "horizontal " << x_error << std::endl;
                    int sign = std::signbit(x_error) ? 1 : -1; // if horizontal error is positive turn into positive direction 
                    //int sign = std::signbit(angle_error) ? -1 : 1; // if angle error is positive turn into positive direction 
                    std::cout << "sign: " << sign << std::endl;
                    send_goal("angle",0.05,(0.5 * PI - abs(angle_error)) * sign); 
                    std::cout << "turn: " << (0.5 * PI - abs(angle_error)) * sign * 180 / PI<< "°" << std::endl;
                    // reduce horizontal error
                    send_goal("dist",0.05,abs(x_error));
                    std::cout << "straight: " << abs(x_error) << std::endl;
                    // turn back
                    send_goal("angle",0.05,0.5 * PI * (-sign));
                    std::cout << "turn back: " << 0.5 * PI * (-sign) * 180 / PI << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // sleep that the cam get time for a good image
                    if(z_error > 0.4)
                      send_goal("dist",0.05,0.2);
                    #endif
                  }
                  else {
                    if(abs(angle_error) > angle_threshold) {
                    //1 
                      RCLCPP_INFO_STREAM(this->get_logger(), "Angle correction " << angle_error*180/PI << "° ...!");                  
                      #ifdef NOROB
                      int sign = std::signbit(angle_error) ? -1 : 1;
                      send_goal("angle",0.05,angle_error * -sign); // correct angle in the opposite direction
                      std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // sleep that the cam get time for a good image
                      #endif
                    }
                    else {
                      RCLCPP_INFO_STREAM(this->get_logger(), "Docking the BOT! Driving: " << z_error << " m");
                      #ifdef NOROB
                      if(z_error < 0.05) {
                        send_goal("dist",0.02,(z_error));
                      }
                      else {
                        send_goal("dist",0.05,(z_error*0.9));
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // sleep that the cam get time for a good image
                      }
                      #endif 
                      goal_reached = true;
                    }
                  }
                }
              } // if marker == 0
              else { // find marker
                // turn around to find the AR Tag
                RCLCPP_INFO(this->get_logger(), "Turn turtle to get Aruco!");
                #ifdef NOROB
                  search_for_tag(count_noTag);
                #endif
                count_noTag++;
              }
          gotImage = false; // wait for new image
          } // if gotImage

          else { // wait for Imagestream
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
          }
        }
        
        // Check if goal is done
        if (rclcpp::ok()) {
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
          result->finished = 1;
          goal_handle->succeed(result);
        }
      }
};  // class DockActionServerTestActionServer

class ImageSubscriber : public rclcpp::Node
{
  using ImageComp = sensor_msgs::msg::CompressedImage;

  public:
    ImageSubscriber() : Node("image_subscriber")
    {
      //std::string name = this->get_namespace();
      std::string name = "/robot3";
      std::string image_topic = name + "/oakd/rgb/image_raw/compressed";
      // Subscribe to image topic
      image_subscriber_ = this->create_subscription<ImageComp>(
        image_topic,1,std::bind(&ImageSubscriber::image_callback, this, _1));
    }

  private:
    void image_callback(const ImageComp::SharedPtr msg)const {
      if(!gotImage){
        image_global = msg;
        gotImage = true;
      }
      //std::cout << "now" << std::endl;
    }
    rclcpp::Subscription<ImageComp>::SharedPtr image_subscriber_;
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