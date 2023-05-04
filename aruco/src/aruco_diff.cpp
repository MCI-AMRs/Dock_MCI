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
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <control_toolbox/pid.hpp>

#define DEBUG
// #define NOROB // uncomment to not use the move commands
#define PI 3.1415
#define OAK_OFFS 0 // 0.17 exact dist oak_bumper would be 0.232 but turtle should drive underneath
#define MARKER_LENGTH 0.05
#define MARKER_ID 20

using ImageComp = sensor_msgs::msg::CompressedImage;
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using Twist = geometry_msgs::msg::Twist;

ImageComp::SharedPtr image_global;

bool gotImage = false;
int count = 0;
int count_border = 5;
std::vector<double> y_err_vec, x_err_vec, phi_err_vec;
uint64_t last_time_;

class CamSubscriber : public rclcpp::Node
{
    public:
        CamSubscriber()
        : Node("cam_subscriber")
        {
            callback_group1_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group2_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

            rclcpp::SubscriptionOptions options1;
            options1.callback_group = callback_group1_;
            rclcpp::SubscriptionOptions options2;
            options2.callback_group = callback_group2_;

            std::cout << "image subs" << std::endl;
            image_subscriber_ = this->create_subscription<ImageComp>(
            "/robot2/oakd/rgb/image_raw/compressed",1,std::bind(&CamSubscriber::image_callback, this, std::placeholders::_1), options1);
            std::cout << "calib subs" << std::endl;
            calibration_subscriber_ = this->create_subscription<Info>(
            "/robot2/oakd/rgb/camera_info",1,std::bind(&CamSubscriber::calib_callback, this, std::placeholders::_1),options2);

            cmd_vel_publisher_ = this->create_publisher<Twist>("/robot2/cmd_vel",10);

            // control_toolbox::Pid pid_y_error;
            // control_toolbox::Pid pid_angle;
        }
    
    private:
        cv_bridge::CvImagePtr cv_ptr_;
        Info::SharedPtr calibData;
        double angle_error;
        double x_error;
        double y_error;
        rclcpp::Time last_pose_time_;
        
        void image_callback(const ImageComp::SharedPtr msg)
        {
            if(!gotImage){
                last_pose_time_ = msg->header.stamp;
                image_global = msg;
                gotImage = true;
                last_time_ = this->now().nanoseconds();
                run();
            }
        }

       void calib_callback(const Info::SharedPtr msg)
        {
            calibData = msg;
            RCLCPP_INFO(get_logger(), "Unsubscribing from Calibration");
            calibration_subscriber_.reset();
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

        void run(){
          if(gotImage){
            try {
              cv_ptr_ = cv_bridge::toCvCopy(image_global,sensor_msgs::image_encodings::MONO8);
            }
            catch (cv_bridge::Exception& e) {
              RCLCPP_INFO(this->get_logger(),"cv_bridge exception: %s", e.what());
              return;
            }

            int i = pose_estimation(cv_ptr_);

            if (i != 0){
                gotImage = false;
                return;
            }
            cv::imshow("image_stream", cv_ptr_->image);
            cv::waitKey(1);

            if(count < count_border){
                y_err_vec.push_back(y_error);
                x_err_vec.push_back(x_error);
                phi_err_vec.push_back(angle_error);
                count++;
                gotImage = false;
            }

            if(count >= count_border){
                count = 0;
                float y_err = median(y_err_vec);
                float x_err = median(x_err_vec);
                float phi_err = median(phi_err_vec);
                y_err_vec.clear();
                x_err_vec.clear();
                phi_err_vec.clear();


            }
          }
          else {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
          }
        }

        void drive(float x_err, float y_err, float phi_err){
            float guard = 0.5;
            if (y_err < 0.03){
                guard = 0.2;
            }
            std::cout << guard << std::endl;

            std::cout << "median y: " << y_err << std::endl;
            std::cout << "median x: " << x_err << std::endl;
            std::cout << "median phi: " << phi_err*180/PI << std::endl;

            Twist cmd_vel;
            pid_y_error.initPid(0.005, 0.0001, 0.0,0.3, 0.01); // 0.31, 0.05 m/s max vel
            pid_angle.initPid(0.05, 0.001, 0.001, 0.4, 0,01); // 1.9, 0.4 rad/s max vel 

            uint64_t time = this->now().nanoseconds();

            if(std::round(abs(y_err) * 1000) / 1000 > 0.015){ // round to two decimals
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

            // Cam Fake Turtlebot
            // float mtx[9] = {1024.147705078125, 0.0, 647.973876953125,0.0, 1024.147705078125, 363.7773132324219,0.0, 0.0, 1.0};
            // float dist[14] = {9.57563591003418, -92.45447540283203, 0.0016312601510435343, 0.0018333167536184192, 308.990478515625, 9.401731491088867, -91.41809844970703, 305.3674621582031, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, mtx);
            cv::Mat distCoeffs = cv::Mat(1, 8, CV_64F, dist);

            //cv::Mat distCoeffs = cv::Mat(1, 5, CV_32F, dist);
            cv::Mat imageCopy;
            img->image.copyTo(imageCopy);

            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
            
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
                // y_error = tvecs.at(0)[0];

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
        rclcpp::Subscription<ImageComp>::SharedPtr image_subscriber_;
        rclcpp::Subscription<Info>::SharedPtr calibration_subscriber_;
        rclcpp::CallbackGroup::SharedPtr callback_group1_;
        rclcpp::CallbackGroup::SharedPtr callback_group2_;
        rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
        control_toolbox::Pid pid_y_error;
        control_toolbox::Pid pid_angle;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<CamSubscriber> Node = std::make_shared<CamSubscriber>();

  rclcpp::Node::make_shared("Aruco_Node");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(Node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}