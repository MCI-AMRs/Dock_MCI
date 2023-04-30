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

#define DEBUG
// #define NOROB // uncomment to not use the move commands
#define PI 3.1415
#define OAK_OFFS 0 // 0.17 exact dist oak_bumper would be 0.232 but turtle should drive underneath
#define MARKER_LENGTH 0.091
#define MARKER_ID 20

using ImageComp = sensor_msgs::msg::CompressedImage;
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;

ImageComp::SharedPtr image_global;

bool gotImage = false;


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
            "/oakd/rgb/image_raw/compressed",1,std::bind(&CamSubscriber::image_callback, this, std::placeholders::_1), options1);
            std::cout << "calib subs" << std::endl;
            calibration_subscriber_ = this->create_subscription<Info>(
            "/oakd/rgb/camera_info",1,std::bind(&CamSubscriber::calib_callback, this, std::placeholders::_1),options2);

        }
    
    private:
        cv_bridge::CvImagePtr cv_ptr_;
        Info::SharedPtr calibData;
        double angle_error;
        double z_error;
        double x_error;
        
        void image_callback(const ImageComp::SharedPtr msg)
        {
            if(!gotImage){
                image_global = msg;
                gotImage = true;
                run();
            }
        }

       void calib_callback(const Info::SharedPtr msg)
        {
            calibData = msg;
            RCLCPP_INFO(get_logger(), "Unsubscribing from Calibration");
            calibration_subscriber_.reset();
        }

        void run(){
          if(gotImage){
            try {
              cv_ptr_ = cv_bridge::toCvCopy(image_global,sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e) {
              RCLCPP_INFO(this->get_logger(),"cv_bridge exception: %s", e.what());
              return;
            }

            pose_estimation(cv_ptr_);

            cv::imshow("image_stream", cv_ptr_->image);
            cv::waitKey(1);

            gotImage = false;
          }
          else { // wait for Imagestream
              std::this_thread::sleep_for(std::chrono::milliseconds(200));
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

            // Cam Fake Turtlebot
            // float mtx[9] = {1024.147705078125, 0.0, 647.973876953125,0.0, 1024.147705078125, 363.7773132324219,0.0, 0.0, 1.0};
            // float dist[14] = {9.57563591003418, -92.45447540283203, 0.0016312601510435343, 0.0018333167536184192, 308.990478515625, 9.401731491088867, -91.41809844970703, 305.3674621582031, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            // Set coordinate system
            cv::Mat objPoints(4, 1, CV_32FC3);
            objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-MARKER_LENGTH/2.f, MARKER_LENGTH/2.f, 0);
            objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(MARKER_LENGTH/2.f, MARKER_LENGTH/2.f, 0);
            objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(MARKER_LENGTH/2.f, -MARKER_LENGTH/2.f, 0);
            objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-MARKER_LENGTH/2.f, -MARKER_LENGTH/2.f, 0);

            cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, mtx);
            cv::Mat distCoeffs = cv::Mat(1, 8, CV_64F, dist);

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
            cv::aruco::drawAxis(img->image, cameraMatrix, distCoeffs, rvecs.at(0), tvecs.at(0), 0.05);

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
            return -1;
            }
        }
        rclcpp::Subscription<ImageComp>::SharedPtr image_subscriber_;
        rclcpp::Subscription<Info>::SharedPtr calibration_subscriber_;
        rclcpp::CallbackGroup::SharedPtr callback_group1_;
        rclcpp::CallbackGroup::SharedPtr callback_group2_;
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

