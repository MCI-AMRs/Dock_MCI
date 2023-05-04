#include <functional>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp> 
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp> 
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_interfaces/action/poseerr.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#define PI 3.1415
#define OAK_OFFS 0.17 // exact dist oak_bumper would be 0.232 but turtle should drive underneath
#define MARKER_LENGTH 0.092

bool gotImage = false;
sensor_msgs::msg::CompressedImage::SharedPtr image_;
int MarkerID;

class PoseEstimationServer : public rclcpp::Node
{
public:
  using PoseError = action_interfaces::action::Poseerr;
  using GoalHandlePoseError = rclcpp_action::ServerGoalHandle<PoseError>;
  using ImageComp = sensor_msgs::msg::CompressedImage;
  using Info = sensor_msgs::msg::CameraInfo;

  explicit PoseEstimationServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pose_estimator_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<PoseError>(
      this,
      "cam_pose_estimator",
      std::bind(&PoseEstimationServer::handle_goal, this, _1, _2),
      std::bind(&PoseEstimationServer::handle_cancel, this, _1),
      std::bind(&PoseEstimationServer::handle_accepted, this, _1));

        callback_group1_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group2_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options1;
        options1.callback_group = callback_group1_;
        rclcpp::SubscriptionOptions options2;
        options2.callback_group = callback_group2_;

      // Subscribe to image topic
      image_subscriber_ = this->create_subscription<ImageComp>(
        "/oakd/rgb/image_raw/compressed",1,std::bind(&PoseEstimationServer::image_callback, this, _1),options1);

      calibration_subscriber_ = this->create_subscription<Info>(
         "/oakd/rgb/camera_info",1,std::bind(&PoseEstimationServer::calib_callback, this, std::placeholders::_1),options2);
  }

private:
  rclcpp_action::Server<PoseError>::SharedPtr action_server_;
  double x_error, y_error, angle_error;
  cv_bridge::CvImagePtr cv_ptr_;
  Info::SharedPtr calibData;

  void image_callback(const ImageComp::SharedPtr msg)
  {
    if(!gotImage){
        image_ = msg;
        gotImage = true;
        std::cout << "image" << std::endl;
    }
  }

  void calib_callback(const Info::SharedPtr msg)
  {
    calibData = msg;
    RCLCPP_INFO(get_logger(), "Unsubscribing from Calibration");
    calibration_subscriber_.reset();
  }


  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PoseError::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal to get pose of MarkerID %d", goal->id);
    (void)uuid;
    MarkerID = goal->id;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePoseError> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePoseError> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&PoseEstimationServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePoseError> goal_handle)
  {
    std::cout << "execute " << std::endl;
    auto feedback = std::make_shared<PoseError::Feedback>();
    auto result = std::make_shared<PoseError::Result>();
    std::cout << "start while" << std::endl;
    while (true){
        if(gotImage){
            // Check if there is a cancel request
            std::cout << "got image" << std::endl;
            if (goal_handle->is_canceling()) {
                result->finished = 0;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            std::cout << "start execute" << std::endl;
            // Decode image as OpenCV image
            try {
                cv_ptr_ = cv_bridge::toCvCopy(image_,sensor_msgs::image_encodings::MONO8);
                std::cout << "converted image" << std::endl;
            }
            catch (cv_bridge::Exception& e) {
                RCLCPP_INFO(this->get_logger(),"cv_bridge exception: %s", e.what());
                return;
            }
            // Do the pose estimation
            int i = pose_estimation(cv_ptr_);

            cv::imshow("test",cv_ptr_->image);
            cv::waitKey(1);

            if(i == 0){
                feedback->error.angular.z = angle_error;
                feedback->error.linear.x = x_error;
                feedback->error.linear.y = y_error;                
            }
            else{
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }

            goal_handle->publish_feedback(feedback);
        }
        gotImage = false;
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->finished = 1;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
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

    //cv::Mat distCoeffs = cv::Mat(1, 5, CV_32F, dist);
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
            
        if(std::find(ids.begin(),ids.end(),MarkerID) != ids.end()) { // check if the expected marker can be seen
            for(int i = 0; i < nMarkers; i++) {  // check if the robot is infront of the expected marker to dock to the right machine
                if(ids.at(i) == MarkerID)
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

        std::cout << "angle error: " << (angle_error * 180 / PI) << "°" << std::endl;
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
        std::cout << "z error: " << x_error << std::endl;
        std::cout << "y error aruco: " << inv_tvec.at<double>(0,0) << std::endl;
        std::cout << "y error cam: " << tvecs.at(0)[0]<< std::endl;
        std::cout << "choosedn y_error: " << y_error << std::endl;
        
        img->image = imageCopy;
        return 0;
    }
    else {
        return -1;
        }
  }
    rclcpp::CallbackGroup::SharedPtr callback_group1_;
    rclcpp::CallbackGroup::SharedPtr callback_group2_;
    rclcpp::Subscription<ImageComp>::SharedPtr image_subscriber_;
    rclcpp::Subscription<Info>::SharedPtr calibration_subscriber_;
};  // class PoseEstimationServer

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<PoseEstimationServer> Node = std::make_shared<PoseEstimationServer>();

  rclcpp::Node::make_shared("PoseEstimator_Server_Node");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(Node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}