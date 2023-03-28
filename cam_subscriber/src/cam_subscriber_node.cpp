#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <opencv2/opencv.hpp> 
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp> 
#include <cv_bridge/cv_bridge.h>


class ImageSubscriber : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;
  using ImageComp = sensor_msgs::msg::CompressedImage;
  using ImageInfo = sensor_msgs::msg::CameraInfo;

  public:
    ImageSubscriber() : Node("image_subscriber"){
      // Subscribe to image topic
      image_subscriber_ = this->create_subscription<ImageComp>(
      "/oakd/rgb/image_raw/compressed",1,std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));
      
      // Subscribe to image topic
      image_info_subscriber_ = this->create_subscription<ImageInfo>(
        "/oakd/rgb/camera_info",1,std::bind(&ImageSubscriber::info_callback, this, std::placeholders::_1));
    }


  private:
    void image_callback(const ImageComp::SharedPtr msg)
    {
      auto img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      cv::imshow("Compressed", img->image);
      cv::waitKey(1);
    }

    void info_callback(const ImageInfo::SharedPtr msg)
    {
      float mtx[9];
      float dist[8];


      for(int i = 0; i < 8; i++)
      {
        mtx[i] = msg->k[i];
        dist[i] = msg->d[i];
      }
      mtx[8] = msg->k[8];

      cv::Mat cameraMatrix(3,3,CV_32F,mtx);
      cv::Mat distCoeffs(1, 5, CV_32F, dist);

      std::cout << cameraMatrix << std::endl;
      std::cout << distCoeffs << std::endl;
    }

    rclcpp::Subscription<ImageInfo>::SharedPtr image_info_subscriber_;
    rclcpp::Subscription<ImageComp>::SharedPtr image_subscriber_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}