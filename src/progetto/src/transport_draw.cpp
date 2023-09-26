#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void drawContours(cv::Mat& image)
{
  cv::Mat grayImage;
  cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

  cv::Mat binaryImage;
  cv::threshold(grayImage, binaryImage, 100, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  cv::Scalar color(0, 255, 0);  // Colore dei contorni (verde)

  for (size_t i = 0; i < contours.size(); ++i)
  {
    cv::drawContours(image, contours, i, color, 2);
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    drawContours(image);

    cv::imshow("view", image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
