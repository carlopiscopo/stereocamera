#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
  // Inizializza il nodo ROS
  ros::init(argc, argv, "image_publisher1");
  ros::NodeHandle nh;

  // Crea un oggetto image_transport per pubblicare le immagini
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera1/image_raw", 1);

  // Configura la telecamera ESP32-CAM utilizzando OpenCV
  cv::VideoCapture cap("rtsp://192.168.1.46:8554/mjpeg/1");
 // cv::VideoCapture cap("rtsp://172.20.10.4:8554/mjpeg/1");
  if (!cap.isOpened()) {
    ROS_ERROR("Impossibile connettersi allo stream RTSP");
    return -1;
  }

  ros::Rate rate(30); 
  
  while (nh.ok()) {
    cv::Mat frame;
    cap.read(frame);

    // Crea un oggetto di messaggio di tipo sensor_msgs::Image utilizzando cv_bridge
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    // Pubblica l'immagine sul topic ROS
    pub.publish(msg);

    // Attendi un intervallo di tempo
   // ros::Duration(0.1).sleep();
  }

  return 0;
}

