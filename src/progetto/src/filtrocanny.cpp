#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
  
/* cv::Mat è la struttura usata da opencv per rappresentare le immagini, è una matrice multidimensionale */

ros::Publisher processed_image_pub;

//definisco la funzione di callback che viene chiamata ogni volta che viene ricevuto un mex sul topic "image_raw"
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // Converti il messaggio dell'immagine in un'immagine OpenCV
    cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

    // Applica il filtro di Canny per rilevare i bordi
    cv::Mat edges; //variabile usata per memorizzare l'immagine con i bordi
    cv::Canny(image, edges, 50, 150); //applica l'algoritmo di rilevamente dei bordi di canny all'immagine

    // Visualizza l'immagine con i bordi rilevati
    cv::imshow("Processed Image", edges);
    cv::waitKey(1);

    // Converti l'immagine elaborata in un messaggio di tipo sensor_msgs/Image
    sensor_msgs::ImagePtr processed_image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, edges).toImageMsg();

    // Pubblica l'immagine elaborata sul topic "processed_image"
    processed_image_pub.publish(processed_image_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("CV_Bridge exception: %s", e.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_subscriber");
  ros::NodeHandle nh;

  // Crea il subscriber per il topic "image_raw"
  ros::Subscriber sub = nh.subscribe("/camera1/image_raw", 1, imageCallback);

  // Crea il publisher per il topic "processed_image"
  processed_image_pub = nh.advertise<sensor_msgs::Image>("processed_image", 1);

  ros::spin();

  return 0;
}
