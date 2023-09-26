#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher"); //inizializza il nodo "image_publisher"
  ros::NodeHandle nh; //crea oggetto nodehandle per comunicare con il sistema ros
  image_transport::ImageTransport it(nh); //crea un oggetto ImageTransport associato al nodehadle

  //crea un publisher per il topic "image_raw" con dimensione buffer 1
  image_transport::Publisher pub = it.advertise("image_raw", 1); 

  cv::VideoCapture cap(0); //apre la prima camera disponibile
  // Controlla se la webcam può essere aperta
  //resituisce true se la camera è aperta correttamenet altrimenti false
  if (!cap.isOpened())
  {
    ROS_ERROR("Failed to open camera!");
    return 1; 
  }

  // Imposta la risoluzione della webcam
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  cv::Mat frame; //oggetto che serve a memorizzare il frame acquisito dalla camera
  sensor_msgs::ImagePtr msg; //puntatore a oggetto di tipo Image per il messaggio da pubblicare

  ros::Rate loop_rate(60); // Frequenza di pubblicazione desiderata
  while (nh.ok())
  {
    cap >> frame; //acquisisce nuovo frame dalla camera
    // Controlla se il frame acquisito è valido
    if (!frame.empty())
    {
      //converte il frame in un messaggio di tipo Image usando cv_bridge
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg); //pubblica il messaggio
    }

    ros::spinOnce();
    loop_rate.sleep(); //attendi per mantenere il rate desiderato
  }

  return 0;
}
