#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// using gstreamer_pipeline to send video
std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
  return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
         std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
         "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
         std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char **argv)
{
  //define values for the video
  // int capture_width = 1280;
  // int capture_height = 720;
  int capture_width = 1920;
  int capture_height = 1080;
  int display_width = 1280;
  int display_height = 720;
  int framerate = 60;
  int flip_method = 2;

  // define ros topic
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  // define gstreamer_pipeline
  std::string pipeline = gstreamer_pipeline(capture_width,
                                            capture_height,
                                            display_width,
                                            display_height,
                                            framerate,
                                            flip_method);

  std::cout << "Using pipeline: \n\t" << pipeline << "\n";

  //capture the video from the pipeline
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

  // Check if video device can be opened with the given index
  if (!cap.isOpened())
  {
    return (-1);
    cout << "Failed to open camera." << endl;
  }
  Mat frame;
  Mat img_gray;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(300);
  while (ros::ok())
  {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if (!frame.empty())
    {
      //Reduce the image so that only the floor remains
      // frame = frame(Range(275, 600), Range(0, 1280));

      //Make the image grayscale
      // cvtColor(frame, img_gray, COLOR_BGR2GRAY);

      //place the image on the topic
      //msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame).toImageMsg();
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }
    loop_rate.sleep();
  }
}