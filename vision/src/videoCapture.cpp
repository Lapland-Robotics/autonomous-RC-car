#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

// if capture read error
// sudo service nvargus-daemon restart

std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
  return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
         std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
         "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
         std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char **argv)
{
  int capture_width = 1280;
  int capture_height = 720;
  int display_width = 1280;
  int display_height = 720;
  int framerate = 60;
  int flip_method = 2;

  std::string pipeline = gstreamer_pipeline(capture_width,
                                            capture_height,
                                            display_width,
                                            display_height,
                                            framerate,
                                            flip_method);
  std::cout << "Using pipeline: \n\t" << pipeline << "\n";

  VideoCapture cap(pipeline, CAP_GSTREAMER);

  VideoWriter video("/home/sdv/catkin_ws/src/pilot/resources/trackRecording.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 15, Size(1280, 720));

  cv::namedWindow("CSI Camera", WINDOW_AUTOSIZE);
  Mat frame;

  cout << "Hit ESC to exit"
       << "\n";
  while (true)
  {
    if (!cap.read(frame))
    {
      cout << "Capture read error" << std::endl;
      break;
    }

    video.write(frame);
    imshow("CSI Camera", frame);
    int keycode = waitKey(10) & 0xff;
    if (keycode == 27)
      break;
  }

  // When everything done, release the video capture object
  cap.release();
  video.release();

  // Closes all the frames
  destroyAllWindows();
  return 0;
}