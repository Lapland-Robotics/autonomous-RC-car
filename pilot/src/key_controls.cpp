#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <driving_controls_msg_cpp/driving_controls.h>
#include <stdlib.h>
#include <sstream>

#include <termios.h>
#include <iostream>
#define STDIN_FILENO 0

using namespace std;
int speed = 0;
int steering = 0;
int setSpeed = 0;
int setSteering = 0;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

uint remap(int numIn, int InMin, int InMax, uint outMin, uint outMax)
{
  return (numIn - InMin) * (outMax - outMin) / (InMax - InMin) + outMin;
}

int main(int argc, char **argv)
{
  // driving controls
  ros::init(argc, argv, "pilot");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<driving_controls_msg_cpp::driving_controls>("driving_controls_msg_cpp", 10);
  ros::Rate loop_rate(10);

  // Black magic to prevent Linux from buffering keystrokes.
  struct termios t;
  tcgetattr(STDIN_FILENO, &t);
  t.c_lflag &= ~ICANON;
  tcsetattr(STDIN_FILENO, TCSANOW, &t);

  // Once the buffering is turned off, the rest is simple.
  // Using 3 char type, Cause up down right left consist with 3 character
  cout << "Move: ";
  char c, d, e;
  {
    cin >> c;
    cin >> d;
    cin >> e;

    driving_controls_msg_cpp::driving_controls msg;
    if ((c == 27) && (d = 91))
    {
      if (e == 65)
      {
        //up
        speed += 1;
        setSpeed = remap(speed, -100, 100, 200, 400);
        msg.speed = setSpeed;
        msg.steering = setSteering;
        cout << " speed: " << speed << endl;
      }
      if (e == 66)
      {
        //down
        speed -= 1;
        setSpeed = remap(speed, -100, 100, 200, 400);
        msg.speed = setSpeed;
        msg.steering = setSteering;
        cout << " speed: " << speed << endl;
      }
      if (e == 67)
      {
        //right
        steering -= 1;
        setSteering = remap(steering, -25, 25, 265, 376);
        msg.steering = setSteering;
        msg.speed = setSpeed;
        cout << " steering: " << steering << endl;
      }
      if (e == 68)
      {
        //left
        steering += 1;
        setSteering = remap(steering, -25, 25, 265, 376);
        msg.steering = setSteering;
        msg.speed = setSpeed;
        cout << " steering: " << steering << endl;
      }

      pub.publish(msg);
    }
  }
  return 0;
}