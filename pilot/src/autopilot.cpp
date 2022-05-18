#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <driving_controls_msg_cpp/driving_controls.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <termios.h>
#include <iostream>
#include <stdlib.h>
#include <sstream>

#define STDIN_FILENO 0

using namespace std;
using namespace cv;

vector<vector<Point>> contours;
vector<Vec4i> hierarchy;

Point leftBottomMin = Point(0, 200);
Point leftBottomMax = Point(160, 400);
Point leftMiddleMin = Point(600, 100);
Point leftMiddleMax = Point(700, 300);
Point leftTopMin = Point(250, 200);
Point leftTopMax = Point(300, 300);

Point rightMiddleMin = Point(450, 70);
Point rightMiddleMax = Point(550, 240);
Point rightBottomMin = Point(950, 130);
Point rightBottomMax = Point(1300, 400);

int leftBottomCounter;
int leftMiddleCounter;
int leftTopCounter;

int rightBottomCounter;
int rightMiddleCounter;

int vehicleSpeed = 14;
int cornerSpeed = 13;
int previousSpeed;

Mat camera;
ros::Publisher pub;
driving_controls_msg_cpp::driving_controls controlMsg;

uint remap(int numIn, int InMin, int InMax, uint outMin, uint outMax)
{
    return (numIn - InMin) * (outMax - outMin) / (InMax - InMin) + outMin;
}

void setSpeed(int speed)
{
    if (previousSpeed != speed)
    {
        controlMsg.speed = remap(speed, -100, 100, 200, 400);
    }
    pub.publish(controlMsg);
    previousSpeed = speed;
}

bool comparePoints(int contourPointX, int contourPointY, int minX, int minY, int maxX, int maxY)
{
    if (contourPointX >= minX && contourPointY >= minY && contourPointX <= maxX && contourPointY <= maxY)
        return true;
    else
        return false;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        //Read the frame
        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::waitKey(30);

        //rescale so only the floor is visible
        //frame = frame(Range(290, 600), Range(0, 1280));

        //convert to gray scale image
        //Mat img_gray;
        //cvtColor(frame, img_gray, COLOR_BGR2GRAY);

        //blur the image for better edge detection
        Mat img_blur;
        // GaussianBlur(frame, img_blur, Size(3, 3), 0);
        GaussianBlur(frame, img_blur, Size(7, 7), 0);

        //apply Canny edge detection
        Mat canny;
        //adjust threshold 1 and 2 that we only see the white line
        //Canny(img_blur, canny, 50, 570, 3, false);
        Canny(img_blur, canny, 30, 300, 3, false);

        // Find contours on canny image
        findContours(canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        vector<vector<Point>> contours_poly(contours.size());
        vector<Rect> boundRect(contours.size());
        vector<Point2f> centers(contours.size());
        vector<float> radius(contours.size());

        for (int i = 0; i < contours.size(); i++)
        {
            for (auto &j : contours[i])
            {
                //check that the line is in the designated area

                //left bottom rectangle
                if (comparePoints(j.x, j.y, leftBottomMin.x, leftBottomMin.y, leftBottomMax.x, leftBottomMax.y))
                {
                    //putText(frame, "leftBottom", Point(30, 100), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 255), 3, 2);
                    leftBottomCounter++;
                }

                //left top rectangle
                if (comparePoints(j.x, j.y, leftTopMin.x, leftTopMin.y, leftTopMax.x, leftTopMax.y))
                {
                    //putText(frame, "leftTop", Point(50, 200), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 255), 3, 2);
                    leftTopCounter++;
                }

                //left middle rectangle
                if (comparePoints(j.x, j.y, leftMiddleMin.x, leftMiddleMin.y, leftMiddleMax.x, leftMiddleMax.y))
                {
                    //putText(frame, "leftMiddle", Point(700, 100), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 255), 3, 2);
                    leftMiddleCounter++;
                }

                //right bottom rectangle
                if (comparePoints(j.x, j.y, rightBottomMin.x, rightBottomMin.y, rightBottomMax.x, rightBottomMax.y))
                {
                    //putText(frame, "rightBottom", Point(900, 200), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 0, 255), 3, 2);
                    rightBottomCounter++;
                }

                //right middle rectangle
                if (comparePoints(j.x, j.y, rightMiddleMin.x, rightMiddleMin.y, rightMiddleMax.x, rightMiddleMax.y))
                {
                    putText(frame, "rightMiddle", Point(200, 50), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 0, 255), 3, 2);
                    rightMiddleCounter++;
                }

                //draw the found contours on the screen
                // drawContours(frame, contours, i, Scalar(0, 0, 255), 2, 8, hierarchy, 0, Point());
                // approxPolyDP(contours[i], contours_poly[i], 3, true);
                // boundRect[i] = boundingRect(contours_poly[i]);

                //draw the areas where the line should be in
            }
        }

        int leftAmount = leftBottomCounter + leftMiddleCounter + leftTopCounter;
        int rightAmount = rightBottomCounter + rightMiddleCounter;

        //left bottom green
        rectangle(frame, leftBottomMin, leftBottomMax, Scalar(0, 255, 0), 2);
        //left top green
        rectangle(frame, leftTopMin, leftTopMax, Scalar(0, 255, 0), 2);
        //left middle green
        rectangle(frame, leftMiddleMin, leftMiddleMax, Scalar(0, 255, 0), 2);
        //left blue
        rectangle(frame, rightMiddleMin, rightMiddleMax, Scalar(255, 0, 0), 2);
        //right blue
        rectangle(frame, rightBottomMin, rightBottomMax, Scalar(255, 0, 0), 2);

        //if more than 20 points have been seen in both left areas, assume that the line goes to the right
        if (leftBottomCounter >= 15 && leftMiddleCounter >= 15 || leftTopCounter >= 15 && leftAmount > rightAmount)
        {
            controlMsg.steering = remap(-25, -25, 25, 265, 376);
            setSpeed(cornerSpeed);
            //putText(frame, "RIGHT", Point(550, 50), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 100, 255), 3, 2);
        }
        //if more than 20 points have been seen in both right areas, assume that the line goes to the left
        else if (rightBottomCounter >= 15 && rightMiddleCounter >= 15 && rightAmount + leftTopCounter > leftAmount)
        {
            controlMsg.steering = remap(25, -25, 25, 265, 376);
            setSpeed(cornerSpeed);
            //putText(frame, "LEFT", Point(550, 50), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 100, 255), 3, 2);
        }
        else
        {
            controlMsg.steering = remap(0, -25, 25, 265, 376);
            setSpeed(vehicleSpeed);
            //putText(frame, "STRAIGHT", Point(550, 50), FONT_HERSHEY_SIMPLEX, 2, Scalar(100, 100, 255), 3, 2);
        }

        //ROS_WARN_STREAM("autopilot " << controlMsg);

        pub.publish(controlMsg);

        leftBottomCounter = 0;
        leftTopCounter = 0;
        leftMiddleCounter = 0;
        rightBottomCounter = 0;
        rightMiddleCounter = 0;

        //show the result
        imshow("Linde detection", frame);

        //wait one millisecond between successive frames and break the loop if key q is pressed
        if (waitKey(10) == 'q')
        {
            cout << "Stopping line detection" << endl;
            //vid_capture.release();
            destroyAllWindows();
            controlMsg.speed = remap(0, -100, 100, 200, 400);
            ROS_WARN_STREAM("steering " << controlMsg);

            pub.publish(controlMsg);

            exit(0);
            //break;
        }
        if (waitKey(10) == 'p')
        {
            cout << "Pausing the video" << endl;
            while (waitKey(10) != 'p')
            {
            }
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pilot_image_listener", ros::init_options::NoSigintHandler);
    ros::init(argc, argv, "autopilot_driving_control", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    //driving controls
    pub = nh.advertise<driving_controls_msg_cpp::driving_controls>("driving_controls_msg_cpp", 10);
    ros::Rate loop_rate(10);
    //driving_controls_msg_cpp::driving_controls controlMsg;

    controlMsg.speed = remap(vehicleSpeed, -100, 100, 200, 400);
    pub.publish(controlMsg);

    //vision
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

    ros::spin();
    destroyAllWindows();
    return 0;
}