#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

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
//RNG rng(12345);

ros::Publisher pub;
float objectDistance;
float distanceFactor;

void objectDistanceCallback(const std_msgs::Float32 &msg)
{
    // ROS_WARN_STREAM("distance: " << objectDistance << "m");
    objectDistance = msg.data;
    distanceFactor = objectDistance * 2;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    //ROS_WARN_STREAM("distance: " << objectDistance << "m");

    try
    {
        //Read the frame
        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::waitKey(30);

        //imshow("frame", frame);

        Mat img_gray;
        cvtColor(frame, img_gray, COLOR_BGR2GRAY);

        //blur the image for better edge detection
        Mat img_blur;
        GaussianBlur(img_gray, img_blur, Size(7, 7), 0);

        //imshow("img_blur", img_blur);

        //apply Canny edge detection
        Mat canny;
        //adjust threshold 1 and 2 that we only see the white line
        Canny(img_blur, canny, 80, 390, 3, false);

        //imshow("canny", canny);

        //Find contours on canny image
        findContours(canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

        vector<vector<Point>> contours_poly(contours.size());
        vector<Rect> boundRect(contours.size());
        vector<Point2f> centers(contours.size());
        vector<float> radius(contours.size());

        for (int i = 0; i < contours.size(); i++)
        {
            drawContours(frame, contours, i, Scalar(0, 0, 255), 2, 8, hierarchy, 0, Point());
            approxPolyDP(contours[i], contours_poly[i], 3, true);
            boundRect[i] = boundingRect(contours_poly[i]);
            minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
        }

        Mat drawing = Mat::zeros(canny.size(), CV_8UC3);
        for (size_t i = 0; i < contours.size(); i++)
        {
            //Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            Scalar color = Scalar(255, 0, 0);

            drawContours(drawing, contours_poly, (int)i, color);
            rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);

            if (boundRect[i].br().x - boundRect[i].tl().x > 30)
            {
                int height = (boundRect[i].br().x - boundRect[i].tl().x) * 0.279126214;
                int width = (boundRect[i].br().y - boundRect[i].tl().y) * 0.490322581;

                putText(frame, "X1: " + to_string(boundRect[i].tl().x) + " X2: " + to_string(boundRect[i].br().x),
                        Point(boundRect[i].tl().x, boundRect[i].br().y), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 255), 2, 2);

                putText(frame, "Y1: " + to_string(boundRect[i].tl().y) + " Y2: " + to_string(boundRect[i].br().y),
                        Point(boundRect[i].tl().x, boundRect[i].br().y + 50), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 0), 2, 2);

                putText(frame, "H" + to_string(height * distanceFactor).erase(6) + "mm", boundRect[i].tl(), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 255), 2, 2);
                putText(frame, "             W" + to_string(width * distanceFactor).erase(6) + "mm", boundRect[i].tl(), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 2, 2);
            }

            //cout << "start point: " << boundRect[i].tl() << " end point: " << boundRect[i].br() << endl;
            //cout << "height: " << height * objectDistance << " width: " << width * objectDistance << endl;
            //400px = 115cm
            //circle(drawing, centers[i], (int)radius[i], color, 2);
        }

        putText(frame, "Dis " + to_string(objectDistance).erase(5) + "m", Point(50, 670), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 140, 255), 3, 2);

        imshow("Drawing", drawing);
        imshow("Contours", frame);

        //show the result
        //imshow("Linde detection", frame);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    catch (Exception &e)
    {
        ROS_ERROR_STREAM(&e);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pilot_image_listener", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    //ros::Rate loop_rate(10);

    ros::Subscriber subObj = nh.subscribe("/object_distance", 1000, objectDistanceCallback);

    //vision
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

    ros::spin();
    destroyAllWindows();
    return 0;
}