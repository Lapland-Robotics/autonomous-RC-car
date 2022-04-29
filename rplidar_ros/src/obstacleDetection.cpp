/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <driving_controls_msg_cpp/driving_controls.h>

#include <stdlib.h>
#include <iostream>
#include <math.h>

#define RAD2DEG(x) ((x)*180. / M_PI)

using namespace std;

ros::Publisher pub;
driving_controls_msg_cpp::driving_controls msg;

//in ranges value 810 = -30°
//in ranges value 1136 = 30°

uint remap(int numIn, int InMin, int InMax, uint outMin, uint outMax)
{
    return (numIn - InMin) * (outMax - outMin) / (InMax - InMin) + outMin;
}

void frontDetection(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    int count = scan->scan_time / scan->time_increment;
    //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    for (int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);

        if (degree < 0 && degree > -30 || degree > 0 && degree < 30)
        {
            if (scan->ranges[i] < 1.01 /* && !isinf(scan->ranges[i]) */)
            {
                //set speed and steering to 0 to stop the car
                msg.speed = remap(0, -100, 100, 200, 400);
                msg.steering = remap(0, -25, 25, 265, 376);
                pub.publish(msg);
                ROS_INFO_STREAM("Obstacle in front of the car (distance: " << scan->ranges[i] << "m) \n");
                ROS_INFO_STREAM("msg:" << msg);
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::init(argc, argv, "obstacle_detection_driving_control");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, frontDetection);
    pub = n.advertise<driving_controls_msg_cpp::driving_controls>("driving_controls_msg_cpp", 10);

    ros::spin();
}
