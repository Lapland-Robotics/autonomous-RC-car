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
// #include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <stdlib.h>
#include <iostream>
#include <math.h>

#define RAD2DEG(x) ((x)*180. / M_PI)

using namespace std;

ros::Publisher pub;
std_msgs::Float32 objectDistance;
ros::Time lastScan;

void frontDetection(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    if (ros::Time::now() - lastScan > ros::Duration(0.5))
    {
        int count = scan->scan_time / scan->time_increment;

        for (int i = 0; i < count; i++)
        {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            //check if the data is from the front
            if ((degree < 0 && degree > -1 || degree > 0 && degree < 1) && !isinf(scan->ranges[i])) //
            {

                objectDistance.data = scan->ranges[i];
                cout << scan->ranges[i] << endl;
                pub.publish(objectDistance);
            }
        }
        lastScan = ros::Time::now();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;
    pub = n.advertise<std_msgs::Float32>("/object_distance", 1000);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, frontDetection);
    lastScan = ros::Time::now();
    ros::spin();
    return 0;
}
