#include "ros/ros.h"
#include <signal.h>
#include "driving_controls_msg_cpp/driving_controls.h"
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <ros/callback_queue.h>

#include <thread>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <JHPWMPCA9685.h>

using namespace std;
// servo pwm board pin 0
// ESC pwm board pin 1

//servo max right 265
//servo max left 376

// ESC stop 300
// ESC forward min 320, max 400
// ESC backwards min 298, max 200

bool CanDrive = false;
bool isStoped = false;

PCA9685 *pca9685 = new PCA9685();

ros::Subscriber obstacles_PRIO_messages_sub;
ros::CallbackQueue PRIO_Calback_queue;

void obstaclePRIOCallback(const std_msgs::Bool &CanDriveMsg)
{
    //ROS_WARN_STREAM("status: " << (CanDriveMsg.data ? "drive" : "stop"));
    CanDrive = CanDriveMsg.data;
}

//print the incomming values and check if there valid and than set the values
void setValues(const driving_controls_msg_cpp::driving_controls &msg)
{
    //ROS_WARN("speed: [%d]", msg.speed);
    //ROS_WARN("steering: [%d]", msg.steering);

    //check if the incoming value is valid and set the PWM signal

    if (msg.speed < 410 && msg.speed > 205 && CanDrive)
    {
        ROS_DEBUG("valid speed value");
        pca9685->setPWM(0, 0, msg.speed);
        isStoped = false;
    }
    else if (!isStoped){
        pca9685->setPWM(0, 0, 285);
        usleep(8 * 1000);
        pca9685->setPWM(0, 0, 308);   
    }
    else
    {
        isStoped = true;
    }

    if (msg.steering >= 265 && msg.steering <= 376)
    {
        ROS_DEBUG("valid steering value");
        pca9685->setPWM(1, 0, msg.steering);
    }
}

void signalInterrupt(int sig)
{
    //stop the car when user shutsdown the program
    ROS_WARN_STREAM("STOPPING CAR");
    pca9685->openPCA9685();
    pca9685->setPWM(0, 0, 300);
    ros::Duration(0.2).sleep();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driving_controls_sub");
    ros::NodeHandle n;

    //make a priority callback for obstacle detection data
    ros::NodeHandle nh_PRIO;
    ros::CallbackQueue *PRIO_Calback_queue_ptr = &PRIO_Calback_queue;
    nh_PRIO.setCallbackQueue(PRIO_Calback_queue_ptr);

    //signal interrupt handler
    signal(SIGINT, signalInterrupt);

    obstacles_PRIO_messages_sub = nh_PRIO.subscribe("obstacle_detection_control", 50, obstaclePRIOCallback);

    std::thread PRIO_thread_spinner = std::thread([PRIO_Calback_queue_ptr]() {
        ros::SingleThreadedSpinner PRIO_spinner;
        PRIO_spinner.spin(PRIO_Calback_queue_ptr);
    });

    //check the conecction on the PWM board
    int err = pca9685->openPCA9685();
    if (err < 0)
    {
        printf("Error: %d", pca9685->error);
    }
    else
    {
        printf("PCA9685 Device Address: 0x%02X\n", pca9685->kI2CAddress);
        pca9685->setAllPWM(0, 0);
        pca9685->reset();
        pca9685->setPWMFrequency(50);

        ros::Subscriber sub = n.subscribe("driving_controls_msg_cpp", 1000, setValues);
        ros::spin();
    }

    PRIO_thread_spinner.join();
    return 0;
}