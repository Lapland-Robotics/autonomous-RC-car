#include "ros/ros.h"
#include <signal.h>
#include "driving_controls_msg_cpp/driving_controls.h"
#include "std_msgs/String.h"

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

PCA9685 *pca9685 = new PCA9685();

int getkey()
{
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO | ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

//print the incomming values and check if there valid and than set the values
void setValues(const driving_controls_msg_cpp::driving_controls &msg)
{
    //ROS_WARN("speed: [%d]", msg.speed);
    //ROS_WARN("steering: [%d]", msg.steering);
    if (msg.speed < 410 && msg.speed > 205)
    {
        ROS_DEBUG("valid speed value");
        pca9685->setPWM(0, 0, msg.speed);
    }

    if (msg.steering >= 265 && msg.steering <= 376)
    {
        ROS_DEBUG("valid steering value");
        pca9685->setPWM(1, 0, msg.steering);
    }
}

void mySigintHandler(int sig)
{
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

    signal(SIGINT, mySigintHandler);

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
    return 0;
}