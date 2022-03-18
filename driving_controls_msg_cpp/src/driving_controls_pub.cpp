#include "ros/ros.h"
#include "driving_controls_msg_cpp/driving_controls.h"
#include <stdlib.h>
#include <sstream>

using namespace std;

//Progran to manualy enter values for the sevo and ESC

//function to remap the given values to a PWM value
uint remap(int numIn, int InMin, int InMax, uint outMin, uint outMax)
{
    return (numIn - InMin) * (outMax - outMin) / (InMax - InMin) + outMin;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<driving_controls_msg_cpp::driving_controls>("driving_controls_msg_cpp", 10);
    ros::Rate loop_rate(10);

    //loop to ask for value and publish it on a rostopic
    while (ros::ok())
    {
        driving_controls_msg_cpp::driving_controls msg;
        uint32_t value = 0;
        cout << "speed: ";
        cin >> value;
        value = remap(value, -100, 100, 200, 400);
        msg.speed = value;
        cout << "steering: ";
        cin >> value;
        value = remap(value, -25, 25, 265, 376);
        
        msg.steering = value;

        cout << "given speed: " << msg.speed << endl
             << "given steering: " << msg.steering << endl;

        pub.publish(msg);
        loop_rate.sleep();
    }

    return 0;
}