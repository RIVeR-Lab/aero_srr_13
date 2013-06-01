nclude "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "buildDepthImage");

    ros::NodeHandle n;

    //Just publishing junk message until I get the image message code written.
    ros::Publisher depthImage_publisher = n.advertise<std_msgs::String>("Someday I'll be an image", 1000);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "Listening for image data. " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        depthImage_publisher.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
