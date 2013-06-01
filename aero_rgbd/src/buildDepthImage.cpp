#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "std_msgs/String.h"
#include <sstream>

//Need to figure out what point type Samir is using.
void pointCloud_callBack( const pcl::PointCloud<T> input )
{

}

int main(int argc, char **argv)
{
    //Initialize, and create a node handle
    ros::init(argc, argv, "buildDepthImage");
    ros::NodeHandle n;

    //Create subscriber for the PointCloud2
    ros::Subscriber sub = n.subscribe( "input", 1, pointCloud_callBack );

    //Just publishing junk message until I get the image message code written.
    ros::Publisher depthImage_publisher = n.advertise<std_msgs::String>("Someday_I_ll_be_an_image", 1000);

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
