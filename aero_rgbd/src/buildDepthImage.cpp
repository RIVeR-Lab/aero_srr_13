#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "std_msgs/String.h"
#include <sstream>

class DepthImageBuilder {

    private:
        sensor_msgs::Image depthImageMsg;

        template<typename CloudT> void
        toROSDepthMsg( const CloudT cloud )
        {
            //Test to make sure the cloud is dense.
            if(cloud->width == 0 && cloud->height == 0 )
                throw std::runtime_error("Needs to be a dense cloud.");
            else
            {
                if(cloud->points.size() != cloud->width * cloud->height)
                    throw std::runtime_error("The width and height do not match the cloud size.");
                //Set the dimensions of the depthImage
                depthImageMsg.height = cloud->height;
                depthImageMsg.width = cloud->width;
            }
            //The image is the Z coordinates from the cloud frame, i.e. the depths in mm as 16bit unsigned ints
            depthImageMsg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
            depthImageMsg.step = depthImageMsg.width * sizeof(uint8_t);
            depthImageMsg.data.resize( depthImageMsg.step * depthImageMsg.height );

            //Copy data from the cloud to the image.
            for( size_t y = 0; y < cloud->height; y++ )
            {
                for( size_t x = 0; x < cloud->width; x++ )
                {
                    uint8_t *pixel = &( depthImageMsg.data[ y * depthImageMsg.step + x ] );
                    memcpy( pixel, &cloud->at(x, y).z, sizeof(uint8_t));
                }
            }
        }

    public:
        void pointCloud_callback( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input )
        {
            toROSDepthMsg( input );
        }

        sensor_msgs::Image getDepthImageMsg(){return depthImageMsg;}
};

int main(int argc, char **argv)
{
    DepthImageBuilder builder;

    //Initialize, and create a node handle
    ros::init(argc, argv, "buildDepthImage");
    ros::NodeHandle n;

    //Create subscriber for the PointCloud2
    ros::Subscriber sub = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >( "input", 1, &DepthImageBuilder::pointCloud_callback, &builder );

    //Just publishing junk message until I get the image message code written.
    //Setup image transport and publish the image message.
    image_transport::ImageTransport it(n);
    image_transport::Publisher imagePublisher = it.advertise("/upper_stereo/depth_image", 1);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "Listening for image data. " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        imagePublisher.publish(builder.getDepthImageMsg());

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
