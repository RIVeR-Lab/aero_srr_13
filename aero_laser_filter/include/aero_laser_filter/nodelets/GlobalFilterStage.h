/**
 * @file   GlobalFilterStage.h
 *
 * @date   Apr 13, 2013
 * @author Adam Panzica
 * @brief  Class definition for the GlobalFilter nodelet
 */

#ifndef GLOBALFILTERSTAGE_H_
#define GLOBALFILTERSTAGE_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pcl/filters/crop_box.h>
#include <tf/transform_listener.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_laser_filter/utilities/AeroLaserFilterUtilities.h>
//***********    NAMESPACES     ****************//

namespace aero_laser_filter
{

namespace sm = sensor_msgs;


class GlobalFilterStage: public nodelet::Nodelet
{
public:
	virtual void onInit();

private:

	void loadParams();
	void registerTopics();

	void cloudCB(const sm::PointCloud2ConstPtr& message);

	void cropCloud(const PointCloudPtr_t& in, PointCloudPtr_t& out);

	void transformCloud(const PointCloudPtr_t& in, PointCloudPtr_t& out);

	void filterCloud(const PointCloudPtr_t& in, PointCloudPtr_t& out);

	ros::NodeHandle nh_;
	ros::NodeHandle p_nh_;
	ros::Publisher  point_pub_;
	ros::Subscriber point_sub_;

	tf::TransformListener *transformer_;

	pcl::CropBox<Point_t> crop_box_;
	Point_t crop_bottom_left_;
	Point_t crop_top_right_;

	std::string input_topic_;
	std::string output_topic_;
	std::string output_frame_;
};

} /* namespace aero_laser_filter */
#endif /* GLOBALFILTERSTAGE_H_ */
