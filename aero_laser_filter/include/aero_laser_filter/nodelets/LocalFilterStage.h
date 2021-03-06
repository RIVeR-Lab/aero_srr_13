/**
 * @file   LocalFilterStage.h
 *
 * @date   Apr 13, 2013
 * @author Adam Panzica
 * @brief  Class definition for the LocalFilter stage nodelet
 */

#ifndef LOCALFILTERSTAGE_H_
#define LOCALFILTERSTAGE_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pcl/filters/crop_box.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_laser_filter/utilities/AeroLaserFilterUtilities.h>
#include <aero_laser_filter/LocalStageConfig.h>
//***********    NAMESPACES     ****************//

namespace aero_laser_filter
{

namespace sm = sensor_msgs;

class LocalFilterStage: public nodelet::Nodelet
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

	void drCB(const aero_laser_filter::LocalStageConfig& config, uint32_t level);



	ros::NodeHandle nh_;
	ros::NodeHandle p_nh_;
	ros::Publisher  point_pub_;
	ros::Subscriber point_sub_;

	dynamic_reconfigure::Server<LocalStageConfig>* dr_server_;

	tf::TransformListener *transformer_;

	pcl::CropBox<Point_t> crop_box_;
	Point_t crop_bottom_left_;
	Point_t crop_top_right_;

	double robot_size_;
	double inflation_res_;

	std::string input_topic_;
	std::string output_topic_;
	std::string output_frame_;
};

} /* namespace aero_laser_filter */
#endif /* LOCALFILTERSTAGE_H_ */
