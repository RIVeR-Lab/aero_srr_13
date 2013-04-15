/**
 * @file   ConversionStage.h
 *
 * @date   Apr 13, 2013
 * @author parallels
 * @brief  \TODO
 */

#ifndef CONVERSIONSTAGE_H_
#define CONVERSIONSTAGE_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include<nodelet/nodelet.h>
#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<laser_geometry/laser_geometry.h>
//************ LOCAL DEPENDANCIES ****************//

//***********    NAMESPACES     ****************//

namespace aero_laser_filter {

namespace sm = sensor_msgs;
namespace lg = laser_geometry;

class ConversionStage : public nodelet::Nodelet
{
public:
	virtual void onInit();

private:

	void scanCB(const sm::LaserScanConstPtr& message);

	void loadParams();
	void registerTopics();

	lg::LaserProjection projector_;

	ros::NodeHandle nh_;
	ros::NodeHandle p_nh_;
	ros::Subscriber scan_sub_;
	ros::Publisher  point_pub_;

	std::string input_topic_;
	std::string output_topic_;
};

} /* namespace aero_laser_filter */
#endif /* CONVERSIONSTAGE_H_ */
