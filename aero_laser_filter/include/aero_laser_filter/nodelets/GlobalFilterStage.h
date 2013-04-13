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
//************ LOCAL DEPENDANCIES ****************//

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

	ros::NodeHandle nh_;
	ros::NodeHandle p_nh_;
	ros::Publisher  point_pub_;
	ros::Subscriber point_sub_;

	std::string input_topic_;
	std::string output_topic_;
};

} /* namespace aero_laser_filter */
#endif /* GLOBALFILTERSTAGE_H_ */
