/**
 * @file   ConversionStage.cpp
 *
 * @date   Apr 13, 2013
 * @author parallels
 * @brief  \TODO
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <sensor_msgs/PointCloud2.h>
#include <pluginlib/class_list_macros.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_laser_filter/nodelets/ConversionStage.h>
//***********    NAMESPACES     ****************//

PLUGINLIB_DECLARE_CLASS(aero_laser_filter, ConversionStage, aero_laser_filter::ConversionStage, nodelet::Nodelet)

namespace aero_laser_filter {

void ConversionStage::onInit()
{
	this->nh_   = this->getNodeHandle();
	this->p_nh_ = this->getPrivateNodeHandle();
	this->loadParams();
	this->registerTopics();
}

void ConversionStage::loadParams()
{
	this->input_topic_ = "input_topic";
	this->output_topic_= "output_topic";

	this->p_nh_.getParam(this->input_topic_,  this->input_topic_);
	this->p_nh_.getParam(this->output_topic_, this->output_topic_);
}

void ConversionStage::registerTopics()
{
	this->scan_sub_ = this->nh_.subscribe(this->input_topic_, 2, &ConversionStage::scanCB, this);
	this->point_pub_= this->nh_.advertise<sm::PointCloud2>(this->output_topic_, 2);
}

void ConversionStage::scanCB(const sm::LaserScanConstPtr& message)
{
	sm::PointCloud2Ptr cloud_message(new sm::PointCloud2());
	this->projector_.projectLaser(*message, *cloud_message);
	this->point_pub_.publish(cloud_message);
}

} /* namespace aero_laser_filter */
