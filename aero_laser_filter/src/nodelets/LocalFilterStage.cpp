/**
 * @file   LocalFilterStage.cpp
 *
 * @date   Apr 13, 2013
 * @author parallels
 * @brief  \TODO
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <sensor_msgs/PointCloud2.h>
#include <pluginlib/class_list_macros.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_laser_filter/nodelets/LocalFilterStage.h>
//***********    NAMESPACES     ****************//

PLUGINLIB_DECLARE_CLASS(aero_laser_filter, LocalFilterStage, aero_laser_filter::LocalFilterStage, nodelet::Nodelet)


namespace aero_laser_filter
{

	void LocalFilterStage::onInit()
	{
		this->nh_   = this->getNodeHandle();
		this->p_nh_ = this->getPrivateNodeHandle();
		this->loadParams();
		this->registerTopics();
	}

	void LocalFilterStage::loadParams()
	{
		this->input_topic_ = "input_topic";
		this->output_topic_= "output_topic";

		this->p_nh_.getParam(this->input_topic_,  this->input_topic_);
		this->p_nh_.getParam(this->output_topic_, this->output_topic_);
	}

	void LocalFilterStage::registerTopics()
	{
		this->point_sub_ = this->nh_.subscribe(this->input_topic_, 2, &LocalFilterStage::cloudCB, this);
		this->point_pub_ = this->nh_.advertise<sm::PointCloud2>(this->output_topic_, 2);
	}

	void LocalFilterStage::cloudCB(const sm::PointCloud2ConstPtr& message)
	{
		sm::PointCloud2Ptr cloud_message(new sm::PointCloud2());

	}

} /* namespace aero_laser_filter */
