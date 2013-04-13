/**
 * @file   GlobalFilterStage.cpp
 *
 * @date   Apr 13, 2013
 * @author Adam Panzica
 * @brief  Class implemention for the GlobalFilter nodelet
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <sensor_msgs/PointCloud2.h>
#include <pluginlib/class_list_macros.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_laser_filter/nodelets/GlobalFilterStage.h>
//***********    NAMESPACES     ****************//

PLUGINLIB_DECLARE_CLASS(aero_laser_filter, GlobalFilterStage, aero_laser_filter::GlobalFilterStage, nodelet::Nodelet)


namespace aero_laser_filter {

void GlobalFilterStage::onInit()
{
	this->nh_   = this->getNodeHandle();
	this->p_nh_ = this->getPrivateNodeHandle();
	this->loadParams();
	this->registerTopics();
}

void GlobalFilterStage::loadParams()
{
	this->input_topic_ = "input_topic";
	this->output_topic_= "output_topic";

	this->p_nh_.getParam(this->input_topic_,  this->input_topic_);
	this->p_nh_.getParam(this->output_topic_, this->output_topic_);
}

void GlobalFilterStage::registerTopics()
{
	this->point_sub_ = this->nh_.subscribe(this->input_topic_, 2, &GlobalFilterStage::cloudCB, this);
	this->point_pub_ = this->nh_.advertise<sm::PointCloud2>(this->output_topic_, 2);
}

void GlobalFilterStage::cloudCB(const sm::PointCloud2ConstPtr& message)
{
	sm::PointCloud2Ptr cloud_message(new sm::PointCloud2());

}

} /* namespace aero_laser_filter */
