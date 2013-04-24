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
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <boost/bind.hpp>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_laser_filter/nodelets/GlobalFilterStage.h>
//***********    NAMESPACES     ****************//

PLUGINLIB_DECLARE_CLASS(aero_laser_filter, GlobalFilterStage, aero_laser_filter::GlobalFilterStage, nodelet::Nodelet)


namespace aero_laser_filter {

	void GlobalFilterStage::onInit()
	{
		this->nh_          = this->getNodeHandle();
		this->p_nh_        = this->getPrivateNodeHandle();
		this->transformer_ = new tf::TransformListener(this->nh_);
		this->dr_server_   = new dynamic_reconfigure::Server<GlobalStageConfig>(this->p_nh_);
		this->dr_server_->setCallback(boost::bind(&GlobalFilterStage::drCB, this, _1, _2));

		this->loadParams();
		this->registerTopics();
	}

	void GlobalFilterStage::loadParams()
	{
		this->input_topic_ = "input_topic";
		this->output_topic_= "output_topic";

		this->p_nh_.getParam(this->input_topic_,  this->input_topic_);
		this->p_nh_.getParam(this->output_topic_, this->output_topic_);
		this->output_frame_= "/world";
		std::string crop_ns("crop");
		std::string btmlft_ns("bottom_left");
		std::string toprgt_ns("top_right");
		std::string x("x");
		std::string y("y");
		std::string z("z");

		this->p_nh_.getParam(this->input_topic_,  this->input_topic_);
		this->p_nh_.getParam(this->output_topic_, this->output_topic_);
		this->p_nh_.getParam("output_frame", this->output_frame_);

		double temp_x = 0;
		double temp_y = 0;
		double temp_z = 0;
		this->p_nh_.getParam(crop_ns+btmlft_ns+x, temp_x);
		this->p_nh_.getParam(crop_ns+btmlft_ns+y, temp_y);
		this->p_nh_.getParam(crop_ns+btmlft_ns+z, temp_z);
		this->crop_bottom_left_.x = temp_x;
		this->crop_bottom_left_.y = temp_y;
		this->crop_bottom_left_.z = temp_z;

		temp_x = 30;
		temp_y = 30;
		temp_z = 0;
		this->p_nh_.getParam(crop_ns+toprgt_ns+x, temp_x);
		this->p_nh_.getParam(crop_ns+toprgt_ns+y, temp_y);
		this->p_nh_.getParam(crop_ns+toprgt_ns+z, temp_z);
		this->crop_top_right_.x = temp_x;
		this->crop_top_right_.y = temp_y;
		this->crop_top_right_.z = temp_z;
	}

	void GlobalFilterStage::registerTopics()
	{
		this->point_sub_ = this->nh_.subscribe(this->input_topic_, 2, &GlobalFilterStage::cloudCB, this);
		this->point_pub_ = this->nh_.advertise<sm::PointCloud2>(this->output_topic_, 2);
	}

	void GlobalFilterStage::cloudCB(const sm::PointCloud2ConstPtr& message)
	{
		//NODELET_INFO_STREAM("Global: Got New Global Laser Data");
		sm::PointCloud2Ptr output_cloud(new sm::PointCloud2());
		PointCloudPtr_t processed_cloud(new PointCloud_t());
		pcl::fromROSMsg(*message, *processed_cloud);
		//NODELET_INFO_STREAM("Global: Extracted Raw Point Cloud from Message");
		this->cropCloud(processed_cloud, processed_cloud);
		//NODELET_INFO_STREAM("Global: Croped Point Cloud");
		this->filterCloud(processed_cloud, processed_cloud);
		//NODELET_INFO_STREAM("Global: Filtered Point Cloud");
		this->transformCloud(processed_cloud, processed_cloud);
		//NODELET_INFO_STREAM("Global: Transformed Point Cloud");

		pcl::toROSMsg(*processed_cloud, *output_cloud);
		this->point_pub_.publish(output_cloud);
		//NODELET_INFO_STREAM("Global: Published Point Cloud");

	}

	void GlobalFilterStage::cropCloud(const PointCloudPtr_t& in, PointCloudPtr_t& out)
	{
		this->crop_box_.setMin(this->crop_bottom_left_.getVector4fMap());
		this->crop_box_.setMax(this->crop_top_right_.getVector4fMap());
		this->crop_box_.setInputCloud(in);
		this->crop_box_.filter(*out);
	}

	void GlobalFilterStage::transformCloud(const PointCloudPtr_t& in, PointCloudPtr_t& out)
	{
		try
		{
			this->transformer_->waitForTransform(this->output_frame_, in->header.frame_id, in->header.stamp, ros::Duration(1.0));
			pcl_ros::transformPointCloud(this->output_frame_, *in, *out, *this->transformer_);
		}
		catch(std::exception& e)
		{
			NODELET_ERROR_STREAM_THROTTLE(5, "Problem Transforming PointCloud in GlobalFilter: "<<e.what());
			*out = *in;
		}
	}

	void GlobalFilterStage::filterCloud(const PointCloudPtr_t& in, PointCloudPtr_t& out)
	{
		//for now no filtering, just pass through
		*out = *in;
	}

	void GlobalFilterStage::drCB(const GlobalStageConfig& config, uint32_t level)
	{
		this->crop_bottom_left_.x = config.crop_min_x;
		this->crop_bottom_left_.y = config.crop_min_y;
		this->crop_top_right_.x   = config.crop_max_x;
		this->crop_top_right_.y   = config.crop_max_y;
	}

} /* namespace aero_laser_filter */
