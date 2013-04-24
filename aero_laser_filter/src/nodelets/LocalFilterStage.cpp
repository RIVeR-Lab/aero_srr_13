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
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <boost/bind.hpp>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_laser_filter/nodelets/LocalFilterStage.h>
//***********    NAMESPACES     ****************//

PLUGINLIB_DECLARE_CLASS(aero_laser_filter, LocalFilterStage, aero_laser_filter::LocalFilterStage, nodelet::Nodelet)


namespace aero_laser_filter
{

	void LocalFilterStage::onInit()
	{
		this->nh_          = this->getNodeHandle();
		this->p_nh_        = this->getPrivateNodeHandle();
		this->transformer_ = new tf::TransformListener(this->nh_);
		this->dr_server_   = new dynamic_reconfigure::Server<LocalStageConfig>(this->p_nh_);
		this->dr_server_->setCallback(boost::bind(&LocalFilterStage::drCB, this, _1, _2));

		this->loadParams();
		this->registerTopics();
	}

	void LocalFilterStage::loadParams()
	{
		this->input_topic_ = "input_topic";
		this->output_topic_= "output_topic";
		this->output_frame_= "laser";
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

		temp_x = 10;
		temp_y = 10;
		temp_z = 0;
		this->p_nh_.getParam(crop_ns+toprgt_ns+x, temp_x);
		this->p_nh_.getParam(crop_ns+toprgt_ns+y, temp_y);
		this->p_nh_.getParam(crop_ns+toprgt_ns+z, temp_z);
		this->crop_top_right_.x = temp_x;
		this->crop_top_right_.y = temp_y;
		this->crop_top_right_.z = temp_z;

		this->robot_size_    = 1;
		this->inflation_res_ = 0.1;
		this->p_nh_.getParam("robot_size", this->robot_size_);
		this->p_nh_.getParam("inflation_res", this->inflation_res_);
	}

	void LocalFilterStage::registerTopics()
	{
		this->point_sub_ = this->nh_.subscribe(this->input_topic_, 2, &LocalFilterStage::cloudCB, this);
		this->point_pub_ = this->nh_.advertise<sm::PointCloud2>(this->output_topic_, 2);
	}

	void LocalFilterStage::cloudCB(const sm::PointCloud2ConstPtr& message)
	{
		NODELET_INFO_STREAM("Local: Got New Point Cloud Message");
		sm::PointCloud2Ptr output_cloud(new sm::PointCloud2());
		PointCloudPtr_t cropped_cloud(new PointCloud_t());
		PointCloudPtr_t processed_cloud(new PointCloud_t());
		pcl::fromROSMsg(*message, *cropped_cloud);
		NODELET_INFO_STREAM("Local: Extracted Raw Point Cloud from Message");

		this->cropCloud(cropped_cloud, cropped_cloud);
		NODELET_INFO_STREAM("Local: Croped Point Cloud");
		this->filterCloud(cropped_cloud, processed_cloud);
		NODELET_INFO_STREAM("Local: Filtered Point Cloud");
		this->transformCloud(processed_cloud, processed_cloud);
		NODELET_INFO_STREAM("Local: Transformed Point Cloud");

		pcl::toROSMsg(*processed_cloud, *output_cloud);
		this->point_pub_.publish(output_cloud);
		NODELET_INFO_STREAM("Local: Published Point Cloud");

	}

	void LocalFilterStage::cropCloud(const PointCloudPtr_t& in, PointCloudPtr_t& out)
	{
		this->crop_box_.setMin(this->crop_bottom_left_.getVector4fMap());
		this->crop_box_.setMax(this->crop_top_right_.getVector4fMap());
		this->crop_box_.setInputCloud(in);
		this->crop_box_.filter(*out);
	}

	void LocalFilterStage::transformCloud(const PointCloudPtr_t& in, PointCloudPtr_t& out)
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

	void LocalFilterStage::filterCloud(const PointCloudPtr_t& in, PointCloudPtr_t& out)
	{
		//Need to store this first incase in=out
		int size = in->size();
		NODELET_INFO_STREAM("Local: Filtering Cloud... Size: "<<size);
#pragma omp parallel for
		for(int i=0; i<size; i++)
		{
			//For now, we're just going to do a really simple box inflation
			for(int x=0; x<this->robot_size_; x+=this->inflation_res_)
			{
				for(int y=0; y<this->robot_size_; y+=this->inflation_res_)
				{
					NODELET_INFO_STREAM("Local: Inflating Point... Index: "<<i<<", x: "<<x<<", y: "<<y);
					Point_t inflation_point(in->at(i));
					inflation_point.x+= x-robot_size_/2.0;
					inflation_point.y+= y-robot_size_/2.0;
					out->push_back(inflation_point);
				}
			}
		}
	}

	void LocalFilterStage::drCB(const LocalStageConfig& config, uint32_t level)
	{
		this->crop_bottom_left_.x = config.crop_min_x;
		this->crop_bottom_left_.y = config.crop_min_y;
		this->crop_top_right_.x   = config.crop_max_x;
		this->crop_top_right_.y   = config.crop_max_y;
		this->robot_size_         = config.inflation_size;
		this->inflation_res_      = config.inflation_resolution;

	}

} /* namespace aero_laser_filter */
