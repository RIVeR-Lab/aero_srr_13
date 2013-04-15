/**
 * @file   OccupancyVisualizer.cpp
 *
 * @date   Apr 15, 2013
 * @author parallels
 * @brief  \TODO
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/utilities/OccupancyVisualizer.h>

//***********    NAMESPACES     ****************//
namespace sm = sensor_msgs;
namespace aero_path_planning
{

OccupancyVisualizer::OccupancyVisualizer():
		obstacle_pub(NULL),
		unknown_pub(NULL),
		free_pub(NULL),
		difficult_pub(NULL),
		goal_pub(NULL)
{
	this->filter_.setFilterFieldName("rgba");
}

void OccupancyVisualizer::visualizeGridTrait(const OccupancyGrid& grid, const PointTrait& trait, PointCloud& out)
{
	PointCloudConstPtr gridptr(new PointCloud(grid.getGrid()));
	this->filter_.setInputCloud(gridptr);
	this->filter_.setFilterLimits(trait, trait);
	this->filter_.filter(out);
}

void OccupancyVisualizer::setVisualizationPublishers(ros::Publisher* obstacle_pub, ros::Publisher* unknown_pub, ros::Publisher* free_pub, ros::Publisher* difficult_pub, ros::Publisher* goal_pub)
{
	this->obstacle_pub  = obstacle_pub;
	this->unknown_pub   = unknown_pub;
	this->free_pub      = free_pub;
	this->difficult_pub = difficult_pub;
	this->goal_pub      = goal_pub;
}

bool OccupancyVisualizer::visualizeGrid(const OccupancyGridPtr grid)
{
	bool success = (this->obstacle_pub!=NULL)&&(this->unknown_pub!=NULL)&&(this->free_pub!=NULL)&&(this->difficult_pub!=NULL)&&(this->goal_pub!=NULL);
	if(success)
	{
		sm::PointCloud2Ptr obsctacle_cloud_msg(new sm::PointCloud2());
		PointCloud obsctacle_cloud;
		sm::PointCloud2Ptr unkown_cloud_msg(new sm::PointCloud2());
		PointCloud unkown_cloud;
		sm::PointCloud2Ptr free_cloud_msg(new sm::PointCloud2());
		PointCloud free_cloud;
		sm::PointCloud2Ptr difficult_cloud_msg(new sm::PointCloud2());
		PointCloud difficult_cloud;
		sm::PointCloud2Ptr goal_cloud_msg(new sm::PointCloud2());
		PointCloud goal_cloud;


		this->visualizeGridTrait(*grid, OBSTACLE, obsctacle_cloud);
		pcl::toROSMsg(obsctacle_cloud, *obsctacle_cloud_msg);
		obsctacle_cloud_msg->header.frame_id = grid->getFrameId();
		this->obstacle_pub->publish(obsctacle_cloud_msg);

		this->visualizeGridTrait(*grid, UNKNOWN, unkown_cloud);
		pcl::toROSMsg(unkown_cloud, *unkown_cloud_msg);
		unkown_cloud_msg->header.frame_id = grid->getFrameId();
		this->unknown_pub->publish(unkown_cloud_msg);

		this->visualizeGridTrait(*grid, FREE_LOW_COST, free_cloud);
		pcl::toROSMsg(free_cloud, *free_cloud_msg);
		free_cloud_msg->header.frame_id = grid->getFrameId();
		this->free_pub->publish(free_cloud_msg);

		this->visualizeGridTrait(*grid, FREE_HIGH_COST, difficult_cloud);
		pcl::toROSMsg(difficult_cloud, *difficult_cloud_msg);
		difficult_cloud_msg->header.frame_id = grid->getFrameId();
		this->difficult_pub->publish(difficult_cloud_msg);

		this->visualizeGridTrait(*grid, GOAL, goal_cloud);
		pcl::toROSMsg(goal_cloud, *goal_cloud_msg);
		goal_cloud_msg->header.frame_id = grid->getFrameId();
		this->goal_pub->publish(goal_cloud_msg);
	}

	return success;
}

} /* namespace aero_path_planning */
