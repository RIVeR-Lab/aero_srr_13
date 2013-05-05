/**
 * @file   occupancy_visualizer_node.cpp
 *
 * @date   Apr 15, 2013
 * @author parallels
 * @brief  \TODO
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <ros/ros.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/utilities/OccupancyVisualizer.h>
//***********    NAMESPACES     ****************//

using namespace aero_path_planning;
namespace sm = sensor_msgs;

class OccVisNode
{
public:
	OccVisNode()
	{
		this->occ_sub       = nh_.subscribe("occupancy", 2, &OccVisNode::occCB, this);
		this->obstacle_pub  = nh_.advertise<sm::PointCloud2>("obstacle_data", 2);
		this->unknown_pub   = nh_.advertise<sm::PointCloud2>("unknown_data", 2);
		this->free_pub      = nh_.advertise<sm::PointCloud2>("free_data", 2);
		this->difficult_pub = nh_.advertise<sm::PointCloud2>("difficult_data", 2);
		this->goal_pub      = nh_.advertise<sm::PointCloud2>("goal_data", 2);
		this->visualizer_.setVisualizationPublishers(&obstacle_pub, &unknown_pub, &free_pub, &difficult_pub, &goal_pub);
	}

	void occCB(const OccupancyGridMsgConstPtr& message)
	{
		OccupancyGridPtr grid(new OccupancyGrid(*message));
		this->visualizer_.visualizeGrid(grid);
	}
private:
	OccupancyVisualizer visualizer_;

	ros::NodeHandle nh_;
	ros::Subscriber occ_sub;
	ros::Publisher  obstacle_pub;
	ros::Publisher  unknown_pub;
	ros::Publisher  free_pub;
	ros::Publisher  difficult_pub;
	ros::Publisher  goal_pub;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "occupancy_visualizer");
	OccVisNode node;
	ros::spin();

}
