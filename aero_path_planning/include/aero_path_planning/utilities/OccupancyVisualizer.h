/**
 * @file   OccupancyVisualizer.h
 *
 * @date   Apr 15, 2013
 * @author parallels
 * @brief  \TODO
 */

#ifndef OCCUPANCYVISUALIZER_H_
#define OCCUPANCYVISUALIZER_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include <pcl/filters/passthrough.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/utilities/OccupancyGrid.h>
//***********    NAMESPACES     ****************//

namespace aero_path_planning
{

class OccupancyVisualizer
{
public:
	OccupancyVisualizer();
	/**
	 * @author Adam Panzica
	 * @param [in]  grid  The OccupancyGrid to visualize a portion of
	 * @param [in]  trait The PointTrait type to visualize
	 * @param [out] out   PointCloud to store the resultant visualization to
	 */
	void visualizeGridTrait(const OccupancyGrid& grid, const PointTrait& trait, PointCloud& out);

	/**
	 * @author Adam Panzica
	 * @brief Sets the publishers to use to publish visualization data
	 * @param [in] obstacle_pub  Publisher to output obstacle data to
	 * @param [in] unknown_pub   Publisher to output unkown data to
	 * @param [in] free_pub      Publisher to ouput free data to
	 * @param [in] difficult_pub Publisher to output difficult terrain data to
	 * @param [in] unkown_pub    Publisher to output unkown data to
	 */
	void setVisualizationPublishers(ros::Publisher* obstacle_pub, ros::Publisher* unknown_pub, ros::Publisher* free_pub, ros::Publisher* difficult_pub, ros::Publisher* unkown_pub);

	/**
	 * @author Adam Panzica
	 * @brief Visualizes an entier occupancy grid
	 * @param [in] grid The gird to visualize
	 * @return True if it was succesfully visualized (all publishers were valid), else false
	 */
	bool visualizeGrid(const OccupancyGridPtr grid);

private:
	pcl::PassThrough<Point> filter_; ///Passthrough filter to filter with

	ros::Publisher* obstacle_pub;
	ros::Publisher* unknown_pub;
	ros::Publisher* free_pub;
	ros::Publisher* difficult_pub;
	ros::Publisher* goal_pub;

};

} /* namespace aero_path_planning */
#endif /* OCCUPANCYVISUALIZER_H_ */
