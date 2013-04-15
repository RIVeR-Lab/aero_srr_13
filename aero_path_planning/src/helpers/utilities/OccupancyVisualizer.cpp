/**
 * @file   OccupancyVisualizer.cpp
 *
 * @date   Apr 15, 2013
 * @author parallels
 * @brief  \TODO
 */

//*********** SYSTEM DEPENDANCIES ****************//

//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/utilities/OccupancyVisualizer.h>

//***********    NAMESPACES     ****************//
namespace aero_path_planning {

OccupancyVisualizer::OccupancyVisualizer()
{
	this->filter_.setFilterFieldName("rgba");
}

void OccupancyVisualizer::visualizeGrid(const OccupancyGrid& grid, const PointTrait& trait, PointCloud& out)
{
	PointCloudConstPtr gridptr(grid.getGrid());
	this->filter_.setInputCloud(gridptr);
	this->filter_.setFilterLimits(trait, trait);
	this->filter_.filter(out);
}

} /* namespace aero_path_planning */
