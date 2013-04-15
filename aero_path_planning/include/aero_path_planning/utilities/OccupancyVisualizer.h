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
	void visualizeGrid(const OccupancyGrid& grid, const PointTrait& trait, PointCloud& out);

private:
	pcl::PassThrough<Point> filter_; //Passthrough filter to filter with
};

} /* namespace aero_path_planning */
#endif /* OCCUPANCYVISUALIZER_H_ */
