/**
 * @file PathFinder.h
 *
 * @date   Feb 19, 2013
 * @author Adam Panzica
 * @brief Interface definition for a global path finder
 */

//License File


#ifndef PATHFINDER_H_
#define PATHFINDER_H_

//****************SYSTEM DEPENDANCIES**************************//
#include<boost/function.hpp>
#include<deque>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/utilities/OccupancyGrid.h>
//**********************NAMESPACES*****************************//

namespace aero_path_planning
{
	/**
	 * @author Adam Panzica
	 * Defines an interface used for creating carrot-paths for the global planner. Not that the CarrotPlanner may assume holonomicy. The idea is that it will
	 * generate a series of points between start and goal along a path at some semi-regular interval, and then a local-planner will be responsable for connecting
	 * the points smoothly and fully collision free. This allows the global-planner to update its path much more slowly without worry of colliding with obstacles.
	 */
	class CarrotPathFinder
	{
	public:
		virtual ~CarrotPathFinder(){};
		/**
		 * @author Adam Panzica
		 * Typedef defining a valid collision checking function. The function should consume
		 * an aero_path_planning::Point and an aero_path_planning::OccupancyGrid and return
		 * true if the point is in collision, else false. This function will likely be called
		 * many many times, so should be as fast as possible
		 */
		typedef boost::function< bool (const aero_path_planning::Point&, const aero_path_planning::OccupancyGrid&)> collision_func_;


		/**
		 * @author Adam Panzica
		 * @brief Sets the distance, in grid coordinates, that should be between points on the path
		 * @param [in] delta The minimum distance between points, in grid coordinates
		 * @return True if sucessfully set, else false
		 */
		virtual bool setCarrotDelta(double delta) = 0;

		/**
		 * @author Adam Panzica
		 * @brief Sets the map for the path planner to search
		 * @param [in] map The map to search over
		 * @return True if sucessfully set, else false
		 */
		virtual bool setSearchMap(const aero_path_planning::OccupancyGrid& map) = 0;

		/**
		 * @author Adam Panzica
		 * @brief Sets the collision checking function to use
		 * @param [in] collision_checker The collision checking function to use for searching the map
		 * @return True if sucessfully set else false
		 */
		virtual bool setCollision(collision_func_& collision_checker) = 0;

		/**
		 * @author Adam Panzica
		 * @brief  Checks to see if the path finder will return partial paths on timeout
		 * @return True if partial paths are supported, else false
		 */
		virtual bool allowsPartialPath() = 0;

		/**
		 * @author Adam Panzica
		 * @brief Performs the actual search operation and generates a new path
		 * @param [in]  start_point The starting point on the map to search
		 * @param [in]  goal_point  The goal point on the map to generate a point to
		 * @param [in]  timeout     The amount of time to leat the path finder run before timing out
		 * @param [out] result_path A queue to store the resulting path in.
		 * @return True if a complete path was found, else false
		 *
		 * Note that if a timeout occured, the planner may still return a partial path. This will be a per-implementation behavroi however,
		 * and support should be checked by calling the allowsPartialPath() method
		 *
		 */
		virtual bool search(const aero_path_planning::Point& start_point, const aero_path_planning::Point& goal_point, ros::Duration& timeout, std::deque<aero_path_planning::Point>& result_path) = 0;

		/**
		 * @author Adam Panzica
		 * @brief Gets the type of the planner
		 * @param [out] type A string to write the discriptive type of the planner to
		 * @return True if sucessful, else false
		 */
		virtual bool getPlanningType(std::string& type) const = 0;
	};
}; /*END aero_path_planning */

#endif /* PATHFINDER_H_ */
