/**
 * @file   AStarCarrot.h
 *
 * @date   Mar 29, 2013
 * @author Adam Panzica
 * @brief  Class definitions of the A* algorithm as a CarrotPathFinder
 */

#ifndef ASTARCARROT_H_
#define ASTARCARROT_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include <queue>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/planning_strategies/CarrotPathFinder.h>
//***********    NAMESPACES     ****************//

namespace aero_path_planning
{

namespace astar_utilities
{
typedef boost::function<double (aero_path_planning::Point, aero_path_planning::Point)> huristic_func;
typedef boost::function<double (aero_path_planning::Point, aero_path_planning::Point)> cost_func;

/**
 * @author Adam Panzica
 * @brief  Calculates a point huristic based on euclidian distance
 * @param [in] point The point to measure the huristic from
 * @param [in] goal  The goal point to measure the huristic to
 * @return The euclidian distance between the points
 */
double eculidian_distance_huristic(const aero_path_planning::Point& point, const aero_path_planning::Point& goal);

/**
 * @author Adam Panzica
 * @brief  Calculates the PointTrait weighted cost of moving through a point
 * @param [in] this_point The point to calculate the cost for
 * @param [in] last_point The last point in the chain to use for calculating cost
 * @return The cost for moving to this_point @todo fill in actual math here
 */
double point_trait_cost(const aero_path_planning::Point& this_point, const aero_path_planning::Point& last_point);

class AStarNode
{
public:
	typedef boost::shared_ptr<AStarNode> AStarNodePtr;
	AStarNode();
	AStarNode(const AStarNodePtr& copy);
	AStarNode(const AStarNode&    copy);
	AStarNode(const Point& location, const Point& goal, const AStarNodePtr parent, cost_func cost, huristic_func huristic);

	/**
	 * @author Adam Panzica
	 * @return The location of this node
	 */
	const Point& getLocation() const;

	/**
	 * @author Adam Panzica
	 * @return The cost of moving through this node
	 */
	double getG() const;

	/**
	 * @author Adam Panzica
	 * @return The huristic value of this node
	 */
	double getH() const;

	/**
	 * @author Adam Panzica
	 * @return The fitness of this node, = H+G
	 */
	double getF() const;

	AStarNode& operator= (AStarNode const & rhs);
	bool       operator< (AStarNode const & rhs) const;
	bool       operator<=(AStarNode const & rhs) const;
	bool       operator> (AStarNode const & rhs) const;
	bool       operator>=(AStarNode const & rhs) const;
	bool       operator==(AStarNode const & rhs) const;

private:
	aero_path_planning::Point location_;
	AStarNodePtr parent_;
	double g_;
	double h_;
	double f_;
};

}

class AStarCarrot : public aero_path_planning::CarrotPathFinder
{
public:
	AStarCarrot();
};

}



#endif /* ASTARCARROT_H_ */
