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
#include <aero_path_planning/utilities/FitnessQueue.h>
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
double ed_huristic(const aero_path_planning::Point& point, const aero_path_planning::Point& goal);

/**
 * @author Adam Panzica
 * @brief  Calculates the PointTrait weighted cost of moving through a point
 * @param [in] this_point The point to calculate the cost for
 * @param [in] last_point The last point in the chain to use for calculating cost
 * @return The cost for moving to this_point @todo fill in actual math here
 */
double pt_cost(const aero_path_planning::Point& this_point, const aero_path_planning::Point& last_point);

/**
 * @author Adam Panzica
 * @brief  Simple node class for use with an A* search based on an OccupancyGrid
 */
class AStarNode
{
public:
	typedef boost::shared_ptr<AStarNode> AStarNodePtr;
	AStarNode();
	AStarNode(const AStarNodePtr& copy);
	AStarNode(const AStarNode&    copy);

	/**
	 * @author Adam Panzica
	 * @param [in] location  The location of this node in search space
	 * @param [in] goal      The goal location in seach space
	 * @param [in] parent    The parent AStarNode of this node
	 * @param [in] cost      The cost_func to use with calculating the cost of this node
	 * @param [in] huristic  The huristic_func to use whith calculating the huristic value of this node
	 */
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

	/**
	 * @author Adam Panzica
	 * @return The parent node of this node
	 */
	const AStarNodePtr& getParent() const;

	/**
	 * @author Adam Panzica
	 * @param [in] node The node to compare against
	 * @return true if the nodes are at the same location
	 */
	bool sameLocation(const AStarNode& node) const;


	AStarNode& operator= (AStarNode const & rhs);
	/**
	 * @author Adam Panzica
	 * @param rhs
	 * @return this->f_ < rhs.f_
	 */
	bool       operator< (AStarNode const & rhs) const;
	/**
	 * @author Adam Panzica
	 * @param rhs
	 * @return this->f_ <= rhs.f_
	 */
	bool       operator<=(AStarNode const & rhs) const;
	/**
	 * @author Adam Panzica
	 * @param rhs
	 * @return this->f_ > rhs.f_
	 */
	bool       operator> (AStarNode const & rhs) const;
	/**
	 * @author Adam Panzica
	 * @param rhs
	 * @return this->f_ >= rhs.f_
	 */
	bool       operator>=(AStarNode const & rhs) const;
	/**
	 * @author Adam Panzica
	 * @param rhs
	 * @return this->f_ == rhs.f_
	 */
	bool       operator==(AStarNode const & rhs) const;

	friend std::ostream& operator<<(std::ostream& in, const AStarNode& rhs)
	{
		std::string parent((rhs.getParent()!=AStarNode::AStarNodePtr())?("Yes"):("No"));
		in<<"Node(Location: "<<rhs.getLocation().x<<","<<rhs.getLocation().y<<", Parent: "<<parent
		  <<", G:"<<rhs.getG()<<",H:"<<rhs.getH()<<",F:"<<rhs.getF()<<")";
		return in;
	}

	friend std::ostream& operator<<(std::ostream& in, const AStarNodePtr& rhs)
	{
		return in<<(*rhs);
	}

private:
	aero_path_planning::Point location_; ///The location of this node in seach space
	AStarNodePtr parent_;                ///The parent AStarNode of this node
	double g_;                           ///The total cost of moving to this node
	double h_;                           ///The huristic value of this node
	double f_;                           ///The fitness of this node, \[f=g+h\]
};




}

class AStarCarrot : public aero_path_planning::CarrotPathFinder
{
public:
	AStarCarrot();
	AStarCarrot(const AStarCarrot& copy);
	virtual ~AStarCarrot();

	virtual bool setCarrotDelta(double delta);
	virtual bool setSearchMap(const aero_path_planning::OccupancyGrid& map);
	virtual bool setCollision(collision_func_& collision_checker);
	virtual bool allowsPartialPath();
	virtual bool search(const aero_path_planning::Point& start_point, const aero_path_planning::Point& goal_point, ros::Duration& timeout, std::queue<aero_path_planning::Point>& result_path);
	virtual bool getPlanningType(std::string& type) const;

	AStarCarrot& operator=(AStarCarrot const &copy);

private:
	typedef astar_utilities::AStarNode Node_t;
	typedef astar_utilities::AStarNode::AStarNodePtr NodePtr_t;

	bool has_delta_;
	bool has_map_;
	bool has_coll_;

	int                               delta_;
	aero_path_planning::OccupancyGrid map_;
	collision_func_                   collision_checker_;

	Point n_pxy_;
	Point n_xpy_;
	Point n_nxy_;
	Point n_xny_;
	Point n_dpxpy_;
	Point n_dnxpy_;
	Point n_dpxny_;
	Point n_dnxny_;

	void setUpNeighborPoints();

	bool calcNeighbors(const Point& point, std::vector<Point>& neighbors) const;

	void buildSolutionPath(const Node_t& goal_node, std::queue<Point>& path) const;

	bool canSearch() const;

	bool openSetContains(const NodePtr_t& node, const aero_path_planning::FitnessQueue<Node_t, NodePtr_t>& open_set) const;

};

}



#endif /* ASTARCARROT_H_ */
