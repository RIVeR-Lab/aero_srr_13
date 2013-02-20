/**
 * @file RRTCarrot.h
 *
 * @date   Feb 19, 2013
 * @author Adam Panzica
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<boost/random.hpp>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/CarrotPathFinder.h>
//**********************NAMESPACES*****************************//

#ifndef RRTCARROT_H_
#define RRTCARROT_H_

namespace aero_path_planning
{

/**
 * @author Adam Panzica
 * @brief Simple container struct for holding information on an RRT node
 */
struct RRTNode
{
	RRTNode*                   parent_;  ///Pointer to the parent node of this node
	aero_path_planning::Point* location; ///Pointer to the location of this node on a map
};

/**
 * @author Adam Panzica
 * @brief  Simple RRT tree class
 *
 * The RRTCarrot tree provides a simple tree interface for RRT searches.
 * It provides methods for adding new nodes to the tree, finding the nearest
 * neighbor to a node in the tree (uses Euclidian distance), and clearing the
 * tree.
 */
class RRTCarrotTree
{
private:
	typedef std::deque<RRTNode> node_deque;
public:

	/**
	 * @author Adam Panzica
	 * @brief  Standard empty constructor for STL compliance
	 */
	RRTCarrotTree();
	/**
	 * @author Adam Panzica
	 * @brief Standard copy constructor
	 * @param [in] copy RRTCarrtTree to copy
	 */
	RRTCarrotTree(const RRTCarrotTree& copy);
	/**
	 * @author Adam Panzica
	 * @brief Standard copy constructor
	 * @param [in] copy RRTCarrtTree to copy
	 */
	RRTCarrotTree(const RRTCarrotTree* copy);
	/**
	 * @author Adam Panzica
	 * @brief Initializes a new tree with a given guess at tree size
	 * @param [in] size Sets the initial capacity of the tree
	 *
	 * Note that the size parameter isn't a hard maximum, the tree will dynamically
	 * grow as needed, it is simply used to increase performance by pre-allocating
	 * space in the tree for nodes.
	 */
	RRTCarrotTree(int size);

	/**
	 * @author Adam Panzica
	 * @brief Adds a new node to the tree
	 * @param [in] node The new node to add
	 * @return True if sucessfully added, else false
	 */
	bool addNode(const RRTNode& node);

	/**
	 * @author Adam Panzica
	 * @brief Finds the nearest neighbor in the euclidian sense in the tree to the given node
	 * @param [in] to_node The node to find the nearest neighbor to
	 * @return A pointer to the node that is the neighest neighbor of the given node
	 */
	RRTNode* findNearestNeighbor(const RRTNode* to_node) const;

	/**
	 * @author Adam Panzica
	 * @brief  Empties the tree
	 */
	void flushTree();

	typedef node_deque::size_type size_type;
	/**
	 * @author Adam Panzica
	 * @brief Gets the number of nodes in the tree
	 * @return The current number of nodes in the tree
	 */
	size_type size() const;

	RRTCarrotTree& operator=(RRTCarrotTree const &copy);

private:

	node_deque nodes_; ///Container holding all of the nodes in the tree
};

/**
 * @author Adam Panzica
 * @brief Implementation of CarrotPathFinder that uses a bi-directional RRT-Connect search
 */
class RRTCarrot: public CarrotPathFinder
{
public:
	/**
	 * @author Adam Panzica
	 * @brief Default empty constructor
	 */
	RRTCarrot();
	/**
	 * @author Adam Panzica
	 * @brief Standard copy constructor
	 * @param [in] copy The RRTCarrot to copy
	 */
	RRTCarrot(const RRTCarrot& copy);
	/**
	 * @author Adam Panzica
	 * @brief Initializes a new RRTCarrot with a given step size
	 * @param [in] step_size The step size to use when performing a connect operation
	 */
	RRTCarrot(double step_size);
	virtual ~RRTCarrot();


	virtual bool setCarrotDelta(double delta);
	virtual bool setSearchMap(const aero_path_planning::OccupancyGrid& map);
	virtual bool setCollision(collision_func_& collision_checker);
	virtual bool search(const aero_path_planning::Point& start_point, const aero_path_planning::Point& goal_point, std::queue<aero_path_planning::Point*>& result_path);
	virtual bool getType(std::string& type) const;

	RRTCarrot& operator=(RRTCarrot const &copy);

private:

	/**
	 * @author Adam Panzica
	 * @brief  Initializes the random number generator for random sampling
	 */
	void randInit();
	/**
	 * @author Adam Panzica
	 * @brief Returns true when the delta, map and collision_function parameters have been set
	 */
	void isInialized();

	/**
	 * @author Adam Panzica
	 * @brief  Generates a new randomly sampled node
	 * @param  [in] node Pointer to a node to fill with a random location
	 * @return True if node sucessfully generated, else false
	 */
	bool sample(RRTNode* node);

	/**
	 * @author Adam Panzica
	 * @brief Produces the next node stepping along a vector between nodes
	 * @param [in]  last_node   The node to step from
	 * @param [in]  step_vector The vector to step along
	 * @param [out] next_node   Node to write the next step location along the vector to
	 * @return True if sucessfully stepped, else false
	 */
	bool step(const RRTNode* last_node, const Eigen::Vector4f& step_vector, RRTNode* next_node);

	/**
	 * @author Adam Panzica
	 * @brief  Attempts to connect a randomly sampled node to a node on the tree
	 * @param [in] q_rand    The randomly sampled node to connect to
	 * @param [in] tree_node The node on the tree to connect from
	 * @return True if they fully connected, else false
	 */
	bool connect(const RRTNode* q_rand, const RRTNode* tree_node);


	int  step_size_;   ///The distance to step while connecting nodes
	bool initialized_; ///Flag for signalling that the RRTCarrot is ready for searching
	bool has_delta_;   ///Flag for signalling that the delta value has been set
	bool has_coll_;    ///Flag for signalling that the collision function has been set
	bool has_map_;     ///Flag for signalling that the map has been set
	int  delta_;       ///The minimum distance between points on the generated path

	aero_path_planning::OccupancyGrid map_; ///The map to search
	collision_func_ collision_checker_;     ///The collusion checking function

	RRTCarrotTree* start_tree_; ///The tree extending from the start point
	RRTCarrotTree* goal_tree_;  ///The tree extending from the goal point

	boost::variate_generator<boost::mt19937, boost::normal_distribution<> >* rand_gen_; ///Random Number Generator
};

};
#endif /* RRTCARROT_H_ */
