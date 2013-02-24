/**
 * @file RRTCarrot.h
 *
 * @date   Feb 19, 2013
 * @author Adam Panzica
 * @brief \todo
 */

//License File

#ifndef RRTCARROT_H_
#define RRTCARROT_H_

//****************SYSTEM DEPENDANCIES**************************//
#include<boost/random.hpp>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/CarrotPathFinder.h>
//**********************NAMESPACES*****************************//

namespace aero_path_planning
{
/**
 * @author Adam Panzica
 * @brief Simple container struct for holding information on an RRT node
 */
struct RRTNode
{
	boost::shared_ptr<RRTNode> parent_;  ///Pointer to the parent node of this node
	Point                      location; ///Pointer to the location of this node on a map
};

typedef boost::shared_ptr<RRTNode> node_ptr_t;

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
	typedef std::deque<node_ptr_t > node_deque;
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

	~RRTCarrotTree();

	/**
	 * @author Adam Panzica
	 * @brief Adds a new node to the tree
	 * @param [in] node The new node to add
	 * @return True if sucessfully added, else false
	 */
	bool addNode(const node_ptr_t node);

	/**
	 * @author Adam Panzica
	 * @brief Finds the nearest neighbor in the euclidian sense in the tree to the given node
	 * @param [in] to_node The node to find the nearest neighbor to
	 * @return A pointer to the node that is the neighest neighbor of the given node
	 */
	node_ptr_t findNearestNeighbor(const node_ptr_t to_node) const;

	/**
	 * @author Adam Panzica
	 * @brief Gets the last leaf node that was added to the tree
	 * @return Pointer to the last leaf node that was added to the tree
	 */
	node_ptr_t getLeafNode();

	/**
	 * @author Adam Panzica
	 * @brief  Gets the root node of a tree
	 * @return The root node of the tree
	 */
	node_ptr_t getRootNode();

	/**
	 * @author Adam Panzica
	 * @brief  Empties the tree
	 */
	void flushTree();


	/**
	 * @author Adam Panzica
	 * @brief Creates a visualization of the tree in the form of a sensor_msgs::image
	 * @param [out] message The Image message to create the visualization in
	 * @param [in]  x_height The x-width of the image
	 * @param [in]  y_height The y-width of the image
	 */
	void visualizeTree(sensor_msgs::Image& message, int x_height, int y_height);

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
	virtual bool allowsPartialPath();
	virtual bool search(const aero_path_planning::Point& start_point, const aero_path_planning::Point& goal_point, ros::Duration& timeout, std::queue<aero_path_planning::Point>& result_path);
	virtual bool getPlanningType(std::string& type) const;

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
	 *
	 * Note: The sampled node is origin corrected for the map being searched
	 */
	bool sample(node_ptr_t node);

	/**
	 * @author Adam Panzica
	 * @brief Produces the next node stepping along a vector between nodes
	 * @param [in]  last_node   The node to step with
	 * @param [in]  step_vector The vector to step along
	 * @param [out] next_node   Node to write the next step location along the vector to
	 * @return True if sucessfully stepped, else false
	 */
	bool step(node_ptr_t last_node, const Eigen::Vector4f& step_vector, node_ptr_t next_node);

	/**
	 * @author Adam Panzica
	 * @brief  Attempts to connect a randomly sampled node to a node on the tree
	 * @param [in]  q_rand    The randomly sampled node to connect to
	 * @param [in]  tree_node The node on the tree to connect from
	 * @param [out] tree      The tree to add newly connected nodes to
	 * @return True if they fully connected, else false
	 */
	bool connect(const node_ptr_t q_rand, node_ptr_t tree_node, RRTCarrotTree* tree);

	/**
	 * @author Adam Panzica
	 * @brief Mereges a path that spans two trees
	 * @param path_1_node The node that should serve as the parent of the merge
	 * @param path_2_node The node that will be the first child of the merge
	 * @return True if paths sucessfully merged, else false
	 *
	 * This function is intended tp merge a pthat that spans two trees. path_1_node is the terminous of the path on the tree that should serve as the
	 * 'parent' tree. path_2_node is the terminous of that path in the tree that should act as the child. This function will traverse the path begining
	 * at path_2_node, overwriting the parent nodes in order to merge the two paths.
	 */
	bool mergePath(node_ptr_t path_1_node, node_ptr_t path_2_node);

	/**
	 * @author Adam Panzica
	 * @brief  Generates a new random location on the map
	 * @param  [out] x pointer to write the x location to
	 * @param  [out] y pointer to write the y location to
	 * @param  [out] z pointer to write the z location to
	 *
	 * Note that the location is not origin corrected
	 */
	void genLoc(int* x, int* y, int* z);

	/**
	 * @author Adam panzica
	 * @brief Calculates the step vector to increment by when connecting q-tree to q-rand
	 * @param [in]  from_point The Point to calculate the vector from
	 * @param [in]  to_point   The Point to caluclate the vector to
	 * @param [out]  vector     The vector to write the result to
	 * @return True if vector calculated, else false
	 */
	bool generateStepVector(const Point& from_point, const Point& to_point, Eigen::Vector4f& vector);


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
