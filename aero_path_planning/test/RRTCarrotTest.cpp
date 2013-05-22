///**
// * @file RRTCarrotTest.cpp
// *
// * @date   Feb 23, 2013
// * @author Adam Panzica
// * @brief Test Set for RRTCarrot
// */
//
////License File
//
////****************SYSTEM DEPENDANCIES**************************//
//#include<gtest/gtest.h>
//#include<deque>
////*****************LOCAL DEPENDANCIES**************************//
//#include<aero_path_planning/planning_strategies/RRTCarrot.h>
////**********************NAMESPACES*****************************//
//using namespace aero_path_planning;
//
//#define PRINT_NODE_STREAM(node) "[Location: ("<<node->location_.x<<","<<node->location_.y<<","<<node->location_.z<<"), Parent: "<<node->parent_<<"]"
//#define PRINT_EXPECT_NODE(expect,actual) "Expected to get "<<PRINT_NODE_STREAM(expect)<<", Actually got "<<PRINT_NODE_STREAM(actual)
//
////************************************ TEST RRTNODE ***************************************************//
//TEST(RRTNodeTest, testRRTNode)
//{
//	boost::shared_ptr<RRTNode> test_node(new RRTNode);
//	test_node->location_.x = 0;
//	test_node->location_.y = 0;
//	test_node->location_.z = 0;
//	ASSERT_EQ(0, test_node->location_.x);
//	ASSERT_EQ(0, test_node->location_.y);
//	ASSERT_EQ(0, test_node->location_.z);
//	ASSERT_EQ(boost::shared_ptr<RRTNode>(), test_node->parent_);
//
//	RRTNode test_node2;
//	test_node2.location_ = test_node->location_;
//	test_node2.parent_  = test_node;
//
//	ASSERT_EQ(test_node->location_, test_node2.location_);
//	ASSERT_EQ(test_node->location_, test_node2.parent_->location_);
//}
//
////************************************ TEST RRTCARROTTREE **********************************************//
//
//TEST(RRTCarrotTreeTest, testTreeSetup)
//{
//	RRTCarrotTree test_tree;
//
//	//Size should be the initial guess capacity
//	ASSERT_EQ(0, test_tree.size());
//}
//
//TEST(RRTCarrotTreeTest, testAddingNode)
//{
//	RRTCarrotTree test_tree;
//
//	//Push a node onto the tree
//	node_ptr_t test_node1(new RRTNode());
//	test_node1->location_.x = 10;
//	test_node1->location_.y = 0;
//	test_node1->location_.z = 0;
//
//	ASSERT_TRUE(test_tree.addNode(test_node1));
//
//	//Tree size shouldn't have changed since we guessed a size of 10
//	ASSERT_EQ(1, test_tree.size());
//}
//
//TEST(RRTCarrotTreeTest, testFlush)
//{
//	int initial_tree_size = 0;
//	RRTCarrotTree test_tree;
//	//The tree should be empty
//	ASSERT_EQ(initial_tree_size, test_tree.size());
//
//	//Push a node onto the tree
//	//Push a node onto the tree
//	node_ptr_t test_node1(new RRTNode());
//	test_node1->location_.x = 10;
//	test_node1->location_.y = 0;
//	test_node1->location_.z = 0;
//
//	test_tree.addNode(test_node1);
//
//	//The tree should now be size one
//	ASSERT_EQ(initial_tree_size+1, test_tree.size());
//
//	//And the number of references to test_node1 should now be 2
//	ASSERT_EQ(2, test_node1.use_count());
//
//	//Flush the tree, size should be 0, number of references should be 1
//	test_tree.flushTree();
//	ASSERT_EQ(initial_tree_size, test_tree.size());
//	ASSERT_EQ(1, test_node1.use_count());
//}
//
//TEST(RRTCarrotTreeTest, testRootLeafAccess)
//{
//	RRTCarrotTree test_tree;
//	//The tree should be empty
//	ASSERT_EQ(boost::shared_ptr<RRTNode>(), test_tree.getLeafNode());
//	ASSERT_EQ(boost::shared_ptr<RRTNode>(), test_tree.getRootNode());
//
//	//Push a node onto the tree
//	node_ptr_t test_node1(new RRTNode());
//	test_node1->location_.x = 10;
//	test_node1->location_.y = 0;
//	test_node1->location_.z = 0;
//
//	test_tree.addNode(test_node1);
//
//	//Now this node should be both root and leaf
//	ASSERT_EQ(test_node1->location_, test_tree.getLeafNode()->location_);
//	ASSERT_EQ(test_node1->location_, test_tree.getRootNode()->location_);
//
//	//Push a second node onto the tree
//	node_ptr_t test_node2(new RRTNode());
//	test_node1->location_.x = 10;
//	test_node1->location_.y = 5;
//	test_node1->location_.z = 0;
//
//	test_tree.addNode(test_node2);
//
//	//Now the root node should still be test_node1, but the leaf node should be test_node2
//	ASSERT_EQ(test_node1->location_, test_tree.getRootNode()->location_);
//	ASSERT_EQ(test_node2->location_, test_tree.getLeafNode()->location_);
//}
//
//TEST(RRTCarrotTreeTest, testCopy)
//{
//	RRTCarrotTree test_tree;
//
//
//	//Test Copying tree with nodes
//	node_ptr_t test_node1(new RRTNode());
//	test_node1->location_.x = 10;
//	test_node1->location_.y = 0;
//	test_node1->location_.z = 0;
//
//	test_tree.addNode(test_node1);
//
//	RRTCarrotTree test_copy_tree_rf(test_tree);
//	ASSERT_EQ(1, test_copy_tree_rf.size());
//	node_ptr_t result1 = test_copy_tree_rf.getRootNode();
//	ASSERT_EQ(test_node1->location_, result1->location_)<<PRINT_EXPECT_NODE(test_node1, result1);
//
//	RRTCarrotTree test_copy_tree_ptr(&test_tree);
//	ASSERT_EQ(1, test_copy_tree_ptr.size());
//	node_ptr_t result2 = test_copy_tree_ptr.getRootNode();
//	ASSERT_EQ(test_node1->location_, result1->location_)<<PRINT_EXPECT_NODE(test_node1, result2);
//
//	RRTCarrotTree test_copy_tree_eq = test_tree;
//	ASSERT_EQ(1, test_copy_tree_eq.size());
//	node_ptr_t result3 = test_copy_tree_eq.getRootNode();
//	ASSERT_EQ(test_node1->location_, result1->location_)<<PRINT_EXPECT_NODE(test_node1, result3);
//}
//
//TEST(RRTCarrotTreeTest, testNearestNeighbor)
//{
//	RRTCarrotTree test_tree;
//	//The tree should be empty
//	ASSERT_EQ(0, test_tree.size());
//
//	//Push a series of nodes onto the tree
//	node_ptr_t test_node1(new RRTNode());
//	test_node1->location_.x = 0;
//	test_node1->location_.y = 0;
//	test_node1->location_.z = 0;
//
//	node_ptr_t test_node2(new RRTNode());
//	test_node2->location_.x = 10;
//	test_node2->location_.y = 0;
//	test_node2->location_.z = 0;
//
//	node_ptr_t test_node3(new RRTNode());
//	test_node3->location_.x = 0;
//	test_node3->location_.y = 10;
//	test_node3->location_.z = 0;
//
//	test_tree.addNode(test_node2);
//	test_tree.addNode(test_node3);
//	test_tree.addNode(test_node1);
//
//
//	//Find the nearest neighbor to a node at 1,1. Should be test_node1
//	node_ptr_t test_node4(new RRTNode());
//	test_node4->location_.x = 1;
//	test_node4->location_.y = 1;
//	test_node4->location_.z = 0;
//
//
//	node_ptr_t result1 = test_tree.findNearestNeighbor(test_node4);
//	EXPECT_EQ(test_node1->location_, result1->location_)<<PRINT_EXPECT_NODE(test_node1, result1);
//
//	//Find the nearest neighbor to a node at 11,1, should be test_node2
//	test_node4->location_.x = 11;
//	node_ptr_t result2 = test_tree.findNearestNeighbor(test_node4);
//	EXPECT_EQ(test_node2->location_, result2->location_)<<PRINT_EXPECT_NODE(test_node2, result2);
//
//	//Find the nearest neighbor to a node at 5,11, should be test_node3
//	test_node4->location_.x = 5;
//	test_node4->location_.y = 11;
//	node_ptr_t result3 = test_tree.findNearestNeighbor(test_node4);
//	EXPECT_EQ(test_node3->location_, result3->location_)<<PRINT_EXPECT_NODE(test_node3, result3);
//}
//
////************************************ TEST COLLISIONCHECK ***************************************************//
//
//bool collisionCheck(const tf::Point& point, const occupancy_grid::MultiTraitOccupancyGrid& map)
//{
//	try
//	{
//		if(map.getPointTrait(point.x(), point.y())==occupancy_grid::utilities::CellTrait::OBSTACLE)
//		{
//			return true;
//		}
//	}
//	catch(std::exception& e)
//	{
//		return true;
//	}
//	return false;
//}
//
//TEST(CollisionCheckTest, testCollision)
//{
//	tf::Point origin;
//	origin.x=0;
//	origin.y=0;
//	origin.z=0;
//	OccupancyGrid coll_grid(10,10,1,origin);
//	Point obst_point;
//	obst_point.x = 5;
//	obst_point.y = 5;
//	obst_point.z = 0;
//	obst_point.rgba = aero_path_planning::OBSTACLE;
//	coll_grid.setPointTrait(obst_point);
//
//	ASSERT_TRUE(collisionCheck(obst_point, coll_grid));
//
//	Point clear_point;
//	clear_point.x=1;
//	clear_point.y=1;
//	clear_point.z=0;
//	ASSERT_FALSE(collisionCheck(clear_point,coll_grid));
//}
//
////************************************ TEST RRTCARROT ***************************************************//
//
//class RRTCarrotTestFixture : public ::testing::Test, protected RRTCarrot
//{
//protected:
//	RRTCarrotTestFixture():RRTCarrot(1)
//{
//		//Neded because RRTCarrot uses calls to ros::Time but ROS is not initialized in a test fixture
//		ros::Time::init();
//		this->origin_.x = 0;
//		this->origin_.y = 0;
//		this->origin_.z = 0;
//		this->x_size_   = 50;
//		this->y_size_   = 50;
//}
//	void setUpPlanner()
//	{
//		this->setCarrotDelta(1);
//		CarrotPathFinder::collision_func_ cf;
//		cf = boost::bind(&collisionCheck, _1, _2);
//		this->setCollision(cf);
//		OccupancyGrid search_grid(this->x_size_,this->y_size_,1,this->origin_);
//		this->setSearchMap(search_grid);
//		this->seedSampler(ros::Time::now().toNSec());
//
//	}
//
//	Point origin_;
//	int   x_size_;
//	int   y_size_;
//};
//
//TEST_F(RRTCarrotTestFixture, testSetDelta)
//{
//	ASSERT_TRUE(this->setCarrotDelta(10));
//}
//
//TEST_F(RRTCarrotTestFixture, testSetAllowsIncompPath)
//{
//	ASSERT_TRUE(this->allowsPartialPath());
//}
//
//TEST_F(RRTCarrotTestFixture, testSetCollision)
//{
//	CarrotPathFinder::collision_func_ cf;
//	cf = boost::bind(&collisionCheck, _1, _2);
//	ASSERT_TRUE(this->setCollision(cf));
//}
//
//TEST_F(RRTCarrotTestFixture, testSetMap)
//{
//	OccupancyGrid search_grid(this->x_size_,this->y_size_,1,this->origin_);
//	ASSERT_TRUE(this->setSearchMap(search_grid));
//}
//
//TEST_F(RRTCarrotTestFixture, testInitialization)
//{
//	ASSERT_TRUE(this->setCarrotDelta(1));
//	CarrotPathFinder::collision_func_ cf;
//	cf = boost::bind(&collisionCheck, _1, _2);
//	ASSERT_TRUE(this->setCollision(cf));
//	OccupancyGrid search_grid(this->x_size_,this->y_size_,1,this->origin_);
//	ASSERT_TRUE(this->setSearchMap(search_grid));
//	ASSERT_TRUE(this->isInialized());
//}
//
//TEST_F(RRTCarrotTestFixture, testSample)
//{
//	node_ptr_t q_rand(new RRTNode());
//	//Should fail as the planner is not yet initialized
//	ASSERT_FALSE(this->sample(q_rand));
//
//	//Initializes the planner
//	this->setUpPlanner();
//
//	//Should pass now because the planner is initialized
//	ASSERT_TRUE(this->sample(q_rand));
//	//Make sure the point was actually on the map
//	ASSERT_LE(q_rand->location_.x, this->x_size_);
//	ASSERT_LE(q_rand->location_.y, this->y_size_);
//	ASSERT_LE(q_rand->location_.z, 1);
//	//Make sure that q_rand's use count hasn't increased
//	ASSERT_EQ(1, q_rand.use_count());
//
//	//Take a second sample to ensure they're each unique
//	node_ptr_t q_rand2(new RRTNode());
//	ASSERT_TRUE(this->sample(q_rand2));
//
//	ASSERT_FALSE(this->nodesEqual(q_rand2, q_rand));
//
//}
//
//TEST_F(RRTCarrotTestFixture, testStep)
//{
//	//Set up the planner
//	this->setUpPlanner();
//
//	//Set up some nodes and the step vector
//	node_ptr_t from_node(new RRTNode());
//	node_ptr_t to_node(new RRTNode());
//
//	from_node->location_.x = 0;
//	from_node->location_.y = 0;
//	from_node->location_.z = 0;
//
//	Point step_vector;
//	step_vector.x = 1;
//	step_vector.y = 1;
//	step_vector.z = 0;
//
//	node_ptr_t expected_result(new RRTNode());
//	expected_result->parent_  = from_node;
//	expected_result->location_= step_vector;
//
//	//Step along the vector
//	ASSERT_TRUE(this->step(from_node, step_vector.getVector4fMap(), to_node));
//	//Check to see that we wound up where we should have
//	ASSERT_EQ(expected_result->location_.x, to_node->location_.x)<<PRINT_EXPECT_NODE(expected_result, to_node);
//	ASSERT_EQ(expected_result->location_.y, to_node->location_.y)<<PRINT_EXPECT_NODE(expected_result, to_node);
//	ASSERT_EQ(expected_result->location_.z, to_node->location_.z)<<PRINT_EXPECT_NODE(expected_result, to_node);
//	ASSERT_EQ(expected_result->parent_,     to_node->parent_)    <<PRINT_EXPECT_NODE(expected_result, to_node);
//}
//
//TEST_F(RRTCarrotTestFixture, testNodesEqual)
//{
//	//Set up test nodes
//	node_ptr_t node1(new RRTNode());
//	node1->location_ = this->origin_;
//	node_ptr_t node2(new RRTNode());
//	node2->location_.x =10;
//	node2->location_.y =0;
//	node2->location_.z =0;
//	node_ptr_t node3(new RRTNode());
//	node3->location_.x =0;
//	node3->location_.y =10;
//	node3->location_.z =0;
//	node_ptr_t node4(new RRTNode());
//	node4->location_.x =0;
//	node4->location_.y =0;
//	node4->location_.z =10;
//	node_ptr_t node5(new RRTNode());
//	node5->location_.x =10;
//	node5->location_.y =10;
//	node5->location_.z =10;
//	node_ptr_t node6(new RRTNode());
//	node6->location_ = this->origin_;
//
//	ASSERT_FALSE(this->nodesEqual(node1, node2));
//	ASSERT_FALSE(this->nodesEqual(node1, node3));
//	ASSERT_FALSE(this->nodesEqual(node1, node4));
//	ASSERT_FALSE(this->nodesEqual(node1, node5));
//	ASSERT_TRUE (this->nodesEqual(node1, node6));
//}
//
//TEST_F(RRTCarrotTestFixture, testGenerateStepVector)
//{
//	//Need to set up planner to initialize step size
//	this->setUpPlanner();
//	//Set up some nodes to generate a vector between
//	node_ptr_t start_node(new RRTNode());
//	node_ptr_t end_node(new RRTNode());
//	start_node->location_ = this->origin_;
//	end_node->location_.x = 10;
//	end_node->location_.y = 0;
//	end_node->location_.z = 0;
//	Eigen::Vector4f step_vector;
//
//	ASSERT_TRUE(this->generateStepVector(start_node->location_, end_node->location_, step_vector));
//
//	//Given a step size of 1, the step vector should be [1;0;0;0]
//	Point vector;
//	vector.getVector4fMap() = step_vector;
//	EXPECT_EQ(1, vector.x);
//	EXPECT_EQ(0, vector.y);
//	EXPECT_EQ(0, vector.z);
//}
//
//TEST_F(RRTCarrotTestFixture, testConnect)
//{
//	//Set up the planner
//	this->setUpPlanner();
//
//	//Set up a test tree to write results too
//	RRTCarrotTree test_tree;
//
//	//Set up a start node and an end node that will result in a connected straight line
//	node_ptr_t  start_node(new RRTNode());
//	node_ptr_t  end_node(new RRTNode());
//	start_node->location_=this->origin_;
//	end_node->location_.x = 10;
//	end_node->location_.y = 0;
//
//	test_tree.addNode(start_node);
//
//	//Connect the start node to the end node. This should fully connect as there are no obsticales
//	ASSERT_TRUE(this->connect(end_node, start_node, &test_tree));
//	//The root of test_tree should now be start_node, and the leaf should be end_node
//	node_ptr_t root_node = test_tree.getRootNode();
//	node_ptr_t leaf_node = test_tree.getLeafNode();
//	ASSERT_TRUE(this->nodesEqual(start_node, root_node))<<PRINT_EXPECT_NODE(start_node, root_node);
//	ASSERT_TRUE(this->nodesEqual(end_node,   leaf_node))<<PRINT_EXPECT_NODE(end_node,   leaf_node);
//	//The tree should now contain 12 nodes, since the step size is 1
//	ASSERT_EQ(12, test_tree.size());
//}
//
//TEST_F(RRTCarrotTestFixture, testMergePath)
//{
//	//Set up the planner, and a set of nodes in seperate 'trees' that will get connected
//	this->setUpPlanner();
//
//	node_ptr_t    tree1_node1(new RRTNode());
//	tree1_node1->parent_ = node_ptr_t();
//	node_ptr_t    tree1_node2(new RRTNode());
//	node_ptr_t    tree1_node3(new RRTNode());
//	tree1_node2->parent_=tree1_node1;
//	tree1_node3->parent_=tree1_node3;
//
//	node_ptr_t    tree2_node1(new RRTNode());
//	tree2_node1->parent_ = node_ptr_t();
//	node_ptr_t    tree2_node2(new RRTNode());
//	node_ptr_t    tree2_node3(new RRTNode());
//	tree2_node2->parent_=tree2_node1;
//	tree2_node3->parent_=tree2_node2;
//
//	//Merge the two trees
//	ASSERT_TRUE(this->mergePath(tree1_node3, tree2_node3));
//	//tree2_node1's parent should now be tree2_node2
//	ASSERT_TRUE(this->nodesEqual(tree2_node1->parent_, tree2_node2))<<PRINT_EXPECT_NODE(tree2_node2, tree2_node1->parent_);
//	//tree2_node3's parent should new be tree1_node3
//	ASSERT_TRUE(this->nodesEqual(tree2_node3->parent_, tree1_node3))<<PRINT_EXPECT_NODE(tree1_node3, tree2_node3->parent_);
//
//}
//
//TEST_F(RRTCarrotTestFixture, testSearch)
//{
//	//set up the planner and a goal point
//	this->setUpPlanner();
//	Point goal_point;
//	goal_point.x = this->x_size_-this->origin_.x;
//	goal_point.x = this->y_size_-this->origin_.x;
//	goal_point.z = 0;
//	std::deque<Point> path;
//	ros::Duration timeout(1);
//	//Perform a search
//	ASSERT_TRUE(this->search(this->origin_, goal_point, timeout, path));
//	ASSERT_FALSE(path.empty());
//	node_ptr_t path_goal(new RRTNode());
//	node_ptr_t path_end(new RRTNode());
//	path_goal->location_ = goal_point;
//	path_end->location_  = path.front();
//	ASSERT_TRUE(this->nodesEqual(path_goal, path_end))<<PRINT_EXPECT_NODE(path_goal, path_end);
//
//
//}
//
