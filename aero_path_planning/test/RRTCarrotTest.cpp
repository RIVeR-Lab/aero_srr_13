/**
 * @file RRTCarrotTest.cpp
 *
 * @date   Feb 23, 2013
 * @author Adam Panzica
 * @brief Test Set for RRTCarrot
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<gtest/gtest.h>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/RRTCarrot.h>
//**********************NAMESPACES*****************************//
using namespace aero_path_planning;

#define PRINT_NODE_STREAM(node) "[Location: ("<<node->location_.x<<","<<node->location_.y<<","<<node->location_.z<<"), Parent: "<<node->parent_<<"]"
#define PRINT_EXPECT_NODE(expect,actual) "Expected to get "<<PRINT_NODE_STREAM(expect)<<", Actually got "<<PRINT_NODE_STREAM(actual)

/**
 * Basic test for operation of RRTNode to make sure it does what it should
 */
TEST(RRTNodeTest, testRRTNode)
{
	boost::shared_ptr<RRTNode> test_node(new RRTNode);
	test_node->location_.x = 0;
	test_node->location_.y = 0;
	test_node->location_.z = 0;
	ASSERT_EQ(0, test_node->location_.x);
	ASSERT_EQ(0, test_node->location_.y);
	ASSERT_EQ(0, test_node->location_.z);
	ASSERT_EQ(boost::shared_ptr<RRTNode>(), test_node->parent_);

	RRTNode test_node2;
	test_node2.location_ = test_node->location_;
	test_node2.parent_  = test_node;

	ASSERT_EQ(test_node->location_.getVector4fMap(), test_node2.location_.getVector4fMap());
	ASSERT_EQ(test_node->location_.getVector4fMap(), test_node2.parent_->location_.getVector4fMap());
}

TEST(RRTCarrotTreeTest, testTreeSetup)
{
	RRTCarrotTree test_tree;

	//Size should be the initial guess capacity
	ASSERT_EQ(0, test_tree.size());
}

TEST(RRTCarrotTestTree, testAddingNode)
{
	RRTCarrotTree test_tree;

	//Push a node onto the tree
	node_ptr_t test_node1(new RRTNode());
	test_node1->location_.x = 10;
	test_node1->location_.y = 0;
	test_node1->location_.z = 0;

	ASSERT_TRUE(test_tree.addNode(test_node1));

	//Tree size shouldn't have changed since we guessed a size of 10
	ASSERT_EQ(1, test_tree.size());
}

TEST(RRTCarrotTreeTest, testFlush)
{
	int initial_tree_size = 0;
	RRTCarrotTree test_tree;
	//The tree should be empty
	ASSERT_EQ(initial_tree_size, test_tree.size());

	//Push a node onto the tree
	//Push a node onto the tree
	node_ptr_t test_node1(new RRTNode());
	test_node1->location_.x = 10;
	test_node1->location_.y = 0;
	test_node1->location_.z = 0;

	test_tree.addNode(test_node1);

	//The tree should now be size one
	ASSERT_EQ(initial_tree_size+1, test_tree.size());

	//And the number of references to test_node1 should now be 2
	ASSERT_EQ(2, test_node1.use_count());

	//Flush the tree, size should be 0, number of references should be 1
	test_tree.flushTree();
	ASSERT_EQ(initial_tree_size, test_tree.size());
	ASSERT_EQ(1, test_node1.use_count());
}

TEST(RRTCarrotTreeTest, testRootLeafAccess)
{
	int initial_tree_size = 0;
	RRTCarrotTree test_tree;
	//The tree should be empty
	ASSERT_EQ(boost::shared_ptr<RRTNode>(), test_tree.getLeafNode());
	ASSERT_EQ(boost::shared_ptr<RRTNode>(), test_tree.getRootNode());

	//Push a node onto the tree
	node_ptr_t test_node1(new RRTNode());
	test_node1->location_.x = 10;
	test_node1->location_.y = 0;
	test_node1->location_.z = 0;

	test_tree.addNode(test_node1);

	//Now this node should be both root and leaf
	ASSERT_EQ(test_node1->location_.getVector4fMap(), test_tree.getLeafNode()->location_.getVector4fMap());
	ASSERT_EQ(test_node1->location_.getVector4fMap(), test_tree.getRootNode()->location_.getVector4fMap());

	//Push a second node onto the tree
	node_ptr_t test_node2(new RRTNode());
	test_node1->location_.x = 10;
	test_node1->location_.y = 5;
	test_node1->location_.z = 0;

	test_tree.addNode(test_node2);

	//Now the root node should still be test_node1, but the leaf node should be test_node2
	ASSERT_EQ(test_node1->location_.getVector4fMap(), test_tree.getRootNode()->location_.getVector4fMap());
	ASSERT_EQ(test_node2->location_.getVector4fMap(), test_tree.getLeafNode()->location_.getVector4fMap());
}

TEST(RRTCarrotTreeTest, testCopy)
{
	RRTCarrotTree test_tree;


	//Test Copying tree with nodes
	node_ptr_t test_node1(new RRTNode());
	test_node1->location_.x = 10;
	test_node1->location_.y = 0;
	test_node1->location_.z = 0;

	test_tree.addNode(test_node1);

	RRTCarrotTree test_copy_tree_rf(test_tree);
	ASSERT_EQ(1, test_copy_tree_rf.size());
	node_ptr_t result1 = test_copy_tree_rf.getRootNode();
	ASSERT_EQ(test_node1->location_.getVector4fMap(), result1->location_.getVector4fMap())<<PRINT_EXPECT_NODE(test_node1, result1);

	RRTCarrotTree test_copy_tree_ptr(&test_tree);
	ASSERT_EQ(1, test_copy_tree_ptr.size());
	node_ptr_t result2 = test_copy_tree_ptr.getRootNode();
	ASSERT_EQ(test_node1->location_.getVector4fMap(), result1->location_.getVector4fMap())<<PRINT_EXPECT_NODE(test_node1, result2);

	RRTCarrotTree test_copy_tree_eq = test_tree;
	ASSERT_EQ(1, test_copy_tree_eq.size());
	node_ptr_t result3 = test_copy_tree_eq.getRootNode();
	ASSERT_EQ(test_node1->location_.getVector4fMap(), result1->location_.getVector4fMap())<<PRINT_EXPECT_NODE(test_node1, result3);
}

TEST(RRTCarrotTreeTest, testNearestNeighbor)
{
	RRTCarrotTree test_tree;
	//The tree should be empty
	ASSERT_EQ(0, test_tree.size());

	//Push a series of nodes onto the tree
	node_ptr_t test_node1(new RRTNode());
	test_node1->location_.x = 0;
	test_node1->location_.y = 0;
	test_node1->location_.z = 0;

	node_ptr_t test_node2(new RRTNode());
	test_node2->location_.x = 10;
	test_node2->location_.y = 0;
	test_node2->location_.z = 0;

	node_ptr_t test_node3(new RRTNode());
	test_node3->location_.x = 0;
	test_node3->location_.y = 10;
	test_node3->location_.z = 0;

	test_tree.addNode(test_node2);
	test_tree.addNode(test_node3);
	test_tree.addNode(test_node1);


	//Find the nearest neighbor to a node at 1,1. Should be test_node1
	node_ptr_t test_node4(new RRTNode());
	test_node4->location_.x = 1;
	test_node4->location_.y = 1;
	test_node4->location_.z = 0;


	node_ptr_t result1 = test_tree.findNearestNeighbor(test_node4);
	EXPECT_EQ(test_node1->location_.getVector4fMap(), result1->location_.getVector4fMap())<<PRINT_EXPECT_NODE(test_node1, result1);

	//Find the nearest neighbor to a node at 11,1, should be test_node2
	test_node4->location_.x = 11;
	node_ptr_t result2 = test_tree.findNearestNeighbor(test_node4);
	EXPECT_EQ(test_node2->location_.getVector4fMap(), result2->location_.getVector4fMap())<<PRINT_EXPECT_NODE(test_node2, result2);

	//Find the nearest neighbor to a node at 5,11, should be test_node3
	test_node4->location_.x = 5;
	test_node4->location_.y = 11;
	node_ptr_t result3 = test_tree.findNearestNeighbor(test_node4);
	EXPECT_EQ(test_node3->location_.getVector4fMap(), result3->location_.getVector4fMap())<<PRINT_EXPECT_NODE(test_node3, result3);
}

class RRTCarrotTestFixture : public ::testing::Test
{
protected:
	RRTCarrotTestFixture()
{
		//Neded because RRTCarrot uses calls to ros::Time but ROS is not initialized in a test fixture
		ros::Time::init();
		this->origin_.x = 0;
		this->origin_.y = 0;
		this->origin_.z = 0;
		this->x_size_   = 50;
		this->y_size_   = 50;
		this->test_planner_ = new RRTCarrot();
}

	Point origin_;
	int   x_size_;
	int   y_size_;
	RRTCarrot* test_planner_;
};

TEST_F(RRTCarrotTestFixture, testSetDelta)
{
	ASSERT_TRUE(this->test_planner_->setCarrotDelta(10));
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

