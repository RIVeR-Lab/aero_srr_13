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

/**
 * Basic test for operation of RRTNode to make sure it does what it should
 */
TEST(RRTNodeTest, testRRTNode)
{
	boost::shared_ptr<RRTNode> test_node(new RRTNode);
	test_node->location.x = 0;
	test_node->location.y = 0;
	test_node->location.z = 0;
	ASSERT_EQ(0, test_node->location.x);
	ASSERT_EQ(0, test_node->location.y);
	ASSERT_EQ(0, test_node->location.z);
	ASSERT_EQ(boost::shared_ptr<RRTNode>(), test_node->parent_);

	RRTNode test_node2;
	test_node2.location = test_node->location;
	test_node2.parent_  = test_node;

	ASSERT_EQ(test_node->location.getVector4fMap(), test_node2.location.getVector4fMap());
	ASSERT_EQ(test_node->location.getVector4fMap(), test_node2.parent_->location.getVector4fMap());
}

TEST(RRTCarrotTreeTest, testTreeSetup)
{
	int initial_tree_size = 0;
	RRTCarrotTree test_tree;

	//Size should be the initial guess capacity
	ASSERT_EQ(initial_tree_size, test_tree.size());

	//Test copy
	RRTCarrotTree test_copy_tree_rf(test_tree);
	ASSERT_EQ(initial_tree_size, test_copy_tree_rf.size());

	RRTCarrotTree test_copy_tree_ptr(&test_tree);
	ASSERT_EQ(initial_tree_size, test_copy_tree_ptr.size());
}

TEST(RRTCarrotTestTree, testAddingNode)
{
	int initial_tree_size = 0;
	RRTCarrotTree test_tree;

	//Push a node onto the tree
	node_ptr_t test_node1(new RRTNode());
	test_node1->location.x = 10;
	test_node1->location.y = 0;
	test_node1->location.z = 0;

	ASSERT_TRUE(test_tree.addNode(test_node1));

	//Tree size shouldn't have changed since we guessed a size of 10
	ASSERT_EQ(1, test_tree.size());
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
	test_node1->location.x = 10;
	test_node1->location.y = 0;
	test_node1->location.z = 0;

	test_tree.addNode(test_node1);

	//Now this node should be both root and leaf
	ASSERT_EQ(test_node1->location.getVector4fMap(), test_tree.getLeafNode()->location.getVector4fMap());
	ASSERT_EQ(test_node1->location.getVector4fMap(), test_tree.getRootNode()->location.getVector4fMap());
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

