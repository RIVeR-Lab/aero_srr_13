/**
 * @file   AStarCarrotTest.cpp
 *
 * @date   Apr 12, 2013
 * @author Adam Panzica
 * @brief  Tests for the AStarCarrot algorithm
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <gtest/gtest.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/planning_strategies/AStarCarrot.h>
//***********    NAMESPACES     ****************//


#define PRINT_NODE_STREAM(nodeptr) "[Location: ("<<nodeptr->getLocation().x<<","<<nodeptr->getLocation().y<<","<<nodeptr->getLocation().z<<"), Parent: "<<nodeptr->getParent()<<"]"
#define PRINT_EXPECT_NODE(expect,actual) "Expected to get "<<PRINT_NODE_STREAM(expect)<<", Actually got "<<PRINT_NODE_STREAM(actual)

namespace asu = aero_path_planning::astar_utilities;
namespace app = aero_path_planning;

class AStarCarrotTestFixture : public ::testing::Test
{
protected:
	typedef asu::AStarNode::AStarNodePtr AStarNodePtr;

	asu::cost_func     cf_;
	asu::huristic_func hf_;

	app::Point zero_point_;
	app::Point goal_point_;
	asu::AStarNode::AStarNodePtr test_node_zero_ ;

	void SetUp()
	{
		cf_ = boost::bind(asu::pt_cost, _1, _2);
		hf_ = boost::bind(asu::ed_huristic, _1, _2);
		zero_point_.x = 0;
		zero_point_.y = 0;
		zero_point_.z = 0;
		zero_point_.rgba = app::FREE_LOW_COST;

		goal_point_.x = 50;
		goal_point_.y = 50;
		goal_point_.z = 50;
		goal_point_.rgba = app::GOAL;

		test_node_zero_  = AStarNodePtr(new asu::AStarNode(zero_point_, goal_point_, AStarNodePtr(), cf_, hf_));
	}
};

//************************************ TEST AStarNode ***************************************************//

TEST_F(AStarCarrotTestFixture, testAStarNode)
{
	AStarNodePtr goal_node(new asu::AStarNode(goal_point_, goal_point_, test_node_zero_, cf_, hf_));

	ASSERT_FALSE(goal_node->sameLocation(test_node_zero_));
	ASSERT_EQ(goal_node->getLocation().getVector4fMap(), goal_point_.getVector4fMap());
	ASSERT_EQ(goal_node->getParent(), test_node_zero_)<<PRINT_EXPECT_NODE(test_node_zero_, goal_node->getParent());
	ASSERT_EQ(goal_node->getF(), test_node_zero_->getF());
	ASSERT_NE(goal_node->getG(), test_node_zero_->getG());
	ASSERT_NE(goal_node->getH(), test_node_zero_->getH());
	ASSERT_TRUE((*goal_node)==(*test_node_zero_));
}

