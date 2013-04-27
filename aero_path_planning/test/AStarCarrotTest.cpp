/**
 * @file   AStarCarrotTest.cpp
 *
 * @date   Apr 12, 2013
 * @author Adam Panzica
 * @brief  Tests for the AStarCarrot algorithm
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <gtest/gtest.h>
#include<queue>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/planning_strategies/AStarCarrot.h>
//***********    NAMESPACES     ****************//

#define PRINT_POINT_STREAM(point) "[Location: ("<<point.x<<","<<point.y<<","<<point.z<<")]"
#define PRINT_EXPECT_POINT(expect,actual) "Expected to get "<<PRINT_POINT_STREAM(expect)<<", Actually got "<<PRINT_POINT_STREAM(actual)
#define PRINT_NODE_STREAM(nodeptr) "[Location: ("<<nodeptr->getLocation().x<<","<<nodeptr->getLocation().y<<","<<nodeptr->getLocation().z<<"), Parent: "<<nodeptr->getParent()<<"]"
#define PRINT_EXPECT_NODE(expect,actual) "Expected to get "<<PRINT_NODE_STREAM(expect)<<", Actually got "<<PRINT_NODE_STREAM(actual)

namespace asu = aero_path_planning::astar_utilities;
namespace app = aero_path_planning;

template<class T>
void clear_queue(std::queue<T>& queue)
{
	while(!queue.empty())
	{
		queue.pop();
	}
}

class AStarCarrotTestFixture : public ::testing::Test
{
protected:
	typedef asu::AStarNode::AStarNodePtr AStarNodePtr;

	asu::cost_func     cf_;
	asu::huristic_func hf_;
	app::CarrotPathFinder::collision_func_ clf_;

	app::Point zero_point_;
	app::Point goal_point_;
	asu::AStarNode::AStarNodePtr test_node_zero_ ;

	void SetUp()
	{
		cf_ = boost::bind(asu::pt_cost, _1, _2);
		hf_ = boost::bind(asu::ed_huristic, _1, _2);
		clf_= boost::bind(&AStarCarrotTestFixture::collisionCheck, this, _1, _2);
		zero_point_.x = 0;
		zero_point_.y = 0;
		zero_point_.z = 0;
		zero_point_.rgba = app::FREE_LOW_COST;

		goal_point_.x = 49;
		goal_point_.y = 49;
		goal_point_.z = 0;
		goal_point_.rgba = app::GOAL;

		test_node_zero_  = AStarNodePtr(new asu::AStarNode(zero_point_, goal_point_, AStarNodePtr(), cf_, hf_));

		ros::Time::init();
	}

	bool collisionCheck(const aero_path_planning::Point& point, const aero_path_planning::OccupancyGrid& map)
	{
		try
		{
			if(map.getPointTrait(point)==aero_path_planning::OBSTACLE)
			{
				return true;
			}
		}
		catch(std::exception& e)
		{
			return true;
		}
		return false;
	}
};

//************************************ TEST AStarNode ***************************************************//



TEST_F(AStarCarrotTestFixture, testAStarNode)
{
	app::Point test_point1;
	test_point1.x = 1;
	test_point1.y = 0;
	test_point1.z = 0;
	test_point1.rgba = app::FREE_LOW_COST;

	app::Point test_point3;
	test_point3.x = 1;
	test_point3.y = 0;

	EXPECT_TRUE(test_point1.getVector4fMap()==test_point3.getVector4fMap());

	app::Point test_point2;
	test_point2.x = 1;
	test_point2.y = 10;
	test_point2.z = 0;
	test_point2.rgba = app::FREE_LOW_COST;

	AStarNodePtr first_node(new asu::AStarNode(test_point1, goal_point_, test_node_zero_, cf_, hf_));
	AStarNodePtr second_node(new asu::AStarNode(test_point2, goal_point_, first_node, cf_, hf_));
	AStarNodePtr goal_node(new asu::AStarNode(goal_point_, goal_point_, second_node, cf_, hf_));

	//First Node should have a cost of 1
	EXPECT_EQ(1, first_node->getG());

	//Second Node should have a cost of 11
	EXPECT_EQ(11, second_node->getG());

	EXPECT_FALSE(goal_node->sameLocation(test_node_zero_));
	EXPECT_EQ(goal_node->getLocation().getVector4fMap(), goal_point_.getVector4fMap());
	EXPECT_EQ(goal_node->getParent(), second_node)<<PRINT_EXPECT_NODE(first_node, goal_node->getParent());
	EXPECT_NE(goal_node->getG(), test_node_zero_->getG());
	EXPECT_NE(goal_node->getH(), test_node_zero_->getH());
	EXPECT_FALSE((*goal_node)==(*test_node_zero_));
}

//************************************ TEST HURISTIC FUNCTION ***************************************************//
TEST_F(AStarCarrotTestFixture, testHuristic)
{
	app::Point test_point;
	test_point.x = 1;
	test_point.y = 0;
	test_point.z = 0;
	test_point.rgba = app::FREE_LOW_COST;

	EXPECT_EQ(1, hf_(zero_point_, test_point));

	test_point.x = -1;
	EXPECT_EQ(1, hf_(zero_point_, test_point));

	test_point.y = 10;
	test_point.x = 0;
	EXPECT_EQ(10, hf_(zero_point_, test_point));
}

//************************************ TEST COST FUNCTION ***************************************************//
TEST_F(AStarCarrotTestFixture, testCost)
{
	app::Point test_point;
	test_point.x = 1;
	test_point.rgba = app::FREE_LOW_COST;

	EXPECT_EQ(1, cf_(test_point, zero_point_));

	test_point.rgba = app::FREE_HIGH_COST;
	EXPECT_EQ(2, cf_(test_point, zero_point_));

	test_point.rgba = app::TRAVERSED;
	EXPECT_EQ(1.25, cf_(test_point, zero_point_));
}

//************************************ TEST COLLISIONCHECK ***************************************************//

TEST_F(AStarCarrotTestFixture, testCollision)
{
	app::Point origin;
	origin.x=0;
	origin.y=0;
	origin.z=0;
	app::OccupancyGrid coll_grid(10,10,1,origin);
	app::Point obst_point;
	obst_point.x = 5;
	obst_point.y = 5;
	obst_point.z = 0;
	coll_grid.setPointTrait(obst_point, aero_path_planning::OBSTACLE);

	EXPECT_TRUE(collisionCheck(obst_point, coll_grid));

	app::Point clear_point;
	clear_point.x=1;
	clear_point.y=1;
	clear_point.z=0;
	EXPECT_FALSE(collisionCheck(clear_point,coll_grid));

	app::Point off_map;
	off_map.x = 100;
	off_map.y = -1;
	EXPECT_TRUE(collisionCheck(off_map,coll_grid));
}

//************************************ TEST AStarCarrot ***************************************************//

TEST_F(AStarCarrotTestFixture, testAStarCarrot)
{
	std::string frame_id("/robot");
	app::OccupancyGrid test_map(50, 50, .1, zero_point_, app::FREE_LOW_COST, frame_id);
	app::AStarCarrot test_planner;
	std::queue<app::Point> result_path;
	ros::Duration timeout(10);

	EXPECT_FALSE(test_planner.search(zero_point_, goal_point_, timeout , result_path));
	EXPECT_FALSE(test_planner.allowsPartialPath());

	EXPECT_TRUE(test_planner.setCarrotDelta(5));
	EXPECT_TRUE(test_planner.setCollision(clf_));
	EXPECT_TRUE(test_planner.setSearchMap(test_map));

	//Search an empty map. Should create a straight path from start to goal
	clear_queue<app::Point>(result_path);
	EXPECT_TRUE(test_planner.search(zero_point_, goal_point_, timeout , result_path));
	EXPECT_TRUE(result_path.size()>0);
	EXPECT_EQ(zero_point_.getVector4fMap(), result_path.front().getVector4fMap())<<PRINT_EXPECT_POINT(zero_point_, result_path.front());
}
