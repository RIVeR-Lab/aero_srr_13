/**
 * @file   AStarCarrotTest.cpp
 *
 * @date   Apr 12, 2013
 * @author Adam Panzica
 * @brief  Tests for the AStarCarrot algorithm
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <gtest/gtest.h>
#include<deque>
#include <occupancy_grid/MultiTraitOccupancyGrid.hpp>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/planning_strategies/AStarCarrot.h>
//***********    NAMESPACES     ****************//

#define PRINT_POINT_STREAM(point) "[Location: ("<<point.x()<<","<<point.y()<<","<<point.z()<<")]"
#define PRINT_EXPECT_POINT(expect,actual) "Expected to get "<<PRINT_POINT_STREAM(expect)<<", Actually got "<<PRINT_POINT_STREAM(actual)
#define PRINT_NODE_STREAM(nodeptr) "[Location: ("<<nodeptr->getLocation().x()<<","<<nodeptr->getLocation().y()<<","<<nodeptr->getLocation().z()<<"), Parent: "<<nodeptr->getParent()<<"]"
#define PRINT_EXPECT_NODE(expect,actual) "Expected to get "<<PRINT_NODE_STREAM(expect)<<", Actually got "<<PRINT_NODE_STREAM(actual)

namespace asu = aero_path_planning::astar_utilities;
namespace app = aero_path_planning;

template<class T>
void clear_queue(std::deque<T>& deque)
{
	while(!deque.empty())
	{
		deque.pop_front();
	}
}

class AStarCarrotTestFixture : public ::testing::Test
{
protected:
	typedef asu::AStarNode::AStarNodePtr AStarNodePtr;

	asu::cost_func     cf_;
	asu::huristic_func hf_;
	app::CarrotPathFinder::collision_func_ clf_;

	tf::Point zero_point_;
	tf::Point goal_point_;
	asu::AStarNode::AStarNodePtr test_node_zero_ ;

	std::vector<occupancy_grid::MultiTraitOccupancyGrid::trait_t> traits_;
	nav_msgs::MapMetaData info_;

	void SetUp()
	{
		cf_ = boost::bind(asu::pt_cost, _1, _2);
		hf_ = boost::bind(asu::ed_huristic, _1, _2);
		clf_= boost::bind(&AStarCarrotTestFixture::collisionCheck, this, _1, _2);
		zero_point_.setZero();


		goal_point_.setX(49.0*0.25);
		goal_point_.setY(49.0*0.25);
		goal_point_.setZ(0);
		goal_point_.setW(0);


		test_node_zero_  = AStarNodePtr(new asu::AStarNode(zero_point_, goal_point_, AStarNodePtr(), cf_, hf_));

		traits_.push_back(occupancy_grid::utilities::CellTrait::UNKOWN);
		traits_.push_back(occupancy_grid::utilities::CellTrait::FREE_LOW_COST);
		traits_.push_back(occupancy_grid::utilities::CellTrait::FREE_HIGH_COST);
		traits_.push_back(occupancy_grid::utilities::CellTrait::OBSTACLE);
		traits_.push_back(occupancy_grid::utilities::CellTrait::TRAVERSED);
		traits_.push_back(occupancy_grid::utilities::CellTrait::GOAL);
		info_.height = 50;
		info_.width  = 50;
		info_.resolution = 0.25;

		ros::Time::init();
	}

	bool collisionCheck(const tf::Point& point, const occupancy_grid::MultiTraitOccupancyGrid& map)
	{
		try
		{
			if(map.getPointTrait((unsigned int)point.x(), (unsigned int)point.y())==occupancy_grid::utilities::CellTrait::OBSTACLE)
			{
				return true;
			}
		}
		catch(bool& e)
		{
			return true;
		}
		return false;
	}
};

//************************************ TEST AStarNode ***************************************************//



TEST_F(AStarCarrotTestFixture, testAStarNode)
{
	tf::Point test_point1(1,0,0);
	test_point1.setX(1);
	test_point1.setY(0);
	test_point1.setZ(0);


	tf::Point test_point3(1,0,0);
	test_point3.setX(1);
	test_point3.setY(0);
	test_point3.setZ(0);

	EXPECT_TRUE(test_point1==test_point3);

	tf::Point test_point2(1,10,0);
	test_point2.setX(1);
	test_point2.setY(10);
	test_point2.setZ(0);


	AStarNodePtr first_node(new asu::AStarNode(test_point1, goal_point_, test_node_zero_, cf_, hf_));
	AStarNodePtr second_node(new asu::AStarNode(test_point2, goal_point_, first_node, cf_, hf_));
	AStarNodePtr goal_node(new asu::AStarNode(goal_point_, goal_point_, second_node, cf_, hf_));

	//First Node should have a cost of 1
	EXPECT_EQ(1, first_node->getG());

	//Second Node should have a cost of 11
	EXPECT_EQ(11, second_node->getG());

	EXPECT_FALSE(goal_node->sameLocation(test_node_zero_));
	EXPECT_EQ(goal_node->getLocation(), goal_point_);
	EXPECT_EQ(goal_node->getParent(), second_node)<<PRINT_EXPECT_NODE(first_node, goal_node->getParent());
	EXPECT_NE(goal_node->getG(), test_node_zero_->getG());
	EXPECT_NE(goal_node->getH(), test_node_zero_->getH());
	EXPECT_FALSE((*goal_node)==(*test_node_zero_));
}

//************************************ TEST HURISTIC FUNCTION ***************************************************//
TEST_F(AStarCarrotTestFixture, testHuristic)
{
	tf::Point test_point;
	test_point.setX(1);
	test_point.setY(0);
	test_point.setZ(0);

	EXPECT_EQ(1, hf_(zero_point_, test_point));

	test_point.setX(-1);
	EXPECT_EQ(1, hf_(zero_point_, test_point));

	test_point.setY(10);
	test_point.setX(0);
	EXPECT_EQ(10, hf_(zero_point_, test_point));
}

//************************************ TEST COST FUNCTION ***************************************************//
TEST_F(AStarCarrotTestFixture, testCost)
{
	tf::Point test_point;
	test_point.setX(1);

	EXPECT_EQ(1, cf_(test_point, zero_point_));

	EXPECT_EQ(1, cf_(test_point, zero_point_));

	EXPECT_EQ(1, cf_(test_point, zero_point_));
}

//************************************ TEST COLLISIONCHECK ***************************************************//

//TEST_F(AStarCarrotTestFixture, testCollision)
//{
//	tf::Point origin;
//	origin.setX(0);
//	origin.setY(0);
//	origin.setZ(0);
//
//	occupancy_grid::MultiTraitOccupancyGrid coll_grid("test", traits_, occupancy_grid::utilities::CellTrait::FREE_LOW_COST, info_);
//	tf::Point obst_point;
//	obst_point.setX(5);
//	obst_point.setY(5);
//	obst_point.setZ(0);
//	coll_grid.addPointTrait((unsigned int)obst_point.x(), (unsigned int)obst_point.y(), occupancy_grid::utilities::CellTrait::OBSTACLE, 1000);
//
//	EXPECT_TRUE(collisionCheck(obst_point, coll_grid));
//
//	tf::Point clear_point;
//	clear_point.setX(1);
//	clear_point.setY(1);
//	clear_point.setZ(0);
//	EXPECT_FALSE(collisionCheck(clear_point,coll_grid));
//
//	tf::Point off_map;
//	off_map.setX(100);
//	off_map.setY(-1);
//	EXPECT_TRUE(collisionCheck(off_map,coll_grid));
//}

//************************************ TEST AStarCarrot ***************************************************//

TEST_F(AStarCarrotTestFixture, testAStarCarrot)
{
	std::string frame_id("/robot");
	occupancy_grid::MultiTraitOccupancyGridPtr test_map(new occupancy_grid::MultiTraitOccupancyGrid(frame_id, traits_, occupancy_grid::utilities::CellTrait::FREE_LOW_COST, info_));
	app::AStarCarrot test_planner;
	std::deque<geometry_msgs::Pose> result_path;
	ros::Duration timeout(10);

	geometry_msgs::Pose zero_pose;
	geometry_msgs::Pose goal_pose;
	tf::pointTFToMsg(zero_point_, zero_pose.position);
	tf::pointTFToMsg(goal_point_, goal_pose.position);
	EXPECT_FALSE(test_planner.search(zero_pose, goal_pose, timeout , result_path));
	EXPECT_FALSE(test_planner.allowsPartialPath());
	EXPECT_TRUE(test_planner.setCarrotDelta(5));
	EXPECT_TRUE(test_planner.setCollision(clf_));
	EXPECT_TRUE(test_planner.setSearchMap(test_map));
//
//	//Search an empty map. Should create a straight path from start to goal
	clear_queue<geometry_msgs::Pose>(result_path);
	EXPECT_TRUE(test_planner.search(zero_pose, goal_pose, timeout , result_path));
	EXPECT_TRUE(result_path.size()>0);
	tf::Point starting_point;
	tf::pointMsgToTF(result_path.front().position, starting_point);
	EXPECT_EQ(zero_point_, starting_point)<<PRINT_EXPECT_POINT(zero_point_, starting_point);
}
