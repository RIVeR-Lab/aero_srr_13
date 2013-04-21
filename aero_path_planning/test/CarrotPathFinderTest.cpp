/**
 * @file CarrotPathFinderTest.cpp
 *
 * @date   Feb 21, 2013
 * @author Adam Panzica
 * @brief Unit tests for any class which implmeents the aero_path_planning::CarrotPathFinder interface
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<gtest/gtest.h>
#include<boost/foreach.hpp>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/planning_strategies/CarrotPathFinder.h>
//**********************NAMESPACES*****************************//
using namespace aero_path_planning;

namespace aero_path_planning_test
{
typedef CarrotPathFinder* planner_ptr;

/*class CarrotTestFixture : public ::testing::Test
{
protected:
	CarrotTestFixture():
		test_map_(100,100, 1, buildOrigin())
{
}
	~CarrotTestFixture()
	{
		BOOST_FOREACH(planner_ptr planner, this->planner_list_)
				{
			delete planner;
				}
		this->planner_list_.clear();
	}

	void SetUp()
	{
		this->start_point_.x = 0;
		this->start_point_.y = 0;
		this->start_point_.z = 0;
		this->goal_point_.x  = 100;
		this->goal_point_.y  = 100;
		this->goal_point_.z  = 100;

		PointCloud obst1;
		Point obst1s;
		obst1s.x=10;
		obst1s.y=10;
		obst1s.z=10;
		Point obst1e;
		obst1e.x=obst1s.x+10;
		obst1e.y=obst1s.y+10;
		obst1e.z=obst1s.z;
		castLine(obst1s, obst1e, aero_path_planning::OBSTACLE, obst1);
		BOOST_FOREACH(Point point, obst1)
		{
			this->test_map_.setPointTrait(point, aero_path_planning::OBSTACLE);
		}
		//CarrotPathFinder* RRTCarrotPlanner = new RRTCarrot(1);
		//this->planner_list_.push_back(RRTCarrotPlanner);
	}

	std::vector<planner_ptr> planner_list_;
	Point                    origin_;
	Point                    start_point_;
	Point                    goal_point_;
	OccupancyGrid            test_map_;

	Point& buildOrigin()
	{
		this->origin_.x = 0;
		this->origin_.y = 0;
		this->origin_.z = 0;
		return this->origin_;
	}
};*/

//TEST_F(CarrotTestFixture, TestAcceptDelta)
//{
//	ASSERT_TRUE(true);
//	BOOST_FOREACH(CarrotPathFinder* planner, planner_list_)
//	{
//		//ASSERT_TRUE(planner->setCarrotDelta(5));
//	}
//}

TEST(SampleTest, sampleTest)
{
	ASSERT_TRUE(true);
}


int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

};
