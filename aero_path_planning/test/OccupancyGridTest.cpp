/**
 * @file RRTCarrotTest.cpp
 *
 * @date   Feb 23, 2013
 * @author Adam Panzica
 * @brief Test Set for OccupancyGrid
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<gtest/gtest.h>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/OccupancyGrid.h>
//**********************NAMESPACES*****************************//
using namespace aero_path_planning;

 class OccupancyGridTest:public ::testing::Test
 {
 protected:
 	virtual void SetUp()
 	{
 		res_       = .1;

 		x_size_    = 50;
 		y_size_    = 50;
 		z_size_    = 0;

 		zero_.x    = 0;
 		zero_.y    = 0;
 		zero_.z    = 0;
 		zero_.rgba = 0;

 		pxpy_.x    = 10;
 		pxpy_.y    = 5;
 		pxpy_.z    = 0;
 		pxpy_.rgba = aero_path_planning::OBSTACLE;

 		pxny_.x    = 5;
 		pxny_.y    = -10;
 		pxny_.z    = 0;
 		pxny_.rgba = aero_path_planning::TENTACLE;

 		nxny_.x    = -10;
 		nxny_.y    = -5;
 		nxny_.z    = 0;
 		nxny_.rgba = aero_path_planning::FREE_LOW_COST;

 		nxpy_.x    = -5;
 		nxpy_.y    = 10;
 		nxpy_.z    = 0;
 		nxpy_.rgba = aero_path_planning::FREE_HIGH_COST;
 	}

 	double res_;

 	int x_size_;
 	int y_size_;
 	int z_size_;

 	Point zero_;
 	Point pxpy_;
 	Point nxny_;
 	Point pxny_;
 	Point nxpy_;

 };

 TEST_F(OccupancyGridTest, testBasicSetup)
 {
 	//Test empty constructor
 	OccupancyGrid testGrid;
 	ASSERT_EQ(0, testGrid.size());

 	//Test 2D constructor
 	OccupancyGrid testGrid2D(x_size_, y_size_, res_, zero_);
 	ASSERT_EQ(x_size_        , testGrid2D.getXSize());
 	ASSERT_EQ(y_size_        , testGrid2D.getYSize());
 	ASSERT_EQ(z_size_        , testGrid2D.getZSize());
 	ASSERT_EQ(x_size_*y_size_, testGrid2D.size());
 }