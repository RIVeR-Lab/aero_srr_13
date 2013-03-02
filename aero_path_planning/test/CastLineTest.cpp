/**
 * @file CastLineTest.cpp
 *
 * @date   Feb 25, 2013
 * @author Adam Panzica
 * @brief  Tests for the castLine function
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<gtest/gtest.h>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/OccupancyGrid.h>
//**********************NAMESPACES*****************************//

#define PRINT_EXPECTED_POINT(expect, actual)PRINT_POINT_S("Expected Point", expect)<<", "<<PRINT_POINT_S("Actual Point", actual)

using namespace aero_path_planning;
TEST(CastLineTests, testCastLineZero)
{
	PointCloud test_line;
	Point      pls;
	Point      ple;
	pls.x = 0;
	pls.y = 0;
	pls.z = 0;
	ple.x = 0;
	ple.y = 0;
	ple.z = 0;

	aero_path_planning::castLine(pls, ple, 0, test_line);

	//The line from 0,0,0 to 0,0,0 should be of size 1 since it should just have a point at the start
	ASSERT_EQ(1, test_line.size());
	ASSERT_EQ(pls.getVector4fMap(), test_line.at(0).getVector4fMap());
}



TEST(CastLineTests, testCastLinePxPy)
{
	PointCloud test_line;
	Point      pls;
	Point      ple;
	pls.x = 0;
	pls.y = 0;
	pls.z = 0;
	ple.x = 10;
	ple.y = 10;
	ple.z = 0;

	aero_path_planning::castLine(pls, ple, 0, test_line);
	//There should be a line, and it should be greater than two points
	ASSERT_NE(0, test_line.size());
	ASSERT_LT(2, test_line.size());
	//The start and end points should match the given start/end points
	Point start_point = test_line.at(0);
	Point end_point   = test_line.at(test_line.size()-1);
	EXPECT_EQ(pls.getVector4fMap(), start_point.getVector4fMap())<<PRINT_EXPECTED_POINT(pls, start_point);
	EXPECT_EQ(ple.getVector4fMap(), end_point.getVector4fMap())  <<PRINT_EXPECTED_POINT(ple, end_point);
	EXPECT_NE(pls.getVector4fMap(), test_line.at(1).getVector4fMap());
	EXPECT_NE(ple.getVector4fMap(), test_line.at(1).getVector4fMap());
}

TEST(CastLineTest, testCastLinePxNy)
{
	PointCloud test_line;
	Point      pls;
	Point      ple;
	pls.x = 0;
	pls.y = 0;
	pls.z = 0;
	ple.x = 10;
	ple.y = -10;
	ple.z = 0;

	aero_path_planning::castLine(pls, ple, 0, test_line);
	//There should be a line
	ASSERT_NE(0, test_line.size());
	ASSERT_LT(2, test_line.size());
	//The start and end points should match the given start/end points
	Point start_point = test_line.at(0);
	Point end_point   = test_line.at(test_line.size()-1);
	EXPECT_EQ(pls.getVector4fMap(), start_point.getVector4fMap())<<PRINT_EXPECTED_POINT(pls, start_point);
	EXPECT_EQ(ple.getVector4fMap(), end_point.getVector4fMap())  <<PRINT_EXPECTED_POINT(ple, end_point);
	EXPECT_NE(pls.getVector4fMap(), test_line.at(1).getVector4fMap());
	EXPECT_NE(ple.getVector4fMap(), test_line.at(1).getVector4fMap());
}

TEST(CastLineTest, testCastLineNxPy)
{
	PointCloud test_line;
	Point      pls;
	Point      ple;
	pls.x = 0;
	pls.y = 0;
	pls.z = 0;
	ple.x = -10;
	ple.y = 10;
	ple.z = 0;

	aero_path_planning::castLine(pls, ple, 0, test_line);
	//There should be a line
	ASSERT_NE(0, test_line.size());
	ASSERT_LT(2, test_line.size());
	//The start and end points should match the given start/end points
	Point start_point = test_line.at(0);
	Point end_point   = test_line.at(test_line.size()-1);
	EXPECT_EQ(pls.getVector4fMap(), start_point.getVector4fMap())<<PRINT_EXPECTED_POINT(pls, start_point);
	EXPECT_EQ(ple.getVector4fMap(), end_point.getVector4fMap())  <<PRINT_EXPECTED_POINT(ple, end_point);
	EXPECT_NE(pls.getVector4fMap(), test_line.at(1).getVector4fMap());
	EXPECT_NE(ple.getVector4fMap(), test_line.at(1).getVector4fMap());
}

TEST(CastLineTest, testCastLineNxNy)
{
	PointCloud test_line;
	Point      pls;
	Point      ple;
	pls.x = 0;
	pls.y = 0;
	pls.z = 0;
	ple.x = -10;
	ple.y = -10;
	ple.z = 0;

	aero_path_planning::castLine(pls, ple, 0, test_line);
	//There should be a line
	ASSERT_NE(0, test_line.size());
	ASSERT_LT(2, test_line.size());
	//The start and end points should match the given start/end points
	Point start_point = test_line.at(0);
	Point end_point   = test_line.at(test_line.size()-1);
	EXPECT_EQ(pls.getVector4fMap(), start_point.getVector4fMap())<<PRINT_EXPECTED_POINT(pls, start_point);
	EXPECT_EQ(ple.getVector4fMap(), end_point.getVector4fMap())  <<PRINT_EXPECTED_POINT(ple, end_point);
	EXPECT_NE(pls.getVector4fMap(), test_line.at(1).getVector4fMap());
	EXPECT_NE(ple.getVector4fMap(), test_line.at(1).getVector4fMap());
}

