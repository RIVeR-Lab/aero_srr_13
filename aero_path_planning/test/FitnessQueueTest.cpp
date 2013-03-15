/**
 * @file FitnessQueueTest.cpp
 *
 * @date Mar 9, 2013
 * @author aruis
 */

//********************** SYSTEM DEPENDANCIES **********************//
#include<gtest/gtest.h>
//********************** LOCAL  DEPENDANCIES **********************//
#include<aero_path_planning/OryxPathPlanning.h>
#include<aero_path_planning/FitnessQueue.h>
using namespace aero_path_planning;
TEST(FitnessQueueTests, testConstructor)
{
	//Test making FitnessQueues of various types
	FitnessQueue<int, int> int_queue;
	FitnessQueue<double, double> double_queue;
	FitnessQueue<double, Point> point_queue;

	ASSERT_EQ(0, int_queue.size());
	ASSERT_EQ(0, double_queue.size());
	ASSERT_EQ(0, double_queue.size());

}

TEST(FitnessQueueTests, testPush)
{
	//Test making FitnessQueues of various types and adding elements to them
	FitnessQueue<int, int> int_queue;
	FitnessQueue<double, double> double_queue;
	FitnessQueue<double, Point> point_queue;

	int_queue.push(1,  10);
	int_queue.push(7,  7);
	int_queue.push(-1, 100);
	ASSERT_EQ(3, int_queue.size());

	double_queue.push(1.7,  -4.5);
	double_queue.push(7.0,   10.1);
	double_queue.push(-1.56, 10);
	ASSERT_EQ(3, double_queue.size());

	Point testPoint;
	point_queue.push(1.7,   testPoint);
	point_queue.push(7.0,   testPoint);
	point_queue.push(-1.56, testPoint);
	ASSERT_EQ(3, point_queue.size());
}

TEST(FitnessQueueTests, testTopPop)
{
	//Test queues
	FitnessQueue<int, int> int_queue;
	FitnessQueue<double, double> double_queue;
	FitnessQueue<double, Point> point_queue;
	//Add elements
	int_queue.push(1,  10);
	int_queue.push(7,  7);
	int_queue.push(-1, 100);
	ASSERT_EQ(3, int_queue.size());

	double_queue.push(1.7,  -4.5);
	double_queue.push(7.0,   10.1);
	double_queue.push(-1.56, 10);
	ASSERT_EQ(3, double_queue.size());

	//Set test point x's so we can tell them apart
	Point point1;
	Point point2;
	Point point3;
	point1.x = 5;
	point2.x = 7;
	point3.x = 10;

	point_queue.push(1.7,   point1);
	point_queue.push(7.0,   point2);
	point_queue.push(-1.56, point3);
	ASSERT_EQ(3, point_queue.size());

	//Check the top elements
	ASSERT_EQ(7, int_queue.top());
	ASSERT_EQ(10.1, double_queue.top());
	ASSERT_EQ(7, point_queue.top().x);

	//Pop elements
	int_queue.pop();
	double_queue.pop();
	point_queue.pop();

	ASSERT_EQ(10, int_queue.top());
	ASSERT_EQ(-4.5, double_queue.top());
	ASSERT_EQ(5, point_queue.top().x);
}



