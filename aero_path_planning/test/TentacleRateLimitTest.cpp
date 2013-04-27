/**
 * @file   TentacleRateLimitTest.cpp
 *
 * @date   Apr 26, 2013
 * @author Adam Panzica
 * @brief  Test for TentacleRateLimiter class
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <gtest/gtest.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/utilities/TentacleRateLimiter.h>
//***********    NAMESPACES     ****************//

using namespace aero_path_planning;

TEST(TentacleRateLimiterTests, testBasics)
{
	TentacleRateLimiter limiter(81, 5);
	ASSERT_EQ(5, limiter.getRateLimit());
	limiter.setRateLimit(10);
	ASSERT_EQ(10, limiter.getRateLimit());
	limiter.disableLimit(true);
	ASSERT_EQ(1, limiter.nextTentacle(1));
	ASSERT_EQ(10, limiter.nextTentacle(10));
}

TEST(TentacleRateLimiterTests, testSameSide)
{
	//Test tentacles on the left side
	TentacleRateLimiter left_limiter(81, 5);
	//Set the inital tentacle
	ASSERT_EQ(10, left_limiter.nextTentacle(10));
	//Check for a change under the rate limit in positive direction
	ASSERT_EQ(11, left_limiter.nextTentacle(11));
	//Check for a change under the rate limit in the negative direction
	ASSERT_EQ(10, left_limiter.nextTentacle(10));
	//Check for a change over the rate limit in the positive direction
	ASSERT_EQ(15, left_limiter.nextTentacle(30));
	//Check for a change over the rate limit in the negative direction
	ASSERT_EQ(10, left_limiter.nextTentacle(5));


	//Test tentacles on the right side
	TentacleRateLimiter right_limiter(81, 5);
	//Set the inital tentacle
	ASSERT_EQ(50, right_limiter.nextTentacle(50));
	//Check for a change under the rate limit in positive direction
	ASSERT_EQ(51, right_limiter.nextTentacle(51));
	//Check for a change under the rate limit in the negative direction
	ASSERT_EQ(50, right_limiter.nextTentacle(50));
	//Check for a change right_limiter the rate limit in the positive direction
	ASSERT_EQ(55, right_limiter.nextTentacle(70));
	//Check for a change over the rate limit in the negative direction
	ASSERT_EQ(50, right_limiter.nextTentacle(45));
}

TEST(TentacleRateLimiterTests, testCrossing)
{
	TentacleRateLimiter limiter(81, 5);
	//Set the inital tentacle
	ASSERT_EQ(39, limiter.nextTentacle(39));
	//Test Left->Right cross below rate limit
	ASSERT_EQ(80, limiter.nextTentacle(80));
	//Test Right->Left cross below rate limit
	ASSERT_EQ(39, limiter.nextTentacle(39));
	ASSERT_EQ(34, limiter.nextTentacle(20));
	//Test Requested Left->Right cross that when rate-limited doesn't cross broundry
	ASSERT_EQ(39, limiter.nextTentacle(80));
	//Test Left->Right cross that when rate limited crosses broundry
	ASSERT_EQ(78, limiter.nextTentacle(45));
	ASSERT_EQ(73, limiter.nextTentacle(73));
	//Test Right->Left cross that when rate limited doesn't cross boundry
	ASSERT_EQ(78, limiter.nextTentacle(30));
	//Test Right->Left cross that when rate limited does cross boundry
	ASSERT_EQ(39, limiter.nextTentacle(30));

}
