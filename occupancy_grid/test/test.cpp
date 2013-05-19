/**
 * @file   test.cpp
 *
 * @date   May 14, 2013
 * @author parallels
 * @brief  \TODO
 */

/*
 * Copyright (c) 2013, RIVeR Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <gtest/gtest.h>
//************ LOCAL DEPENDANCIES ****************//
#include <occupancy_grid/MultiTraitOccupancyGrid.hpp>
//***********    NAMESPACES     ****************//
using namespace occupancy_grid;

class occ_test_fixture : public ::testing::Test
{
protected:
	std::vector<occupancy_grid::MultiTraitOccupancyGrid::trait_t> traits_;
	nav_msgs::MapMetaData info_;

	void SetUp()
	{
		ros::Time::init();
		traits_.push_back(utilities::CellTrait::UNKOWN);
		traits_.push_back(utilities::CellTrait::FREE_LOW_COST);
		traits_.push_back(utilities::CellTrait::FREE_HIGH_COST);
		traits_.push_back(utilities::CellTrait::OBSTACLE);
		traits_.push_back(utilities::CellTrait::TRAVERSED);
		traits_.push_back(utilities::CellTrait::GOAL);
		info_.height = 100;
		info_.width  = 200;
		info_.resolution = 0.25;
	}

};

TEST_F(occ_test_fixture, testBasicSetup)
{
	MultiTraitOccupancyGrid testGrid("test", traits_, utilities::CellTrait::UNKOWN, info_);
	ASSERT_EQ("test", testGrid.getFrameID());
	ASSERT_EQ(200, testGrid.getXSizeGrid());
	ASSERT_EQ(50, testGrid.getXSizeMeter());
	ASSERT_EQ(100, testGrid.getYSizeGrid());
	ASSERT_EQ(25, testGrid.getYSizeMeter());
	ASSERT_EQ(0.25, testGrid.getResolution());
	MultiTraitOccupancyGrid copyGrid(testGrid);
	ASSERT_EQ("test", copyGrid.getFrameID());
	ASSERT_EQ(200, copyGrid.getXSizeGrid());
	ASSERT_EQ(50, copyGrid.getXSizeMeter());
	ASSERT_EQ(100, copyGrid.getYSizeGrid());
	ASSERT_EQ(25, copyGrid.getYSizeMeter());
	ASSERT_EQ(0.25, copyGrid.getResolution());
}

TEST_F(occ_test_fixture, testSetGet)
{
	MultiTraitOccupancyGrid testGrid("test", traits_, utilities::CellTrait::UNKOWN, info_);
	ASSERT_EQ(utilities::CellTrait::UNKOWN, testGrid.getPointTrait((unsigned int)0,(unsigned int)0).getEnum());
	ASSERT_EQ(utilities::CellTrait::UNKOWN, testGrid.getPointTrait((unsigned int)34,(unsigned int)10).getEnum());
	ASSERT_EQ(utilities::CellTrait::UNKOWN, testGrid.getPointTrait(1.5,4.1).getEnum());
	testGrid.addPointTrait((unsigned int)10,(unsigned int)10, utilities::CellTrait::OBSTACLE);
	testGrid.addPointTrait((unsigned int)10,(unsigned int)10, utilities::CellTrait::OBSTACLE);
	ASSERT_EQ(utilities::CellTrait::OBSTACLE, testGrid.getPointTrait((unsigned int)10,(unsigned int)10).getEnum());
	MultiTraitOccupancyGrid copyGrid(testGrid);
	ASSERT_EQ(utilities::CellTrait::OBSTACLE, copyGrid.getPointTrait((unsigned int)10,(unsigned int)10).getEnum());
}

TEST_F(occ_test_fixture, testMessage)
{
	MultiTraitOccupancyGrid testGrid("test", traits_, utilities::CellTrait::UNKOWN, info_);
	testGrid.addPointTrait((unsigned int)10,(unsigned int)10, utilities::CellTrait::OBSTACLE);
	MultiTraitOccupancyGridMessage testMessage;
	testGrid.toROSMsg(testMessage);
	ASSERT_EQ("test", testMessage.header.frame_id);
	MultiTraitOccupancyGrid copyedGrid(testMessage);
	//Should still be unkown because it takes two sets to change the type
	ASSERT_EQ(utilities::CellTrait::UNKOWN, copyedGrid.getPointTrait((unsigned int)10,(unsigned int)10).getEnum());
	//If trait copying worked properly, this should tip the scales
	copyedGrid.addPointTrait((unsigned int)10,(unsigned int)10, utilities::CellTrait::OBSTACLE);
	ASSERT_EQ(utilities::CellTrait::OBSTACLE, copyedGrid.getPointTrait((unsigned int)10,(unsigned int)10).getEnum());


}


int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
