/**
 * @file   MultiTraitOccupancyGridHelpers.cpp
 *
 * @date   May 18, 2013
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
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/occupancy_grid/MultiTraitOccupancyGridHelpers.h>
#include <pcl_ros/transforms.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/Pose.h>
//***********    NAMESPACES     ****************//
namespace aero_path_planning
{

bool addPointCloudPatch(aero_path_planning::PointCloud& cloud, occupancy_grid::utilities::CellTrait trait, int confidence, occupancy_grid::MultiTraitOccupancyGrid& grid, tf::TransformListener& transformer_, std::string& error_message)
{
	aero_path_planning::PointCloud transformed_cloud;
	try
	{
		transformer_.waitForTransform(grid.getFrameID(), cloud.header.frame_id, grid.getCreationTime(), ros::Duration(1.0/10));
		pcl_ros::transformPointCloud(grid.getFrameID(), cloud, transformed_cloud, transformer_);
		BOOST_FOREACH(aero_path_planning::PointCloud::PointType& point, transformed_cloud)
		{
			geometry_msgs::PoseStamped point_poise;
			point_poise.header.frame_id = grid.getFrameID();
			point_poise.pose.position.x = point.x;
			point_poise.pose.position.y = point.y;
			aero_path_planning::pointToPose(point, point_poise.pose);
			try
			{
				grid.addPointTrait(point_poise.pose, trait, confidence);
			}
			catch(bool& e)
			{
				//do nothing, just means one of the points wasn't on the map
			}
		}
		return true;
	}
	catch(std::exception& e)
	{
		error_message = e.what();
		return false;
	}
	return false;
}

}

