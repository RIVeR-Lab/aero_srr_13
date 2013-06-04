/**
 * @file   OccupancyGrid.cpp
 *
 * @date   May 7, 2013
 * @author Adam Panzica
 * @brief  Implementation of the occupancy_grid namespace
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
#include <boost/foreach.hpp>
//************ LOCAL DEPENDANCIES ****************//
#include <occupancy_grid/MultiTraitOccupancyGrid.hpp>
//***********    NAMESPACES     ****************//

using namespace occupancy_grid;
using namespace occupancy_grid::utilities;

//***************************************** CELLTRAIT *********************************************************//

CellTrait::CellTrait():
				enum_(UNKOWN)
{

}

CellTrait::CellTrait(Enum value):
				enum_(value)
{

}

CellTrait::CellTrait(int value)
{
	this->enum_ = enumFromValue(value);
}


CellTrait::Enum CellTrait::getEnum(void) const
{
	return this->enum_;
}

std::string CellTrait::getString(void) const
{
	return stringFromEnum(this->enum_);
}

int CellTrait::getValue(void) const
{
	return this->enum_;
}


std::ostream& CellTrait::operator<<(std::ostream& out) const
{
	out<<stringFromEnum(this->enum_);
	return out;
}

CellTrait& CellTrait::operator=(const CellTrait& rhs)
{
	this->enum_ = rhs.enum_;
	return *this;
}
CellTrait& CellTrait::operator=(const int& rhs)
{
	this->enum_ = enumFromValue(rhs);
	return *this;
}

bool CellTrait::operator==(const CellTrait& rhs) const
				{
	return rhs.enum_ == this->enum_;
				}
bool CellTrait::operator==(const int& rhs) const
				{
	return this->enum_ == enumFromValue(rhs);
				}

bool CellTrait::operator==(const Enum& rhs) const
				{
	return this->enum_ == rhs;
				}

CellTrait::Enum CellTrait::enumFromValue(int value)
{
	switch(value)
	{
	case FREE_LOW_COST:
		return FREE_LOW_COST;
		break;
	case FREE_HIGH_COST:
		return FREE_HIGH_COST;
		break;
	case OBSTACLE:
		return OBSTACLE;
		break;
	case GOAL:
		return GOAL;
		break;
	case TRAVERSED:
		return TRAVERSED;
		break;
	default:
		return  UNKOWN;
		break;
	}
	return UNKOWN;
}

std::string CellTrait::stringFromEnum(Enum value)
{
	switch(value)
	{
	case FREE_LOW_COST:
		return "FREE_LOW_COST";
		break;
	case FREE_HIGH_COST:
		return "FREE_HIGH_COST";
		break;
	case OBSTACLE:
		return "OBSTACLE";
		break;
	case GOAL:
		return "GOAL";
		break;
	case TRAVERSED:
		return "TRAVERSED";
		break;
	default:
		return  "UNKOWN";
		break;
	}
	return "UNKOWN";
}

//***************************** OCCUPANCYGRID **************************************//
MultiTraitOccupancyGrid::MultiTraitOccupancyGrid():temp_cell_values_(NULL),has_goal_(false), x_offset_(0), y_offset_(0){};

MultiTraitOccupancyGrid::MultiTraitOccupancyGrid(const MultiTraitOccupancyGrid& copy):
		grid_(copy.grid_),
		map_meta_data_(copy.map_meta_data_),
		trait_map_(copy.trait_map_),
		index_map_(copy.index_map_),
		frame_id_(copy.frame_id_),
		temp_cell_values_(new cell_data_t[copy.trait_map_.size()]),
		goal_pose_(copy.goal_pose_),
		has_goal_(copy.has_goal_),
		x_offset_(copy.x_offset_),
		y_offset_(copy.y_offset_)
{

}

MultiTraitOccupancyGrid::MultiTraitOccupancyGrid(const MultiTraitOccupancyGridMessage& message):
						map_meta_data_(message.trait_grids.at(0).info),
						frame_id_(message.header.frame_id),
						temp_cell_values_(new cell_data_t[message.trait_vector.size()]),
						x_offset_(message.x_offset),
						y_offset_(message.y_offset)
{
	this->grid_ = message.trait_grids;
	this->goal_pose_ = message.goal;
	this->has_goal_  = message.has_goal;
	for(unsigned int i=0; i<message.trait_vector.size(); i++)
	{
		CellTrait trait(message.trait_vector.at(i));
		//ROS_INFO_STREAM("Loading Trait "<<trait.getString()<<" to vector index "<<i+1);
		this->trait_map_[trait.getEnum()] = i+1;
		this->index_map_[i+1] = trait.getEnum();
	}
}


MultiTraitOccupancyGrid::MultiTraitOccupancyGrid(const std::string& frame_id, const std::vector<trait_t>& traits, trait_t initial_trait, const nav_msgs::MapMetaData& slice_info, int x_offset, int y_offset):
						map_meta_data_(slice_info),
						frame_id_(frame_id),
						temp_cell_values_(new cell_data_t[traits.size()]),
						x_offset_(x_offset),
						y_offset_(y_offset)
{
	this->goal_pose_.orientation.w = 1;
	this->has_goal_ = false;
	//Initialize the trait vector
	this->grid_ = grid_slice_t(traits.size()+1);
	BOOST_FOREACH(grid_slice_t::value_type& grid_trait, this->grid_)
	{
		grid_trait.info = this->map_meta_data_;
		buildEmptyOccupancyGrid(grid_trait);
	}

	//Build mapping between a grid in the trait vector and a trait type and vice-versa
	for(unsigned int i=1; i<traits.size()+1; i++)
	{
		this->trait_map_[traits.at(i-1).getEnum()] = i;
		this->index_map_[i] = traits.at(i-1).getEnum();
	}

	//Fill in the initial confidance type
	for(unsigned int x=0; x<this->map_meta_data_.width; x++)
	{
		for(unsigned int y=0; y<this->map_meta_data_.height; y++)
		{
			int cell_index = ogu::calcIndexRowMajor2D(x, y, this->map_meta_data_.width);
			this->grid_.at(this->trait_map_[initial_trait.getEnum()]).data[cell_index] = 100;
			this->grid_.at(0).data.at(cell_index) = initial_trait.getEnum();
		}
	}
}

MultiTraitOccupancyGrid::~MultiTraitOccupancyGrid()
{
	delete[] this->temp_cell_values_;
}


unsigned int MultiTraitOccupancyGrid::getXSizeGrid() const
{
	return this->map_meta_data_.width;
}

double MultiTraitOccupancyGrid::getXSizeMeter() const
{
	return this->map_meta_data_.width*this->map_meta_data_.resolution;
}

unsigned int MultiTraitOccupancyGrid::getYSizeGrid() const
{
	return this->map_meta_data_.height;
}

double MultiTraitOccupancyGrid::getYSizeMeter() const
{
	return this->map_meta_data_.height*this->map_meta_data_.resolution;
}

double MultiTraitOccupancyGrid::getResolution() const
{
	return this->map_meta_data_.resolution;
}

int MultiTraitOccupancyGrid::getXOffsetGrid() const
{
	return this->x_offset_;
}

int MultiTraitOccupancyGrid::getYOffsetGrid() const
{
	return this->y_offset_;
}

geometry_msgs::Pose MultiTraitOccupancyGrid::getOrigin() const
{
	return this->map_meta_data_.origin;
}

std::string MultiTraitOccupancyGrid::getFrameID() const
{
	return this->frame_id_;
}

MultiTraitOccupancyGrid::trait_t MultiTraitOccupancyGrid::getPointTrait(int x, int y, bool offset_adjust) const  throw (bool)
{
	if(offset_adjust)
	{
		this->offsetAdjust(x, y);
	}
	if(this->boundsCheck(x, y))
	{
		int index = ogu::calcIndexRowMajor2D(x, y, this->map_meta_data_.width);
		return this->grid_.at(0).data[index];
	}
	else throw false;
}

MultiTraitOccupancyGrid::trait_t MultiTraitOccupancyGrid::getPointTrait(double x, double y) const  throw (bool)
{
	return this->getPointTrait((int)(x/this->map_meta_data_.resolution), (int)(y/this->map_meta_data_.resolution));
}

MultiTraitOccupancyGrid::trait_t MultiTraitOccupancyGrid::getPointTrait(const gm::Pose& point) const  throw (bool)
{
	int tx;
	int ty;
	this->projectPoseToGrid(point, tx, ty);
	return this->getPointTrait(tx, ty, false);
}

void MultiTraitOccupancyGrid::addPointTrait(const gm::Pose& point, trait_t trait, int confidence)  throw (bool)
{
	int tx;
	int ty;
	this->projectPoseToGrid(point, tx, ty);
	//ROS_INFO_STREAM("I'm placing a point at location:"<<tx<<","<<ty<<" of type: "<<trait.getString()<<" with confidance:"<<confidence);
	this->addPointTrait(tx, ty, trait, confidence, false);
}

void MultiTraitOccupancyGrid::addPointTrait(double x, double y, trait_t trait, int confidence)  throw (bool)
{
	this->addPointTrait((int)(x/this->map_meta_data_.resolution), (int)(y/this->map_meta_data_.resolution), trait, confidence);
}

void MultiTraitOccupancyGrid::addPointTrait(int x, int y, trait_t trait, int confidence, bool offset_adjust)  throw (bool)
{
	if(offset_adjust)
	{
		this->offsetAdjust(x, y);
	}
	if(this->boundsCheck(x, y))
	{
		int cell_index = ogu::calcIndexRowMajor2D(x, y, this->map_meta_data_.width);
		//Fill the temp values
		BOOST_FOREACH(trait_map_t::value_type trait, this->trait_map_)
		{
			this->temp_cell_values_[trait.second-1] = this->grid_.at(trait.second).data[cell_index];
		}
		//Increase the confidence of the requested trait
		this->temp_cell_values_[this->trait_map_[trait.getEnum()]-1] += confidence;

		//Normalize the confidence values
		normalizeConfidance(this->temp_cell_values_, this->grid_.size()-1, this->temp_cell_values_);

		//Write back the values to the grid, update the current 'best' choice
		int best_trait_index = -1;
		cell_data_t best_trait_value = 0;
		for(unsigned int i=0; i<this->grid_.size()-1; i++)
		{
			if(this->temp_cell_values_[i]>best_trait_value)
			{
				best_trait_index = i;
				best_trait_value = this->temp_cell_values_[i];
			}
			this->grid_.at(i+1).data[cell_index] = this->temp_cell_values_[i];
		}
		this->grid_.at(0).data[cell_index] = this->index_map_[best_trait_index+1];
	}
	else throw false;
}

void MultiTraitOccupancyGrid::toROSMsg(MultiTraitOccupancyGridMessage& message) const
{
	message.trait_grids     = this->grid_;
	message.header.frame_id = this->frame_id_;
	message.header.stamp    = ros::Time::now();
	message.trait_vector    = std::vector<int32_t>(this->trait_map_.size());
	message.x_offset        = this->x_offset_;
	message.y_offset        = this->y_offset_;
	BOOST_FOREACH(trait_map_t::value_type trait_index, this->trait_map_)
	{
		CellTrait trait(trait_index.first);
		//ROS_INFO_STREAM("Copying Trait "<<trait.getString()<<" to vector index "<<trait_index.second-1);
		message.trait_vector.at(trait_index.second-1) = trait_index.first;
	}
}

void MultiTraitOccupancyGrid::toROSMsg(trait_t trait, nm::OccupancyGrid& message) const
{
	message.header.frame_id = this->frame_id_;
	message.header.stamp    = ros::Time::now();
	message.info            = this->map_meta_data_;
	message.data            = this->grid_.at(this->trait_map_.at(trait.getEnum())).data;
}

void MultiTraitOccupancyGrid::addPointTrait(const nm::OccupancyGrid& confidances, trait_t trait, bool scaling, bool use_zero_as_free, bool use_negative_as_unkown)  throw (bool)
{
	//ROS_INFO_STREAM("Copying Occupancy Grid into MultiTraitOccupancyGrid....");
	unsigned int copy_width  = confidances.info.width;
	unsigned int copy_height = confidances.info.height;
	if(copy_width > this->map_meta_data_.width)
	{
		copy_width = this->map_meta_data_.width;
	}
	if(copy_height > this->map_meta_data_.height)
	{
		copy_height = this->map_meta_data_.height;
	}
	//ROS_INFO_STREAM("Copy Wdith: "<<copy_width<<", Copy Height: "<<copy_height);
	for(int x = 0; x < (int)copy_width; x++)
	{
		for(int y = 0; y < (int)copy_height; y++)
		{
			int copy_confidence = confidances.data[ogu::calcIndexRowMajor2D(x, y, confidances.info.width)];
			CellTrait copy_trait(trait);
			if(use_zero_as_free)
			{
				if(copy_confidence == 0)
				{
					copy_trait      = CellTrait::FREE_LOW_COST;
					copy_confidence = 1000;
				}
			}
			if(use_negative_as_unkown)
			{
				if(copy_confidence<0)
				{
					copy_trait      = CellTrait::UNKOWN;
					copy_confidence = 30;
				}
			}
//			if(trait == trait_t::OBSTACLE)
//			{
//				if(copy_confidence == 100)
//				{
//					copy_trait      = CellTrait::OBSTACLE;
//					copy_confidence = 1000;
//				}
//			}
			try
			{
				double raw_x = (double)x*confidances.info.resolution;
				double raw_y = (double)y*confidances.info.resolution;
				int scale_x  = (int)(raw_x/this->map_meta_data_.resolution);
				int scale_y  = (int)(raw_y/this->map_meta_data_.resolution);
				this->addPointTrait(scale_x, scale_y, copy_trait, copy_confidence, false);
			}
			catch(bool& e)
			{
				ROS_WARN_STREAM("Faild to Copy Point Into Grid at x="<<x<<","<<y);
			}
		}
	}
	//ROS_INFO_STREAM("Finished Copying Data Into Gird!");
}

bool MultiTraitOccupancyGrid::getGoal(gm::Pose& goal) const
{
	goal = this->goal_pose_;
	return this->has_goal_;
}

void MultiTraitOccupancyGrid::setGoal(int x, int y)
{
	int tx = x+this->x_offset_;
	int ty = y+this->y_offset_;
	this->gridCellToMeter(tx, ty, this->goal_pose_.position.x, this->goal_pose_.position.y);
	this->goal_pose_.position.x += this->map_meta_data_.origin.position.x;
	this->goal_pose_.position.y += this->map_meta_data_.origin.position.y;
	this->has_goal_ = true;
	try
	{
		this->addPointTrait(x, y, CellTrait::GOAL, 1000);
	}
	catch(bool& e)
	{
		//Do nothing, just means the goal isn't on the map
	}
}

void MultiTraitOccupancyGrid::setGoal(double x, double y)
{
	this->has_goal_ = true;
	this->goal_pose_.position.x = x+((double)this->x_offset_)/this->map_meta_data_.resolution+this->map_meta_data_.origin.position.x;
	this->goal_pose_.position.y = y+((double)this->y_offset_)/this->map_meta_data_.resolution+this->map_meta_data_.origin.position.y;
	try
	{
		this->addPointTrait(x, y, CellTrait::GOAL, 1000);
	}
	catch(bool& e)
	{
		//Do nothing, just means the goal isn't on the map
	}
}

void MultiTraitOccupancyGrid::setGoal(const gm::Pose& goal)
{
	this->has_goal_  = true;
	this->goal_pose_ = goal;
	try
	{
		this->addPointTrait(goal, CellTrait::GOAL, 1000);
	}
	catch(bool& e)
	{
		//Do nothing, just means the goal isn't on the map
	}
}

ros::Time MultiTraitOccupancyGrid::getCreationTime() const
{
	return this->map_meta_data_.map_load_time;
}

bool MultiTraitOccupancyGrid::generateOccupancyGridforTrait(nm::OccupancyGrid& message, ogu::CellTrait trait) const
{
	if(this->trait_map_.count(trait.getEnum())==1)
	{
		message =  this->grid_.at(this->trait_map_.at(trait.getEnum()));
		return true;
	}
	return false;
}

bool MultiTraitOccupancyGrid::boundsCheck(unsigned int x, unsigned int y) const
{
	bool xbound = x<this->map_meta_data_.width;
	bool ybound = y<this->map_meta_data_.height;
	return xbound&&ybound;
}

void MultiTraitOccupancyGrid::meterToGridCell(double xm, double ym, int& xg, int& yg) const
{
	xg = (int)(xm/this->map_meta_data_.resolution);
	yg = (int)(ym/this->map_meta_data_.resolution);
}

void MultiTraitOccupancyGrid::gridCellToMeter(int xg, int yg, double& xm, double& ym) const
{
	xm = (double)(xg)*this->map_meta_data_.resolution;
	ym = (double)(yg)*this->map_meta_data_.resolution;
}

void MultiTraitOccupancyGrid::offsetAdjust(int& x, int& y) const
{
	x+=this->x_offset_;
	y+=this->y_offset_;
}

void MultiTraitOccupancyGrid::projectPoseToGrid(const gm::Pose& pose, int& x, int& y) const
{
	this->meterToGridCell(pose.position.x-this->map_meta_data_.origin.position.x, pose.position.y-this->map_meta_data_.origin.position.y, x, y);
}
