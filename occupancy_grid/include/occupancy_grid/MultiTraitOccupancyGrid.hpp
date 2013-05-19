/**
 * @file   OccupancyGrid.hpp
 *
 * @date   May 7, 2013
 * @author Adam Panzica
 * @brief  Function/Class definitions for occupancy_grid namespace
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

#ifndef OCCUPANCYGRID_HPP_
#define OCCUPANCYGRID_HPP_

//*********** SYSTEM DEPENDANCIES ****************//
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/unordered_map.hpp>
#include <tf/transform_datatypes.h>
#include <boost/shared_ptr.hpp>
//************ LOCAL DEPENDANCIES ****************//
#include <occupancy_grid/MultiTraitOccupancyGridMessage.h>
//***********    NAMESPACES     ****************//
namespace nm  = nav_msgs;
namespace gm  = geometry_msgs;
namespace occupancy_grid
{

typedef int8_t cell_data_t; ///typedef over the data type of the OccupancyGrid cell data

/**
 * Utilities for working with occupancy grids
 */
namespace utilities
{
/**
 * Class for representing the various types the occupancy grid cells can contain
 */
class CellTrait
{
public:
	/**
	 * @author Adam Panzica
	 * @brief enum representing the various types the occupancy grid cells can contain
	 */
	enum Enum
	{
		UNKOWN = -1,        //!< UNKOWN Cell's value is unkown
		FREE_LOW_COST = 10, //!< FREE_LOW_COST Cell contains open terrain
		FREE_HIGH_COST= 20, //!< FREE_HIGH_COST Cell contains difficult terrain
		OBSTACLE      = 30, //!< OBSTACLE Cell contains an obstacle
		TRAVERSED     = 40, //!< TRAVERSED Cell contains previously traversed terrain
		GOAL          = 100,//!< GOAL Cell contains a goal point
	};

	CellTrait(void);
	CellTrait(Enum value);
	CellTrait(int value);


	/**
	 * @author Adam Panzica
	 * @return The Enum representation of the CellTrait
	 */
	Enum getEnum(void) const;

	/**
	 * @author Adam Panzica
	 * @return The String representation of the CellTrait
	 */
	std::string getString(void) const;

	/**
	 * @author Adam Panzica
	 * @return The integer representation of the CellTrait
	 */
	int getValue(void) const;


	//Operator overloads
	//Assignment
	std::ostream& operator<<(std::ostream& out) const;
	CellTrait& operator=(const CellTrait& rhs);
	CellTrait& operator=(const int& rhs);
	//Equality
	bool operator==(const CellTrait& rhs) const;
	bool operator==(const int& rhs) const;
	bool operator==(const Enum& rhs) const;



private:
	Enum enum_;

	/**
	 * @author Adam Panzica
	 * @brief Converts an integer value into a CellTrait Enum
	 * @param [in] value The value to convert
	 * @return The corrisponding Enum, or UNKOWN if it wasn't a valid conversion
	 */
	static Enum enumFromValue(int value);

	/**
	 * @author Adam Panzica
	 * @brief Converts an Enum into a std::string
	 * @param [in] value the Enum to convert
	 * @return A string representation of the enum
	 */
	static std::string stringFromEnum(Enum value);
};


/**
 * @author Adam Panzica
 * @brief  Calculates the index into a row-major linear array representing a grid for a given x,y pair
 * @param [in] x The x position in the grid (width)
 * @param [in] y The y position in the grid (height)
 * @param [in] width The width of the grid
 * @return Index into row-major linear array for the given x,y pair
 */
inline int calcIndexRowMajor2D(int x, int y, int width)
{
	return y*width+x;
}

/**
 * @author Adam Panzica
 * @brief normalizes a set of confidence values that range from 0-100
 * @param [in]  values An array of values to normalize
 * @param [in]  size   The size of the input and output arrays
 * @param [out] result An array to write out the normalized values. May safely be the same array as input, though will be destructive in that case
 */
inline void normalizeConfidance(const cell_data_t values[], int size, cell_data_t result[])
{
	int sum = 0;
	for(int i=0; i<size; i++)
	{
		sum+=values[i];
	}
	for(int i=0; i<size; i++)
	{
		result[i] = values[i]/sum;
	}
}

/**
 * @author Adam Panzica
 * @brief Builds an empty nav_msgs::OccupancyGrid message using the MapMetaData contained in the info field of the message
 * @param grid The message to initialize with an empty grid. The info field must be set to the desired dimensions
 */
inline void buildEmptyOccupancyGrid(nm::OccupancyGrid& grid)
{
	int size  = calcIndexRowMajor2D(grid.info.width, grid.info.height, grid.info.width);
	grid.data = std::vector<cell_data_t>(size);
}


}; /* END UTILITIES */

namespace ogu = occupancy_grid::utilities;

class MultiTraitOccupancyGrid
{
public:
	typedef ogu::CellTrait trait_t;
private:
	typedef std::vector<nm::OccupancyGrid>   grid_slice_t;
	typedef boost::unordered_map<trait_t::Enum, int> trait_map_t;
	typedef boost::unordered_map<int, trait_t::Enum> index_map_t;
	grid_slice_t       grid_;		   ///The backing map data. Index 0 is always the current max confidence PointTrait, with the remaining indexes defined by the trait map
	nm::MapMetaData    map_meta_data_; ///The meta-data defining information about the grid
	trait_map_t        trait_map_;     ///Mapping between PointTrait type and vector index
	index_map_t        index_map_;     ///Mapping between vector index and PointTrait type
	std::string        frame_id_;
	cell_data_t*       temp_cell_values_; ///array for temporary storage of cell confidances used by normailization
	gm::Pose           goal_pose_;     ///The location of the goal pose on the grid, if there is one
	bool               has_goal_;      ///flag for signalling the goal point

	/**
	 * @author Adam Panzica
	 * @brief helper function for placing a goal point on the map
	 * @param x
	 * @param y
	 */
	void place_goal(unsigned int x, unsigned int y);

	/**
	 * @author Adam Panzica
	 * @brief Checks if the requested point is in the grid
	 * @param x The x-coord of the grid, in grid units
	 * @param y The y-coord of the grid, in grid units
	 * @return True if the point was in bounds, else fase
	 */
	bool boundsCheck(unsigned int x, unsigned int y) const;
public:
	MultiTraitOccupancyGrid();
	/**
	 * @author Adam panzica
	 * @brief Constructs a new grid
	 * @brief [in] frame_id The frame that the map is representing
	 * @param [in] slice_info MapMetaData defining the basic 2D properties of the grid
	 * @param [in] traits A vector of the traits that a point could be. Must, at minimum, contain UNKOWN, FREE_LOW_COST, and OBSTACLE for merging nav_msgs::OccupancyGrids from SLAM services such as gampping to be possible
	 * @param [in] initial_trait The trait to fill the map with initially
	 */
	MultiTraitOccupancyGrid(const std::string& frame_id, const std::vector<trait_t>& traits, trait_t initial_trait, const nm::MapMetaData& slice_info);

	/**
	 * @author Adam Panzica
	 * @brief Constructs a new grid from a MultiTraitOccupancyGridMessage
	 * @param [in] message
	 */
	MultiTraitOccupancyGrid(const MultiTraitOccupancyGridMessage& message);

	/**
	 * @author Adam Panzica
	 * @brief copy constructor
	 * @param [in] copy
	 */
	MultiTraitOccupancyGrid(const MultiTraitOccupancyGrid& copy);

	virtual ~MultiTraitOccupancyGrid();

	/**
	 * @author Adam Panzica
	 * @return The X size of the grid, in grid units
	 */
	int    getXSizeGrid()  const;
	/**
	 * @author Adam Panzica
	 * @return The X size of the grid, in meters
	 */
	double getXSizeMeter() const;
	/**
	 * @author Adam Panzica
	 * @return The Y size of the grid, in grid units
	 */
	int    getYSizeGrid()  const;
	/**
	 * @author Adam Panzica
	 * @return The Y size of the grid, in meters
	 */
	double getYSizeMeter() const;

	/**
	 * @author Adam Panzica
	 * @return The resolution of the grid, in meters/cell
	 */
	double getResolution() const;

	/**
	 * @author Adam Panzica
	 * @return The time that the grid was created
	 */
	ros::Time getCreationTime() const;

	/**
	 * @author Adam Panzica
	 * @return The origin of the map. Will be in meters
	 */
	geometry_msgs::Pose getOrigin() const;

	/**
	 * @author Adam Panzica
	 * @param [out] goal The most recently added goal pose, if any
	 * @return true if there is a goal, else false
	 */
	bool getGoal(gm::Pose& goal) const;

	/**
	 * @author Adam Panzica
	 * @return The frame_id that the map is in
	 */
	std::string getFrameID() const;


	/**
	 * @author Adam Panzica
	 * @brief Gets the OccupancyGrid corrisponding to a trait type
	 * @param [out] message The message to fill
	 * @param [in] trait The trait to get confidances for
	 * @return True if that trait was in the grid, else false
	 */
	bool generateOccupancyGridforTrait(nm::OccupancyGrid& message, ogu::CellTrait trait) const;

	/**
	 * @author Adam Panzica
	 * @brief Adds a goal to the grid
	 * @param [in] goal The position of the goal to add
	 *
	 * Note that the goal point does not need to acctually be on the map. However in this situation it will not have a representation on the grid,
	 * and can only be found via the getGoal method
	 */
	void setGoal(const gm::Pose& goal);

	/**
	 * @author Adam Panzica
	 * @brief Adds a goal to the grid
	 * @param [in] x The x location on the grid, in grid-units
	 * @param [in] y The y location on the grid, in grid-units
	 *
	 * Note that the goal point does not need to acctually be on the map. However in this situation it will not have a representation on the grid,
	 * and can only be found via the getGoal method
	 */
	void setGoal(unsigned int x, unsigned int y);

	/**
	 * @author Adam Panzica
	 * @brief Adds a goal to the grid
	 * @param [in] x The x location on the grid, in meters
	 * @param [in] y The y location on the grid, in meters
	 *
	 *
	 * Note that the goal point does not need to acctually be on the map. However in this situation it will not have a representation on the grid,
	 * and can only be found via the getGoal method
	 */
	void setGoal(double x, double y);

	/**
	 * @author Adam Panzica
	 * @brief Gets the trait of a point on the grid
	 * @param [in] x The x location on the grid, in grid-units
	 * @param [in] y The y location on the grid, in grid-units
	 * @return The point trait at the given location on the grid. Will be the trait that has the highest normalized confidence
	 * @throw bool false if the requested point was not on the grid
	 */
	trait_t getPointTrait(unsigned int x, unsigned int y) const throw (bool);

	/**
	 * @author Adam Panzica
	 * @brief Gets the trait of a point on the grid
	 * @param [in] x The x location on the grid, in meters
	 * @param [in] y The y location on the grid, in meters
	 * @return The point trait at the given location on the grid. Will be the trait that has the highest normalized confidence
	 * @throw bool false if the requested point was not on the grid
	 */
	trait_t getPointTrait(double x, double y) const throw (bool);



	/**
	 * @author Adam Panzica
	 * @brief Gets the trait of a point on the grid
	 * @param [in] point A Pose who's x/y components corrispond to a location on the grid
	 * @return The point trait at the given location on the grid. Will be the trait that has the highest normalized confidence
	 * @throw bool false if the requested point was not on the grid
	 *
	 * Assumes that the given pose is not origin corrected. That is, we subtract the origin
	 * to get the location of the point relative to the grid.
	 */
	trait_t getPointTrait(const gm::PoseStamped& point) const throw (bool);

	/**
	 * @author Adam panzica
	 * @brief Generates a new ROS message from the grid
	 * @param [out] message to create
	 */
	void toROSMsg(MultiTraitOccupancyGridMessage& message) const;

	/**
	 * @author Adam Panzica
	 * @brief Generates a nav_msgs::OccupancyGrid message containing the confidences of a particular trait type
	 * @param [in] trait The trait confidances to fill the message with
	 * @param [out] message The message to fill
	 */
	void toROSMsg(trait_t trait, nm::OccupancyGrid& message) const;

	/**
	 * @author Adam Panzica
	 * @brief Adds confidence to a point being a particular trait
	 * @param [in] x The x location on the grid, in grid-units. Note that the grid always starts at 0
	 * @param [in] y The y location on the grid, in grid units. Note that the grid always starts at 0
	 * @param [in] trait The trait to add confidence to
	 * @param [in] confidence The amount of confidence to add. Defaults to 100 (full confidence)
	 * @throw bool false if the requested point was not on the grid
	 *
	 * Adds confidence to the probability that a point contains a given trait. Also removes proportional confidence from the other trait possibilities
	 */
	void addPointTrait(unsigned int x, unsigned int y, trait_t trait, int confidence = 100) throw (bool);

	/**
	 * @author Adam Panzica
	 * @brief Adds confidence to a point being a particular trait
	 * @param [in] x The x location on the grid, in meters. Note that the grid always starts at 0m
	 * @param [in] y The y location on the grid, in meters. Note that the grid always starts at 0m
	 * @param [in] trait The trait to add confidence to
	 * @param [in] confidence The amount of confidence to add. Defaults to 100 (full confidence)
	 * @throw bool false if the requested point was not on the grid
	 *
	 * Adds confidence to the probability that a point contains a given trait. Also removes proportional confidence from the other trait possibilities
	 */
	void addPointTrait(double x, double y, trait_t trait, int confidence = 100) throw (bool);

	/**
	 * @author Adam Panzica
	 * @brief Adds confidence to a point being a particular trait
	 * @param [in] point A Pose who's x/y components corrispond to a location on the grid
	 * @param [in] trait The trait to add confidence to
	 * @param [in] confidence The amount of confidence to add. Defaults to 100 (full confidence)
	 * @throw bool false if the requested point was not on the grid
	 *
	 * Assumes that the given pose is not origin corrected. That is, we add the origin
	 * to get the location of the point relative to the grid.
	 */
	void addPointTrait(const gm::PoseStamped& point, trait_t trait, int confidence = 100) throw (bool);

	/**
	 * @author Adam Panzica
	 * @brief Adds the confidance contained in an entier OccupancyGrid message to the grid.
	 * @param [in] confidances The gird of confidances to merge in
	 * @param [in] trait The trait type to treat the confidances as
	 * @param [in] scaling True if the input grid should be up/down sampled to match the resolution and dimmensions of this grid, else it will be copied 1/1 and clipped if the dimmensions do not match
	 * @param [in] use_zero_as_free Treates a value of zero as 100% confidance in FREE_LOW_COST at that location
	 * @param [in] use_negative_as_unkown Treats a negative value as 10% confidance in UNKOWN at that location
	 * @throw bool false if the requested point was not on the grid
	 *
	 * Note that even with scaling disabled, there will always be down-sampling of a grid that has a higher resoltuion. Also the two grids must be axis alligned.
	 *
	 * @todo Implement scaling
	 */
	void addPointTrait(const nm::OccupancyGrid& confidances, trait_t trait, bool scaling = false, bool use_zero_as_free = true, bool use_negative_as_unkown = true) throw (bool);

};

typedef boost::shared_ptr<MultiTraitOccupancyGrid> MultiTraitOccupancyGridPtr;
typedef boost::shared_ptr<const MultiTraitOccupancyGrid> MultiTraitOccupancyGridConstPtr;

}; /* END OCCUPANCY_GRID */


#endif /* OCCUPANCYGRID_HPP_ */
