/**
 * @file   OccupancyGrid.hpp
 *
 * @date   May 7, 2013
 * @author Adam Panzica
 * @brief  Class definitions for OccupancyGrid
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
//************ LOCAL DEPENDANCIES ****************//

//***********    NAMESPACES     ****************//
namespace nm  = nav_msgs;
namespace occupancy_grid
{

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
inline void normalizeConfidance(const uint8_t values[], int size, uint8_t result[])
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


}; /* END UTILITIES */

namespace ogu = occupancy_grid::utilities;

class MultiTraitOccupancyGrid
{
	typedef ogu::CellTrait trait_t;
private:
	std::vector<nm::OccupancyGrid>           grid_;		   ///The backing map data. Index 0 is always the current max confidence PointTrait, with the remaining indexes defined by the trait map
	nm::MapMetaData                          map_meta_data_; ///The meta-data defining information about the grid
	boost::unordered_map<trait_t::Enum, int> trait_map_;     ///Mapping between PointTrait type and vector index
	std::string                              frame_id_;

public:
	MultiTraitOccupancyGrid();
	/**
	 * @author Adam panzica
	 * @brief Constructs a new grid
	 * @brief [in] frame_id The frame that the map is representing
	 * @param [in] slice_info MapMetaData defining the basic 2D properties of the grid
	 * @param [in] traits A vector of the traits that a point could be
	 */
	MultiTraitOccupancyGrid(const std::string& frame_id, const std::vector<trait_t>& traits, const nm::MapMetaData& slice_info);

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
	 * @return The origin of the map. Will be in meters
	 */
	geometry_msgs::Pose getOrigin() const;

	/**
	 * @author Adam Panzica
	 * @return The frame_id that the map is in
	 */
	std::string getFrameID() const;

	/**
	 * @author Adam Panzica
	 * @brief Gets the trait of a point on the grid
	 * @param [in] x The x location on the grid, in grid-units
	 * @param [in] y The y location on the grid, in grid-units
	 * @return The point trait at the given location on the grid. Will be the trait that has the highest normalized confidence
	 */
	trait_t getPointTrait(int x, int y) const;

	/**
	 * @author Adam Panzica
	 * @brief Gets the trait of a point on the grid
	 * @param [in] x The x location on the grid, in meters
	 * @param [in] y The y location on the grid, in meters
	 * @return The point trait at the given location on the grid. Will be the trait that has the highest normalized confidence
	 */
	trait_t getPoitTrait(double x, double y) const;

	/**
	 * @author Adam Panzica
	 * @brief Adds confidence to a point being a particular trait
	 * @param [in] x The x location on the grid, in grid-units
	 * @param [in] y The y location on the grid, in grid units
	 * @param [in] trait The trait to add confidence to
	 * @param [in] confidence The amount of confidence to add. Defaults to 100 (full confidence)
	 *
	 * Adds confidence to the probability that a point contains a given trait. Also removes proportional confidence from the other trait possibilities
	 */
	void addPointTrait(int x, int y, trait_t trait, int confidence = 100);

	/**
	 * @author Adam Panzica
	 * @brief Adds confidence to a point being a particular trait
	 * @param [in] x The x location on the grid, in meters
	 * @param [in] y The y location on the grid, in meters
	 * @param [in] trait The trait to add confidence to
	 * @param [in] confidence The amount of confidence to add. Defaults to 100 (full confidence)
	 *
	 * Adds confidence to the probability that a point contains a given trait. Also removes proportional confidence from the other trait possibilities
	 */
	void addPointTrait(double x, double y, trait_t trait, int confidence = 100);

};

}; /* END OCCUPANCY_GRID */


#endif /* OCCUPANCYGRID_HPP_ */
