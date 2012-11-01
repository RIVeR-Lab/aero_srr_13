/**
 * @file	PointConverter.hpp
 * @date	Nov 1, 2012
 * @author	Adam Panzica
 * @brief	Header containg class definitions for converting between points
 */

#ifndef POINTCONVERTER_HPP_
#define POINTCONVERTER_HPP_

//*********************** LOCAL DEPENDENCIES ************************************//
#include "TypeDefinitions.h"
namespace oryx_path_planning{
/**
 * @author	Adam Panzica
 * @brief	Class for converting a point with 'integer' x/y/z into some 'engineering unit' x/y/z coordinates
 *
 * There will obviously be rounding errors do to numerical stability. However, this allows integer representations of grid space
 * to be approximated in the internal point clouds, which greatly improves the numerical stability of lookups to a grid representation,
 * and still have very fast output to 'real' engineering units for display or comparison to results which are in engineering units
 */
class PointConverter{
public:
	inline PointConverter(float scale_value){
		cm = cm.Identity(4,4)*scale_value;
	}

	/**
	 * @author	Adam Panzica
	 * @brief	Performs the actual work
	 * @param point	 Reference to the point to convert to engineering units
	 * @param output Reference to a point to place the converted data into
	 */
	inline void convert(Point& point, Point& output) const{
			output.getVector4fMap() = cm*point.getVector4fMap();
			output.rgba = point.rgba;
	}
	inline void convert(const Point& point, Point& output) const{
		output.getVector4fMap() = cm*point.getVector4fMap();
		output.rgba = point.rgba;
	}
private:
	/**
	 * @brief Conversion matrix used to fast-convert from one coordinate system to another
	 *
	 * A 4x4 SSE aligned diagonal matrix which when multiplied with a vector will convert the vector from one coordinate system ('integer')
	 * to another ('engineering unit'). It will have the form
	 *
	 * [sv	0	... ]
	 * [0	sv	... ]
	 * [0	0	... ]
	 * [0	0	... ]
	 *
	 */
	Eigen::Matrix4f cm;
};

};	/* oryx_path_planning */
#endif /* POINTCONVERTER_HPP_ */
