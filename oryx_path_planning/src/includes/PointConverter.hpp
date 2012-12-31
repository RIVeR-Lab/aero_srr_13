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
namespace oryx_path_planning
{
/**
 * @author	Adam Panzica
 * @brief	Class for converting a point with 'integer' x/y/z into some 'engineering unit' x/y/z coordinates
 *
 * There will obviously be rounding errors do to numerical stability. However, this allows integer representations of grid space
 * to be approximated in the internal point clouds, which greatly improves the numerical stability of lookups to a grid representation,
 * and still have very fast output to 'real' engineering units for display or comparison to results which are in engineering units
 */
class PointConverter
{
public:
	/**
	 * Empty constructor needed by C++
	 */
	inline PointConverter(){}

	/**
	 * copy constructor
	 * @param converter Converter to copy from
	 */
	inline PointConverter(PointConverter& converter):
		cm_(converter.cm_){}
	inline PointConverter(const PointConverter& converter):
		cm_(converter.cm_){}

	/**
	 * @author	Adam Panzica
	 * @brief	Constructs a point converter with a given scaling factor
	 * @param scale_value The conversion factor to convert between coordinate systems
	 */
	inline PointConverter(float scale_value){
		cm_ = cm_.Identity(4,4)*scale_value;
	}

	/**
	 * @author	Adam Panzica
	 * @brief	Converts from an integer representation coordinate system to a floating point one
	 * @param from	 Reference to the point to convert to engineering units
	 * @param to Reference to a point to place the converted data into
	 */
	inline void convertToEng(Point& from, Point& to) const
	{
			to.getVector4fMap() = cm_*from.getVector4fMap();
			to.rgba = from.rgba;
	}
	inline void convertToEng(const Point& from, Point& to) const
	{
		to.getVector4fMap() = cm_*from.getVector4fMap();
		to.rgba = from.rgba;
	}

	/**
	 * @author	Adam Panzica
	 * @brief	Converts from an integer representation coordinate system to a floating point one
	 * @param from	 Reference to the point to convert to grid units
	 * @param to Reference to a point to place the converted data into
	 */
	inline void convertToGrid(Point& from, Point& to) const
	{
			to.getVector4fMap() = cm_.inverse()*from.getVector4fMap();
			to.rgba = from.rgba;
	}
	inline void convertToGrid(const Point& from, Point& to) const
	{
		to.getVector4fMap() = cm_.inverse()*from.getVector4fMap();
		to.rgba = from.rgba;
	}

	/**
	 * @author	Adam Panzica
	 * @brief	Assignment operator, equivalent to PointConverter copy constructor
	 * @param operand The PointConverter to assign this to
	 * @return *this
	 */
	PointConverter& operator=(PointConverter const& operand)
	{
		this->cm_ = operand.cm_;
		return *this;
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
	Eigen::Matrix4f cm_;
};

};	/* oryx_path_planning */
#endif /* POINTCONVERTER_HPP_ */
