/**
 * @file	OryxPathPlanningUtilities.h
 * @date	Oct 17, 2012
 * @author	Adam Panzica
 * @brief	Header containing definitions that are used throughout the oryx_path_planning package
 */

#ifndef ORYXPATHPLANNINGUTILITIES_H_
#define ORYXPATHPLANNINGUTILITIES_H_

#ifndef POINT_PRECISION
#define POINT_PRECISION float	///Macro for defining the precision of the point values being used
#else
#error POINT_PRECISION is already defined
#endif

//*********************** SYSTEM DEPENDENCIES ************************************//
#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <pcl/registration/distances.h>
#include <sensor_msgs/PointCloud2.h>
//*********************** LOCAL DEPENDENCIES ************************************//
#include "TypeDefinitions.h"
#include "PointConverter.hpp"

//*********************** MACROS ************************************//

#ifndef PARAM_WARN
/**
 * @author	Adam Panzica
 * @brief	Prints out a default warning message to ROS_WARN if a parameter was not defined
 * @param param std::string containing the name of the parameter that was not defined
 * @param value std::string representation of the default value that will be used it its place
 */
#define PARAM_WARN(param,value) ROS_WARN(warn_message.c_str(), param.c_str(), value.c_str())
#else
#error PARAM_WARN is already defined!
#endif

#ifndef PRINT_POINT
/**
 * @author	Adam Panzica
 * @brief	Prints out information about a pcl::PointXYZRGBA to ROS_INFO
 * @param prefix	c_str containing a descriptive prefix about the point
 * @param point		pcl::PointXYZRBGA to print the data of
 */
#define PRINT_POINT(prefix,point) ROS_INFO("%s: Point<%f,%f,%f,%x>", prefix, point.x, point.y, point.z, point.rgba)
#else
#error PRINT_POINT is already defined!
#endif

namespace oryx_path_planning{

//*********************** CONSTANTS ******************************//
/**
 * @author	Adam Panzica
 * @brief	singleton class which returns dynamically generated constants
 */
class constants{
public:
	static double PI(){
		return std::atan(1.0)*4;	///Since C++ lacks a predefined PI constant, define it here
	}
};

const std::string warn_message("Parameter <%s> Not Set. Using Default Value <%s>");	///Standard parameter warning message

//*********************** CLASS DEFINITIONS ******************************//
/**
 * @author Adam Panzica
 * @brief Enum for describing the state of a point
 * The integer enumeration value is mapped to an RGBA color space representation for ease of translating to color map
 * as well as to allow for using the pcl::PointRBGA class to represent data
 */
typedef enum PointTrait_t{
	OBSTACLE		= 0xFF0000, //!< OBSTACLE		Point on the grid contains an obstacle (Red)
	INFLATED		= 0x0000FF,	//!< INFLATED		Point on the grid is occupied by a safety inflation radius (Blue)
	UNKNOWN			= 0x808080, //!< UNKNOWN		Point on the grid is unknown (Grey)
	FREE_HIGH_COST	= 0x008080,	//!< FREE_HIGH_COST	Point on the grid is free but is expensive to travel over (Teal)
	FREE_LOW_COST	= 0x008000, //!< FREE_LOW_COST	Point on the grid is free but is easy to travel over (Green)
	TRAVERSED		= 0x6B8E23,	//!< TRAVERSED		Point on the grid is free but has been previously traversed (Olive Drab)
	GOAL            = 0xFFFF00,	//!< GOAL			Point on the grid is the goal position (Yellow)
	TENTACLE		= 0xCD00CD,	//!< TENTACLE		Point on the grid is a Tentacle marker (Pink)
	ROBOT			= 0x000001,	//!< ROBOT			Point on the grid is the robot's center point
} PointTrait;

/**
 * @author Adam Panzics
 * @brief basic chainable exception class
 *
 * Provides an implementation of std::exception which allows for chaining of exceptions.
 * Returns the chained error message in the form of:
 *
 * "this_message \n Caused By-> that_message"
 */
class ChainableException: public std::runtime_error{
public:
	/**
	 * Default empty constructor
	 */
	ChainableException():
		std::runtime_error(""),
		cause(){
	}
	/**
	 * Standard Copy constructor
	 * @param exception The exception to initialize this exception's fields to
	 */
	ChainableException(oryx_path_planning::ChainableException& exception):
		std::runtime_error(exception),
		cause(exception.cause){
	}

	/**
	 * Constructor for creating a new exception without a cause
	 * @param message The error message for this exception
	 */
	ChainableException(std::string& message):
		std::runtime_error(message),
		cause(){
	}
	/**
	 * Constructor for creating a new exception with a cause
	 * @param message	The error message for the exception
	 * @param cause		The exception which caused this exception
	 */
	ChainableException(std::string& message, std::exception& cause):
		std::runtime_error(genMessage(message, cause)),
		cause(cause){
	}

	~ChainableException() throw(){};

	/**
	 * Gets the raw exception that caused this exception
	 * @return A std::exception which caused this exception
	 */
	std::exception& getCause(){
		return this->cause;
	}
protected:
	std::exception	cause;		///The exception which caused this ChaingedException

	std::string& genMessage(std::string& message, std::exception& cause){
		message += "\nCaused By-> ";
		message += std::string(cause.what());
		return message;
	}
};

//*********************** HELPER FUNCTIONS ******************************//

/**
 * Rounds a value to the nearest scale point. EX: raw=12.35, frac=.25, return = 12.25
 * @param raw Raw value to round
 * @param frac Fraction to scale to
 * @return The result of std::floor(raw/scale)*scale
 */
inline double roundToFrac(double raw, double frac){
	return std::floor(raw/frac)*frac;
}

/**
 * Turns a real coordinate into an integer value for a grid with a given resolution.
 * @param raw The raw coordinate value
 * @param resolution The resolution of the grid to place the point on
 * @return The location of the point on the grid, equivalent to (int)std::floor(raw/resolution)
 */
inline int roundToGrid(double raw, double resolution){
	return (int)std::floor(raw/resolution);
}

/**
 * Turns a given point in grid coordinates into a real value
 * @param grid			The point on the grid
 * @param resolution	The resolution of the grid
 * @return A double corresponding to a real value represented by a grid coordinate
 */
inline double gridToReal(int grid, double resolution){
	return (double)grid*resolution;
}

/**
 * @author	Adam Panzica
 * @brief	Generates the series of points along a line between two points and places them into a PointCloud
 * @param startPoint [in]	The starting point of the line
 * @param endPoint	 [in]	The ending point of the line
 * @param res		 [in]	The resolution of the grid to place points on
 * @param rgba		 [in]	The RGBA value to place into the points on the line
 * @param cloud		 [out]	The PointCloud to put the generated points into
 *
 * Uses a 3D implementation of Bresenham's algorithm to generate an approximation of the straight line rasterized to the
 * given grid resolution. The RGBA values of the start point and end point are copied into the respective points in the cloud,
 * with the points between them filled with the value given by the RGBA parameter
 */
inline void castLine(Point& startPoint, Point& endPoint, int rgba, PointCloud& cloud){
	int x_0 = startPoint.x, x_f= endPoint.x, x, delta_x, step_x;
	int y_0 = startPoint.y, y_f= endPoint.y, y, delta_y, step_y;
	int z_0 = startPoint.z, z_f= endPoint.z, z, delta_z, step_z;
	bool s_xy, s_xz;
	int error_xy, error_xz;
	int cx, cy, cz;
	int initial_size = cloud.size();


	//Figure out if the line is 'steep' along the xy plane
	s_xy = std::abs(y_f-y_0)>std::abs(x_f-x_0);
	if(s_xy){
		std::swap<int>(x_0, y_0);
		std::swap<int>(x_f, y_f);
	}
	//Figure out if the line is 'steep' along the xz plane
	s_xz = std::abs(z_f-z_0)>std::abs(x_f-x_0);
	if(s_xz){
		std::swap<int>(x_0, z_0);
		std::swap<int>(x_f, z_f);
	}

	//Calculate the delta in each axis
	delta_x = std::abs(x_f-x_0);
	delta_y = std::abs(y_f-y_0);
	delta_z = std::abs(z_f-z_0);

	//Calculate starting error values
	error_xy = delta_x/2;
	error_xz = delta_x/2;

	//Determine line direction
	(x_0>x_f)?(step_x=-1):(step_x=1);
	(y_0>y_f)?(step_y=-1):(step_y=1);
	(z_0>z_f)?(step_z=-1):(step_z=1);

	//Set up initial point
	y = y_0;
	z = z_0;

	//ROS_INFO("Calculated Line Parameters: s_xy=%s, s_xz=%s, delta_x=%d, delta_y=%d, delta_z=%d, step_x=%d, step_y=%d, step_z=%d",
	//(s_xy)?"true":"false",(s_xz)?"true":"false", delta_x, delta_y, delta_z, step_x, step_y, step_z);

	//Iterate across the line
	for(x = x_0; x<x_f; x+=step_x){
		//store x,y,z for un-swapping
		cx = x; cy = y; cz = z;

		//unswap if needed
		if(s_xz) std::swap<int>(cx, cz);
		if(s_xy) std::swap<int>(cx, cy);

		//Write out the point
		Point point;
		point.x = cx;
		point.y = cy;
		point.z = cz;
		point.rgba = rgba;
		//PRINT_POINT("Line Calculated", point);
		cloud.push_back(point);

		//update the y and z axies
		error_xy -= delta_y;
		error_xz -= delta_z;

		//step y
		if(error_xy < 0){
			y+= step_y;
			error_xy += delta_x;
		}

		//step z
		if(error_xz < 0){
			z+= step_z;
			error_xz += delta_x;
		}
	}

	//ROS_INFO("Line Generated, editing first/last point");
	//Set the RGBA values of the first and last point to their correct values
	cloud.at(cloud.size()-(cloud.size()-initial_size)).rgba = startPoint.rgba;
	cloud.at(cloud.size()-1).rgba = endPoint.rgba;
}

/**
 * @author	Adam Panzica
 * @brief	Generates the series of points along a 90* arc between two points and places them into a PointCloud
 * @param radius		The radius of the arch to cast
 * @param sweep_angle	The angle, in radians, to sweep the arc in the given quadrant and plane
 * @param rgba			The value to fill the rgba field of the points with
 * @param origin		The origin of the arc
 * @param cloud			PointCloud to write the arc into
 * @param quadrant		The quadrant that the arc lies in (1, 2, 3 or 4, anything outside that range will be 1)
 * @param plane			The plane to cast the arc in (0 for xy, 1 for xz, 2 for yz)
 */
inline void castArc(int radius, double sweep_angle, int rgba, Point& origin, PointCloud& cloud, int quadrant=1, int plane=0){
	int x = 0, y = radius;
	int g = 3 - 2*radius;
	int diagonalInc = 10 - 4*radius;
	int rightInc = 6;
	//Calculate the halfway cuttoff (we can use symmetry after this point to speed up calculations)
	double halfCutoff = std::atan2(1,1);
	//Calculate the cutoff point. If it's greater than pi/4, this won't actually matter
	double fullCutoff = oryx_path_planning::constants::PI()/2.0 - sweep_angle;
	//Calculate initial swept angle
	double sweptAngle = std::atan2(y-origin.y, x-origin.x);
	//Draw the first half of the arc, or the whole arc if the sweep angle is less than pi/4
	while (sweptAngle>fullCutoff&&sweptAngle>halfCutoff) {
		//Build the point.
		Point point;
		point.x = x;
		point.y = y;
		point.z = 0;
		point.rgba = rgba;
		//Calculate the currently swept angle
		sweptAngle = std::atan2(point.y, point.x);
		//Adjust the sign of the x/y values to account for quadrant changes
		switch(quadrant){
		case 2:
			point.x = -point.x;
			break;
		case 3:
			point.x = -point.x;
			point.y = -point.y;
			break;
		case 4:
			point.y = -point.y;
			break;
		}
		/*Point pointBottom;
	    pointBottom.x = point.y;
	    pointBottom.y = point.x;
	    pointBottom.z = point.z;
	    pointBottom.rgba = rgba;*/
		switch(plane){
		case 1:
			std::swap<float>(point.x, point.z);
			//std::swap<float>(pointBottom.x, pointBottom.z);
			break;
		case 2:
			std::swap<float>(point.y, point.z);
			//std::swap<float>(pointBottom.y, pointBottom.z);
			break;
		}
		if (g >=  0) {
			g += diagonalInc;
			diagonalInc += 8;
			y -= 1;
		}
		else {
			g += rightInc;
			diagonalInc += 4;
		}
		rightInc += 4;
		x += 1;
		//cloud.push_back(pointTop);
		//PRINT_POINT("Arc Generated Bottom Point", point);
		cloud.push_back(point);
	}
	//ROS_INFO("Reached Swept Angle %f, Goal Angle%f", sweptAngle, sweep_angle);
	//If the full arc was greater than pi/4, build the rest of the arc from symmetry
	if(sweep_angle>halfCutoff){
		for(int index=cloud.size()-1; index>=0; index--){
			Point& point = cloud.at(index);
			//otherwise push back the symmetric point
			Point pointSym;
			switch(plane){
			case 1:
				pointSym.x = point.x;
				pointSym.y = point.z;
				pointSym.z = point.y;
				break;
			case 2:
				pointSym.x = point.z;
				pointSym.y = point.y;
				pointSym.z = point.x;
				break;
			case 0:
			default:
				pointSym.x = point.y;
				pointSym.y = point.x;
				pointSym.z = point.z;
				break;
			}
			pointSym.rgba = point.rgba;
			//PRINT_POINT("Arc Generated Top Point", pointSym);
			cloud.push_back(pointSym);
			//ROS_INFO("Reached Swept Angle %f, Goal Angle%f", std::atan2(point.y, point.x), sweep_angle);
			//If the next point is past the sweep angle, than we're done. break
			sweptAngle = std::atan2(pointSym.y, pointSym.x);
			if(sweptAngle>sweep_angle){
				//ROS_INFO("I'm breaking cuz I got to the angle I wanted");
				break;
			}
		}
	}
	//If we're sweeping more than PI/2, need to use additional symmetry to finish the arc
	if(sweep_angle>oryx_path_planning::constants::PI()/2){
		ROS_INFO("I'm doing the arc past pi/2");
		double remainingSweep = sweep_angle-oryx_path_planning::constants::PI()/2;
		for(int index = cloud.size()-1; index>=0; index--){
			Point pointFinal;
			Point& point = cloud.at(index);
			switch(quadrant){
			case 2:
				break;
			case 3:
				break;
			case 4:
				break;
			case 0:
			default:
				pointFinal.x = point.x;
				pointFinal.y = -point.y;
				pointFinal.z = point.z;
				pointFinal.rgba = point.rgba;
				break;
			}
			cloud.push_back(pointFinal);
			Point checkPoint;
			checkPoint.getVector4fMap()=pointFinal.getVector4fMap()-origin.getVector4fMap();
			if(std::abs(std::atan(checkPoint.y/checkPoint.x))>remainingSweep) break;
		}
	}
}

///**
// * @author	Adam Panzica
// * @brief	Simple class which generates
// */
//class InflationBubble{
//public:
//
//	inline InflationBubble(Point& point, int radius){
//
//	}
//
//	inline InflationBubble(const Point& point, int radius){
//
//	}
//
//	inline virtual ~InflationBubble(){}
//
//	inline PointCloud::iterator begin(){return this->points.begin();}
//
//	inline PointCloud::iterator end(){return this->points.end();}
//
//	inline PointCloud::const_iterator begin() const{return this->points.begin();}
//
//	inline PointCloud::const_iterator end() const{return this->points.end();}
//
//private:
//	double radius;
//	double resolution;
//	PointCloud points;
//
//	/**
//	 * @author	Adam Panzica
//	 * @brief	Constructs a 2d rasterized circle
//	 * @param x0 x-origin of the circle
//	 * @param y0 y-origin of the circle
//	 * @param radius radius of the circle
//	 */
//	void rasterCircle(const Point& origin, int radius)
//	{
//		int f = 1 - radius;
//		int ddF_x = 1;
//		int ddF_y = -2 * radius;
//		int x = 0;
//		int y = radius;
//
//		Point yHigh;
//		yHigh.x	= origin.x;
//		yHigh.y	= origin.y + radius;
//		yHigh.z	= origin.z;
//		yHigh.rgba= origin.rgba;
//		Point yLow;
//		yLow.x	= origin.x;
//		yLow.y	= origin.y - radius;
//		yLow.z	= origin.z;
//		yLow.rgba= origin.rgba;
//		Point xHigh;
//		xHigh.x	= origin.x+radius;
//		xHigh.y	= origin.y;
//		xHigh.z	= origin.z;
//		xHigh.rgba= origin.rgba;
//		Point xLow;
//		xLow.x	= origin.x-radius;
//		xLow.y	= origin.y;
//		xLow.z	= origin.z;
//		xLow.rgba= origin.rgba;
//
//
//		this->points.push_back(yHigh);
//		this->points.push_back(yLow);
//		//this->points.push_back(xHigh);
//		//this->points.push_back(xLow);
//
//		while(x < y)
//		{
//			// ddF_x == 2 * x + 1;
//			// ddF_y == -2 * y;
//			// f == x*x + y*y - radius*radius + 2*x - y + 1;
//			if(f >= 0)
//			{
//				y--;
//				ddF_y += 2;
//				f += ddF_y;
//			}
//			x++;
//			ddF_x += 2;
//			f += ddF_x;
//			setPixel(x0 + x, y0 + y);
//			setPixel(x0 - x, y0 + y);
//			setPixel(x0 + x, y0 - y);
//			setPixel(x0 - x, y0 - y);
//			setPixel(x0 + y, y0 + x);
//			setPixel(x0 - y, y0 + x);
//			setPixel(x0 + y, y0 - x);
//			setPixel(x0 - y, y0 - x);
//		}
//	}
//};

} /* oryx_path_planning */;


#endif /* ORYXPATHPLANNINGUTILITIES_H_ */
