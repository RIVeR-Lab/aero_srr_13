/**
 * @file	OryxPathPlanningUtilities.h
 * @date	Oct 17, 2012
 * @author	Adam Panzica
 * @brief	Header containing definitions that are used throughout the aero_path_planning package
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
#include <boost/foreach.hpp>
//*********************** LOCAL DEPENDENCIES ************************************//
#include <aero_path_planning/TypeDefinitions.h>
#include <aero_path_planning/PointConverter.hpp>

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

#define PRINT_POINT_S(prefix, point) prefix<<": Point<"<<point.x<<","<<point.y<<","<<point.z<<">"

namespace aero_path_planning
{

//*********************** CONSTANTS ******************************//
/**
 * @author	Adam Panzica
 * @brief	singleton class which returns dynamically generated constants
 */
class constants
{
public:
	static double PI()
	{
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
typedef enum PointTrait_t
{
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
class ChainableException: public std::runtime_error
{
public:
	/**
	 * Default empty constructor
	 */
	ChainableException();
	/**
	 * Standard Copy constructor
	 * @param exception The exception to initialize this exception's fields to
	 */
	ChainableException(aero_path_planning::ChainableException& exception);

	/**
	 * Constructor for creating a new exception without a cause
	 * @param message The error message for this exception
	 */
	ChainableException(std::string& message);
	/**
	 * Constructor for creating a new exception with a cause
	 * @param message	The error message for the exception
	 * @param cause		The exception which caused this exception
	 */
	ChainableException(std::string& message, std::exception& cause);

	~ChainableException() throw ();

	/**
	 * Gets the raw exception that caused this exception
	 * @return A std::exception which caused this exception
	 */
	std::exception& getCause();
protected:
	std::exception	cause_;		///The exception which caused this ChaingedException

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
double roundToFrac(double raw, double frac);

/**
 * Turns a real coordinate into an integer value for a grid with a given resolution.
 * @param raw The raw coordinate value
 * @param resolution The resolution of the grid to place the point on
 * @return The location of the point on the grid, equivalent to (int)std::floor(raw/resolution)
 */
int roundToGrid(double raw, double resolution);

/**
 * Turns a given point in grid coordinates into a real value
 * @param grid			The point on the grid
 * @param resolution	The resolution of the grid
 * @return A double corresponding to a real value represented by a grid coordinate
 */
double gridToReal(int grid, double resolution);

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
void castLine(const Point& startPoint, const Point& endPoint, const int& rgba, PointCloud& cloud);

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
void castArc(const int& radius, const double& sweep_angle, const int& rgba, const Point& origin, PointCloud& cloud, int quadrant=1, int plane=0);

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

} /* aero_path_planning */;


#endif /* ORYXPATHPLANNINGUTILITIES_H_ */
