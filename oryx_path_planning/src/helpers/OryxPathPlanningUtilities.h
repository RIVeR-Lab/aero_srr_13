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

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/registration/distances.h>
#include <sensor_msgs/PointCloud2.h>

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

//*********************** TYPEDEFS ******************************//
///Typedef to allow for easier to read code
typedef pcl::PointXYZRGBA Point;

///Typedef to allow for convenient sharing of a PointCloud<pcl::PointXYZRGBA>
typedef pcl::PointCloud<Point> PointCloud;

///Typedef to allow for convenient sharing of a pcl::PointXYZRGBA via pointer
typedef boost::shared_ptr<Point> PointPtr;

///Typedef to allow for convenient sharing of a PointCloud<pcl::PointXYZRGBA> > via pointer
typedef boost::shared_ptr<pcl::PointCloud<Point> > PointCloudPtr;

//*********************** CONSTANTS ******************************//

const double PI = std::atan(1.0)*4;	///Since C++ lacks a predefined PI constant, define it here

const std::string warn_message("Parameter <%s> Not Set. Using Default Value <%s>");	///Standard parameter warning message

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
inline void castLine(Point& startPoint, Point& endPoint, double res, int rgba, PointCloud& cloud){
	int x_0 = roundToGrid(startPoint.x, res), x_f= roundToGrid(endPoint.x, res), x, delta_x, step_x;
	int y_0 = roundToGrid(startPoint.y, res), y_f= roundToGrid(endPoint.y, res), y, delta_y, step_y;
	int z_0 = roundToGrid(startPoint.z, res), z_f= roundToGrid(endPoint.z, res), z, delta_z, step_z;
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
		point.x = gridToReal(cx, res);
		point.y = gridToReal(cy, res);
		point.z = gridToReal(cz, res);
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
 * @brief	Simple class which generates
 */
//class InflationBubble{
//public:
//	///Typedef for an iterator for the class
//	typedef PointCloud::iterator iterator;
//	///Typedef for a const_iterator for the class
//	typedef PointCloud::const_iterator const_iterator;
//
//	inline InflationBubble(Point& point, double radius, double resolution){
//
//	}
//
//	inline InflationBubble(const Point& point, double radius, double resolution){
//
//	}
//
//	inline virtual ~InflationBubble(){}
//
//	inline iterator begin(){return this->points.begin();}
//
//	inline iterator end(){return this->points.end();}
//
//	inline const_iterator begin() const{return this->points.begin();}
//
//	inline const_iterator end() const{return this->points.end();}
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
//	void rasterCircle(int x0, int y0, int radius)
//	{
//	  int f = 1 - radius;
//	  int ddF_x = 1;
//	  int ddF_y = -2 * radius;
//	  int x = 0;
//	  int y = radius;
//
//	  Point yHigh(x0, y0 + radius);
//	  Point yLow(x0, y0 - radius);
//	  Point xHigh(x0 + radius, y0);
//	  Point xLow(x0 - radius, y0);
//
//
//	  this->points.push_back(yHigh);
//	  this->points.push_back(yLow);
//	  this->points.push_back(xHigh);
//	  this->points.push_back(yLow);
//
//	  while(x < y)
//	  {
//	    // ddF_x == 2 * x + 1;
//	    // ddF_y == -2 * y;
//	    // f == x*x + y*y - radius*radius + 2*x - y + 1;
//	    if(f >= 0)
//	    {
//	      y--;
//	      ddF_y += 2;
//	      f += ddF_y;
//	    }
//	    x++;
//	    ddF_x += 2;
//	    f += ddF_x;
//	    setPixel(x0 + x, y0 + y);
//	    setPixel(x0 - x, y0 + y);
//	    setPixel(x0 + x, y0 - y);
//	    setPixel(x0 - x, y0 - y);
//	    setPixel(x0 + y, y0 + x);
//	    setPixel(x0 - y, y0 + x);
//	    setPixel(x0 + y, y0 - x);
//	    setPixel(x0 - y, y0 - x);
//	  }
//	}
//};

} /* oryx_path_planning */;


#endif /* ORYXPATHPLANNINGUTILITIES_H_ */
