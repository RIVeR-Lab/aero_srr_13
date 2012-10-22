/**
 * @file	OccupancyGrid.h
 * @date	Oct 22, 2012
 * @author	Adam Panzica
 * @brief	//TODO fill in detailed discription here
 */

#ifndef OCCUPANCYGRID_H_
#define OCCUPANCYGRID_H_

//*********************** SYSTEM DEPENDENCIES ************************************//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/register_point_struct.h>

//*********************** LOCAL DEPENDENCIES ************************************//
#include "OryxPathPlanningUtilities.h"


//*********************** CLASS DEFINITIONS ************************************//
namespace oryx_path_planning{

/**
 * @author Adam Panzica
 * @brief Enum for describing the state of a point
 * The integer enumeration value is mapped to an RGBA color space representation for ease of translating to color map
 * as well as to allow for using the pcl::PointRBGA class to represent data
 */
enum PointTrait{
	OBSTACLE		= 0xFF0000, //!< OBSTACLE		Point on the grid contains an obstacle (Red)
	INFLATED		= 0x0000FF,	//!< INFLATED		Point on the grid is occupied by a safety inflation radius (Blue)
	UNKNOWN			= 0x808080, //!< UNKNOWN		Point on the grid is unknown (Grey)
	FREE_HIGH_COST	= 0x008080,	//!< FREE_HIGH_COST	Point on the grid is free but is expensive to travel over (Teal)
	FREE_LOW_COST	= 0x008000, //!< FREE_LOW_COST	Point on the grid is free but is easy to travel over (Green)
	GOAL            = 0xFFFF00	//!< GOAL			Point on the grid is the goal position (Yellow)
};

/**
 * @author Adam Panzica
 * @brief Struct which describes the data in a point in the occupancy grid
 */
struct PointXYZWithTrait: pcl::PointXYZ{
public:
	/**
	 * @author Adam Panzica
	 * @brief Default constructor which creates an empty point with its coords set to <0,0,0> and its PointTrait set to UNKNOWN
	 */
	inline PointXYZWithTrait():
		pcl::PointXYZ(){
		this->point_trait = UNKNOWN;
	}

	/**
	 * @author	Adam Panzica
	 * @brief	Constructor for creating a point with a set of coord, <x,y,z>, and a given PointTrait
	 * @param x		The x coordinate of the point
	 * @param y		The y coordinate of the point
	 * @param z		The z coordinate of the point
	 * @param trait	The PointTrait of the point
	 */
	inline PointXYZWithTrait(double x, double y, double z, oryx_path_planning::PointTrait trait):
		pcl::PointXYZ(x, y, z){
		this->point_trait = trait;
	}

	inline PointXYZWithTrait(oryx_path_planning::PointXYZWithTrait& point):
		PointXYZ(point){
		this->point_trait = point.point_trait;
	}

	/**
	 * Default destructor
	 */
	inline virtual ~PointXYZWithTrait(){};

	oryx_path_planning::PointTrait point_trait;	///The PointTrait associated with the point
};

/**
 * @author	Adam Panzica
 * @brief	Class which represents an Occupancy Grid that contains data on the robot's local frame
 */
class OccupancyGrid{
public:
	/**
	 * @author	Adam Panzica
	 * @brief	Default constructor
	 */
	OccupancyGrid();
	/**
	 * @author	Adam Panzica
	 * @brief	Default copy constructor
	 * @param grid The OccupancyGrid to copy
	 */
	OccupancyGrid(oryx_path_planning::OccupancyGrid& grid);
	/**
	 * @author	Adam Panzica
	 * @brief	Creates a new Occupancy Grid with a given set of dimensions and grid resolution
	 * @param xDim			The size of the occupancy grid in real units in the x-axis
	 * @param yDim			The size of the occupancy grid in real units in the y-axis
	 * @param zDim			The size of the occupancy grid in real units in the y-axis
	 * @param resolution	The grid resolution of the occupancy grid
	 * @param seedTrait		The PointTrait to initialize the values in the occupancy grid to
	 */
	OccupancyGrid(double xDim, double yDim, double zDim, double resolution, oryx_path_planning::PointTrait seedTrait);
	/**
	 * Default destructor
	 */
	virtual ~OccupancyGrid();

	/**
	 * @author	Adam Panzica
	 * @brief	Gets the PointTrait of a given point on the grid
	 * @param x	The x-coord of the point
	 * @param y	The y-coord of the point
	 * @param z The z-coord of the point
	 * @return The PointTrait of the point at the given coordinates
	 */
	oryx_path_planning::PointTrait getPointTrait(double x, double y, double z);

	/**
	 * @author	Adam Panzica
	 * @brief	Sets the PointTrait of a point on the grid
	 * @param x	The x-coord of the point
	 * @param y	The y-coord of the point
	 * @param z	The z-coord of the point
	 * @param trait	The PointTrait to set the point to
	 * @return True if succesful, else false
	 */
	bool setPointTrait(double x, double y, double z, oryx_path_planning::PointTrait trait);

	/**
	 * @author	Adam Panzica
	 * @brief	Gets the whole PointCloud which backs this occupancy grid
	 * @return	A boost::shared_pt containing the PointCloud<oryx_path_planning::PointXYZWithTrait> which backs this occupancy grid
	 */
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > getGrid();

	/**
	 * @author	Adam Panzica
	 * @brief	Generates a sensor_msgs::PointCloud2 message from the data in the grid
	 * @param message	The sensor_msgs::PointCloud2 to write the data to
	 * @return	True if successful, else false
	 */
	bool generateMessage(sensor_msgs::PointCloud2& message);

	/**
	 * @author	Adam Panzic
	 * @brief	Generates a string representation of a slice of the Occupancy Grid
	 * @param sliceAxis	The axis to create the slice along. O=x-axis, 1 = y-axis, 2=z-axis
	 * @param slice		The distance along the slice axis to make the slice
	 * @return	A shared pointer to a std::string containing an ASCII-art representation of the occupancy grid slice specified
	 */
	boost::shared_ptr<std::string> toString(int sliceAxis, double slice);
private:
	double xDim;	///The x size of this grid
	double yDim;	///The y size of this grid
	double zDim;	///The z size of this grid
	double res;		///The grid resolution of this occupancy grid
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > occGrid;		///A smart pointer to the point cloud which contains the data for this occupancy grid
};
};
#endif /* OCCUPANCYGRID_H_ */
