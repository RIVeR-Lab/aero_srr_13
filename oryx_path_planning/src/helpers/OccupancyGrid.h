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
#include <oryxsrr_msgs/OccupancyGrid.h>
//*********************** LOCAL DEPENDENCIES ************************************//
#include "OryxPathPlanningUtilities.h"



namespace oryx_path_planning{
//*********************** PROTOTYPES ******************************//
class OccupancyGridAccessException;
class OccupancyGrid;
//*********************** TYPEDEFS ******************************//
///Typedef to allow for convenient sharing of a OccupancyGrid via pointer
typedef boost::shared_ptr<OccupancyGrid> OccupancyGridPtr;

//*********************** CLASS DEFINITIONS ************************************//
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
	GOAL            = 0xFFFF00,	//!< GOAL			Point on the grid is the goal position (Yellow)
	TENTACLE		= 0xCD00CD	//!< TENTACLE		Point on the grid is a Tentacle marker (Pink)
} PointTrait;

/**
 * @author	Adam Panzica
 * @brief	Basic exception for stating that an invalid location on the occupancy grid was accessed
 */
class OccupancyGridAccessException: public ChainableException{
public:
	/**
	 * @author	Adam Panzica
	 * @brief	Default constructor
	 */
	inline OccupancyGridAccessException():ChainableException(){};
	/**
	 * @author	Adam Panzica
	 * @brief	Creates an exception with a message describing it
	 * @param message A descriptive message for the error
	 */
	inline OccupancyGridAccessException(std::string& message):ChainableException(message){};
	/**
	 * @author	Adam Panzica
	 * @brief	Creates an exception with a message and a cause
	 * @param message	A descriptive message for the error
	 * @param cause		The cause of the exception
	 */
	inline OccupancyGridAccessException(std::string& message, std::exception& cause): ChainableException(message, cause){};
};

/**
 * @author	Adam Panzica
 * @brief	Class which represents an Occupancy Grid that contains data on the robot's local frame
 */
class OccupancyGrid{
public:
	///Typedef to make iterators easier
	typedef pcl::PointCloud<pcl::PointXYZRGBA>::iterator iterator;

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
	OccupancyGrid(OccupancyGrid& grid);

	/**
	 * @author	Adam Panzica
	 * @brief	Creates a new 2D Occupancy Grid with a given set of dimensions and grid resolution
	 * @param xDim			The size of the occupancy grid in real units in the x-axis
	 * @param yDim			The size of the occupancy grid in real units in the y-axis
	 * @param resolution	The grid resolution of the occupancy grid
	 * @param origin		The origin of the occupancy grid
	 * @param seedTrait		The PointTrait to initialize the values in the occupancy grid to (defaults to UNKOWN)
	 */
	OccupancyGrid(double xDim, double yDim, double resolution, const oryx_path_planning::Point& origin, PointTrait_t seedTrait=oryx_path_planning::UNKNOWN);

	/**
	 * @author	Adam Panzica
	 * @brief	Creates a new Occupancy Grid with a given set of dimensions and grid resolution
	 * @param xDim			The size of the occupancy grid in real units in the x-axis
	 * @param yDim			The size of the occupancy grid in real units in the y-axis
	 * @param zDim			The size of the occupancy grid in real units in the z-axis
	 * @param resolution	The grid resolution of the occupancy grid
	 * @param origin		The origin of the occupancy grid
	 * @param seedTrait		The PointTrait to initialize the values in the occupancy grid to (defaults to UNKOWN)
	 */
	OccupancyGrid(double xDim, double yDim, double zDim, double resolution, const oryx_path_planning::Point& origin, oryx_path_planning::PointTrait_t seedTrait=oryx_path_planning::UNKNOWN);

	/**
	 * @author	Adam Panzica
	 * @brief	Creates a new OccupancyGrid which uses a supplied PointCloud as its base
	 * @param xDim			The size of the occupancy grid in real units in the x-axis
	 * @param yDim			The size of the occupancy grid in real units in the y-axis
	 * @param zDim			The size of the occupancy grid in real units in the z-axis. For a 2D PointCloud, specify a zDim of Zero and insure that all z-values in the XYZ data of the points are 0.
	 * @param resolution	The grid resolution of the occupancy grid
	 * @param origin		The origin of the occupancy grid
	 * @param cloud			The PointCloud to use as the base for the occupancy grid
	 * @throw OccupancyGridAccessException If there is a point in the PointCloud that doesn't fit in the specified occupancy grid size
	 */
	OccupancyGrid(double xDim, double yDim, double zDim, double resolution, const oryx_path_planning::Point& origin, PointCloudPtr cloud) throw(OccupancyGridAccessException);

	/**
	 * @author	Adam Panzica
	 * @brief	Creates an OccupancyGrid from an oryxsrr_msgs::OccupancyGrid
	 * @param message	The message to make the OccupancyGrid from
	 */
	OccupancyGrid(oryxsrr_msgs::OccupancyGridPtr message);

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
	 * @throw OccupancyGridAccessException if invalid coordinates were given
	 */
	oryx_path_planning::PointTrait getPointTrait(double x, double y, double z) throw(OccupancyGridAccessException);

	/**
	 * @author	Adam Panzica
	 * @brief	Gets the PointTrait of a given point on the grid
	 * @param point	The coordinates of the point on the grid
	 * @return The PointTrait of the point at the given coordinates
	 * @throw OccupancyGridAccessException if invalid coordinates were given
	 */
	oryx_path_planning::PointTrait getPointTrait(oryx_path_planning::Point point)throw(OccupancyGridAccessException);

	/**
	 * @author	Adam Panzica
	 * @brief	Sets the PointTrait of a point on the grid
	 * @param x	The x-coord of the point
	 * @param y	The y-coord of the point
	 * @param z	The z-coord of the point
	 * @param trait	The PointTrait to set the point to
	 * @return True if successful, else false
	 * @throw OccupancyGridAccessException if invalid coordinates were given
	 */
	bool setPointTrait(double x, double y, double z, oryx_path_planning::PointTrait trait) throw(OccupancyGridAccessException);

	/**
	 * @author	Adam Panzica
	 * @brief	Sets the PointTrait of a point on the grid
	 * @param point	The coordinates of the point on the grid
	 * @param trait	The PointTrait to set the point to
	 * @return True if successful, else false
	 * @throw OccupancyGridAccessException if invalid coordinates were given
	 */
	bool setPointTrait(oryx_path_planning::Point point, oryx_path_planning::PointTrait trait) throw(OccupancyGridAccessException);

	/**
	 * @author	Adam Panzica
	 * @brief	Gets the whole PointCloud which backs this occupancy grid
	 * @return	A boost::shared_pt containing the PointCloud<oryx_path_planning::PointXYZWithTrait> which backs this occupancy grid
	 */
	PointCloudPtr getGrid();

	/**
	 * @author	Adam Panzica
	 * @brief	Generates a sensor_msgs::PointCloud2 message from the data in the grid
	 * @param message	The sensor_msgs::PointCloud2 to write the data to
	 * @return	True if successful, else false
	 *
	 * Note that it is up to the caller to properly set the fields in the header other than the stamp
	 */
	bool generateMessage(sensor_msgs::PointCloud2Ptr message);

	/**
	 * @author	Adam Panzica
	 * @brief	Generates an oryxsrr_msgs::OccupancyGrid message from the OccupancyGrid
	 * @param message	The message container to fill
	 * @return	True if successful
	 *
	 * Note that it is up to the caller to properly set the fields in the header other than the stamp
	 */
	bool generateMessage(oryxsrr_msgs::OccupancyGridPtr message);

	/**
	 * @author	Adam Panzica
	 * @return	A reference to an iterator at the beginning of the occupancy grid
	 */
	iterator begin();

	/**
	 * @author	Adam Panzica
	 * @return	A reference to an iterator at the end of the occupancy grid
	 */
	iterator end();

	/**
	 * @author	Adam Panzic
	 * @brief	Generates a string representation of a slice of the Occupancy Grid
	 * @param sliceAxis	The axis to create the slice along. O=x-axis, 1 = y-axis, 2=z-axis
	 * @param slice		The distance along the slice axis to make the slice
	 * @return	A shared pointer to a std::string containing an ASCII-art representation of the occupancy grid slice specified
	 */
	boost::shared_ptr<std::string> toString(int sliceAxis, double slice);
private:

	/**
	 * @author	Adam Panzica
	 * @brief	Checks to make sure a set of coordinates are on the occupancy grid
	 * @param point Coordinates of the point to check
	 * @throw OccupancyGridAccessException if invalid coordinates were given
	 */
	bool boundsCheck(oryx_path_planning::Point& point)throw(OccupancyGridAccessException);

	/**
	 * @author	Adam Panzica
	 * @brief	Calculates the 1-dimension index value of an <x,y,z> coordinate set based on real values
	 * @param x	x-coord
	 * @param y	y-coord
	 * @param z	z-coord
	 * @return An index into a 1-d array corresponding to the given coord set
	 */
	int calcIndex(double x, double y, double z);

	/**
	 * @author	Adam Panzica
	 * @brief	Calculates the 1-dimension index value of an <x,y,z> coordinate set based on grid values
	 * @param x	x-coord
	 * @param y	y-coord
	 * @param z	z-coord
	 * @return An index into a 1-d array corresponding to the given coord set
	 */
	int calcIndex(int x, int y, int z);

	/**
	 * Gets a point out of the point cloud based on real coordinates
	 * @param x	x-coord
	 * @param y	y-coord
	 * @param z	z-coord
	 * @return The point at the given coordinate
	 */
	Point& getPoint(oryx_path_planning::Point& point);

	/**
	 * Gets a point out of the point cloud based on integer coordinates
	 * @param x	x-coord
	 * @param y	y-coord
	 * @param z	z-coord
	 * @return The point at the given coordinate
	 */
	Point& getPoint(int x, int y, int z);

	/**
	 * @author Adam Panzica
	 * @brief Helper function that initializes the occupancy grid
	 * @param seedTrait The PointTrait to set all of the points to
	 */
	void intializeGrid(PointTrait_t seedTrait);

	double xDim;	///The x dimension of this grid
	double yDim;	///The y dimension of this grid
	double zDim;	///The z dimension of this grid
	int xSize;		///The x size of this grid
	int ySize;		///The y size of this grid
	int zSize;		///The z size of this grid
	double res;		///The grid resolution of this occupancy grid
	int x_ori;		///The x origin of the grid in grid units
	int y_ori;		///The y origin of the grid in grid units
	int z_ori;		///The z origin of the grid in grid units
	oryx_path_planning::Point origin;		///The origin of the occupancy grid
	PointCloudPtr occGrid;	///A smart pointer to the point cloud which contains the data for this occupancy grid
};


};
#endif /* OCCUPANCYGRID_H_ */
