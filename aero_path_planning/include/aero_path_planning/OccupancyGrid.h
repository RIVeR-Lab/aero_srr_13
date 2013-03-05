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
//*********************** LOCAL DEPENDENCIES ************************************//
#include <aero_path_planning/OryxPathPlanningUtilities.h>
#include <aero_path_planning/OccupancyGridMsg.h>



namespace aero_path_planning
{
//*********************** PROTOTYPES ******************************//
class OccupancyGridAccessException;
class OccupancyGrid;
//*********************** TYPEDEFS ******************************//
///Typedef to allow for convenient sharing of a OccupancyGrid via pointer
typedef boost::shared_ptr<OccupancyGrid> OccupancyGridPtr;

typedef pcl::PointCloud<Point> OccupancyGridCloud;

//*********************** CLASS DEFINITIONS ************************************//

/**
 * @author	Adam Panzica
 * @brief	Basic exception for stating that an invalid location on the occupancy grid was accessed
 */
class OccupancyGridAccessException: public ChainableException
{
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
class OccupancyGrid
{
public:
	///Typedef to make iterators easier
	typedef pcl::PointCloud<pcl::PointXYZRGBA>::iterator iterator;
	///Typedef to make const_iterators easier
	typedef pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator const_iterator;

	/**
	 * @author	Adam Panzica
	 * @brief	Default constructor
	 */
	OccupancyGrid();
	/**
	 * @author	Adam Panzica
	 * @brief	Default copy constructor
	 * @param grid The const OccupancyGrid to copy
	 */
	OccupancyGrid(const OccupancyGrid& grid);

	/**
	 * @author	Adam Panzica
	 * @brief	Creates a new 2D Occupancy Grid with a given set of dimensions and grid resolution
	 * @param xDim			The size of the occupancy grid in some integer unit in the x-axis
	 * @param yDim			The size of the occupancy grid in some integer unit the y-axis
	 * @param resolution	A conversion factor to go from the integer unit representation to an engineering unit representation
	 * @param origin		The origin of the occupancy grid, in integer unit coordinates
	 * @param seedTrait		The PointTrait to initialize the values in the occupancy grid to (defaults to UNKOWN)
	 */
	OccupancyGrid(int xDim, int yDim, double resolution, const aero_path_planning::Point& origin, PointTrait_t seedTrait=aero_path_planning::UNKNOWN);

	/**
	 * @author	Adam Panzica
	 * @brief	Creates a new Occupancy Grid with a given set of dimensions and grid resolution
	 * @param xDim			The size of the occupancy grid in some integer unit in the x-axis
	 * @param yDim			The size of the occupancy grid in some integer unit the y-axis
	 * @param zDim			The size of the occupancy grid in some integer unit the y-axis
	 * @param resolution	A conversion factor to go from the integer unit representation to an engineering unit representation
	 * @param origin		The origin of the occupancy grid, in integer unit coordinates
	 * @param origin		The origin of the occupancy grid
	 * @param seedTrait		The PointTrait to initialize the values in the occupancy grid to (defaults to UNKOWN)
	 */
	OccupancyGrid(int xDim, int yDim, int zDim, double resolution, const aero_path_planning::Point& origin, aero_path_planning::PointTrait_t seedTrait=aero_path_planning::UNKNOWN);

	/**
	 * @author	Adam Panzica
	 * @brief	Creates a new OccupancyGrid which uses a supplied PointCloud as its base
	 * @param xDim			The size of the occupancy grid in some integer unit in the x-axis
	 * @param yDim			The size of the occupancy grid in some integer unit the y-axis
	 * @param zDim			The size of the occupancy grid in some integer unit the y-axis
	 * @param resolution	A conversion factor to go from the integer unit representation to an engineering unit representation
	 * @param origin		The origin of the occupancy grid, in integer unit coordinates
	 * @param cloud			The PointCloud to use as the base for the occupancy grid
	 * @throw OccupancyGridAccessException If there is a point in the PointCloud that doesn't fit in the specified occupancy grid size
	 */
	OccupancyGrid(int xDim, int yDim, int zDim, double resolution, const aero_path_planning::Point& origin, const OccupancyGridCloud& cloud) throw(OccupancyGridAccessException);

	/**
	 * @author	Adam Panzica
	 * @brief	Creates an OccupancyGrid from an oryxsrr_msgs::OccupancyGrid
	 * @param message	Reference to the message to make the OccupancyGrid from
	 */
	OccupancyGrid(const aero_path_planning::OccupancyGridMsg& message);

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
	aero_path_planning::PointTrait getPointTrait(int x, int y, int z) const throw(OccupancyGridAccessException);

	/**
	 * @author	Adam Panzica
	 * @brief	Gets the PointTrait of a given point on the grid
	 * @param point	The coordinates of the point on the grid
	 * @return The PointTrait of the point at the given coordinates
	 * @throw OccupancyGridAccessException if invalid coordinates were given
	 */
	aero_path_planning::PointTrait getPointTrait(aero_path_planning::Point point) const throw(OccupancyGridAccessException);

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
	bool setPointTrait(int x, int y, int z, aero_path_planning::PointTrait trait) throw(OccupancyGridAccessException);

	/**
	 * @author	Adam Panzica
	 * @brief	Sets the PointTrait of a point on the grid
	 * @param point	The coordinates of the point on the grid
	 * @param trait	The PointTrait to set the point to
	 * @return True if successful, else false
	 * @throw OccupancyGridAccessException if invalid coordinates were given
	 */
	bool setPointTrait(aero_path_planning::Point point, aero_path_planning::PointTrait trait) throw(OccupancyGridAccessException);

	/**
	 * @author Adam Panzica
	 * @brief Gets the location of the goal point on the occupancy grid, if it exists
	 * @return	A Point containing the coordinates of the goal point, if it exists
	 * @throw	false if there is no goal point on the grid
	 */
	const Point& getGoalPoint() const throw (bool);

	/**
	 * @author Adam Panzica
	 * @brief Gets the location of the origin for this grid
	 * @return A reference to the origin of this grid
	 */
	const Point& getOriginPoint() const;

	/**
	 * @author Adam Panzica
	 * @brief Sets the goal point on a grid
	 * @param point The point to set as the goal
	 * @throw OccupancyGridAccessException if the point specified is not on the gird
	 */
	void setGoalPoint(aero_path_planning::Point point) throw(OccupancyGridAccessException);

	/**
	 * @author	Adam Panzica
	 * @brief	Gets the whole PointCloud which backs this occupancy grid
	 * @return	A reference to the PointCloud<aero_path_planning::PointXYZWithTrait> which backs this occupancy grid
	 */
	const OccupancyGridCloud& getGrid() const;

	/**
	 * @author	Adam Panzica
	 * @brief	Generates a sensor_msgs::PointCloud2 message from the data in the grid
	 * @param message	The sensor_msgs::PointCloud2 to write the data to
	 * @return	True if successful, else false
	 *
	 * Note that it is up to the caller to properly set the fields in the header other than the stamp
	 */
	bool generateMessage(sensor_msgs::PointCloud2Ptr message) const;

	/**
	 * @author	Adam Panzica
	 * @brief	Generates an oryxsrr_msgs::OccupancyGrid message from the OccupancyGrid
	 * @param message	The message container to fill
	 * @return	True if successful
	 *
	 * Note that it is up to the caller to properly set the fields in the header other than the stamp
	 */
	bool generateMessage(aero_path_planning::OccupancyGridMsg& message) const;

	/**
	 * @author Adam Panzica
	 * @brief Produces a sensor_msgs::Image out of the occupancy grid data, for use with visualization tools like rviz
	 * @param message Reference to a sensor_msgs::Image to build
	 * @return TRUE if successful, else false
	 *
	 * @todo Currently, it assumes a 2D, XY grid. Will still work with 3D, but will only take the XY plane at Z=0
	 */
	bool generateMessage(sensor_msgs::Image& message) const;

	/**
	 * @author Adam Panzica
	 * @brief  Gets the size of the grid in the X dimention
	 * @return The size of the grid in the X dimention, in grid units
	 */
	int getXSize() const;
	/**
	 * @author Adam Panzica
	 * @brief  Gets the size of the grid in the Y dimention
	 * @return The size of the grid in the Y dimention, in grid units
	 */
	int getYSize() const;
	/**
	 * @author Adam Panzica
	 * @brief  Gets the size of the grid in the Y dimention
	 * @return The size of the grid in the Y dimention, in grid units
	 */
	int getZSize() const;


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
	 * @author	Adam Panzica
	 * @return	A reference to an iterator at the beginning of the occupancy grid
	 */
	const_iterator cbegin() const;

	/**
	 * @author	Adam Panzica
	 * @return	A reference to an iterator at the end of the occupancy grid
	 */
	const_iterator cend() const;

	/**
	 * @author Adam Panzica
	 * @return The size of the OccupancyGrid in the form x*y*z
	 */
	unsigned long size() const;

	/**
	 * @author	Adam Panzic
	 * @brief	Generates a string representation of a slice of the Occupancy Grid
	 * @param sliceAxis	The axis to create the slice along. O=x-axis, 1 = y-axis, 2=z-axis
	 * @param slice		The distance along the slice axis to make the slice
	 * @return	A shared pointer to a std::string containing an ASCII-art representation of the occupancy grid slice specified
	 */
	boost::shared_ptr<std::string> toString(int sliceAxis, int slice) const;

	/**
	 * @author	Adam Panzica
	 * @brief	Copys the data from a point into the occupancy grid
	 * @param copy_point The point to copy into the grid
	 * @param origin_corrected True if the x/y/z coordinates in copy_ponit have been offset by the grid origin (aka all positive values)
	 */
	void setPoint(aero_path_planning::Point& copy_point, bool origin_corrected = true);
	void setPoint(const aero_path_planning::Point& copy_point, bool origin_corrected = true);

	/**
	 * @author	Adam Panzica
	 * @brief	Gets the default PointConverter that will transform between grid units and engineering units for this grid
	 * @return	A PointConverter that uses the resolution given to this OccupancyGrid as its scale factor
	 */
	const PointConverter& getConverter() const;
private:

	/**
	* @author Adam Panzica
	* @brief  Intializes the dimmensions of the grid
	*/
	void intializeDim(int x_dim, int y_dim, int z_dim);

	/**
	 * @author	Adam Panzica
	 * @brief	Checks to make sure a set of coordinates are on the occupancy grid
	 * @param point Coordinates of the point to check
	 * @throw OccupancyGridAccessException if invalid coordinates were given
	 */
	bool boundsCheck(aero_path_planning::Point& point) const throw(OccupancyGridAccessException);

	/**
	 * @author	Adam Panzica
	 * @brief	Calculates the 1-dimension index value of an <x,y,z> coordinate set based on grid values
	 * @param x	x-coord
	 * @param y	y-coord
	 * @param z	z-coord
	 * @return An index into a 1-d array corresponding to the given coord set
	 */
	int calcIndex(int x, int y, int z) const;

	/**
	 * Gets a point out of the point cloud based on real coordinates
	 * @param x	x-coord
	 * @param y	y-coord
	 * @param z	z-coord
	 * @return The point at the given coordinate
	 */
	Point& getPoint(aero_path_planning::Point& point , bool origin_corrected = true);
	Point& getPoint(const aero_path_planning::Point& point , bool origin_corrected = true);
	const Point& getPoint(aero_path_planning::Point& point , bool origin_corrected = true) const;
	const Point& getPoint(const aero_path_planning::Point& point , bool origin_corrected = true) const;

	/**
	 * Gets a point out of the point cloud based on integer coordinates
	 * @param x	x-coord
	 * @param y	y-coord
	 * @param z	z-coord
	 * @return The point at the given coordinate
	 */
	Point& getPoint(int x, int y, int z);
	const Point& getPoint(int x, int y, int z) const;

	/**
	 * @author Adam Panzica
	 * @brief Helper function that initializes the occupancy grid
	 * @param seedTrait The PointTrait to set all of the points to
	 */
	void intializeGrid(PointTrait_t seedTrait);

	/**
	 * @author	Adam Panzica
	 * @brief	Helper function which searches for a goal point in the grid
	 */
	void searchForGoal();

	int x_dim_;	///The x dimension of this grid
	int y_dim_;	///The y dimension of this grid
	int z_dim_;	///The z dimension of this grid
	double res_;	///The grid resolution of this occupancy grid
	bool has_goal_;						///Flag to signal there is a goal point on the grid
	aero_path_planning::Point origin_;	///The origin of the occupancy grid
	aero_path_planning::Point goal_;		///The location of the goal point on the grid
	OccupancyGridCloud occ_grid_;			///The point cloud which contains the data for this occupancy grid
	aero_path_planning::PointConverter converter_;	///Used to convert the internal integer units to output engineering units
};


};
#endif /* OCCUPANCYGRID_H_ */
