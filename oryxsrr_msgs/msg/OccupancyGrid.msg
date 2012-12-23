#Header
Header header

#Information about the size of the occupancy grid
float64 xDim
float64 yDim
float64 zDim
float64 res

#the origin of the grid
geometry_msgs/Point origin

#The PointCloud which backs the occupancy grid
sensor_msgs/PointCloud2 cloud

