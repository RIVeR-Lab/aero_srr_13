#Message for sending an OccupancyGrid

#Header
Header header

#Grid Data
sensor_msgs/PointCloud2 grid_data

#Grid information
int32 x_dim
int32 y_dim
int32 z_dim
float64 resolution

#Origin Information
int32 x_origin
int32 y_origin
int32 z_origin

#Goal Information
bool has_goal

int32 x_goal
int32 y_goal
int32 z_goal

