#Message for passing MultiTraitOccupancyGrids between nodes
Header header

#Each grid corrisponds to a trait
nav_msgs/OccupancyGrid[] trait_grids

#Location of the goal point on the grid
geometry_msgs/Pose goal
bool has_goal

#Vector of trait types, in order of appearance in trait_grids with 0 being the quick-access grid
int32[] trait_vector