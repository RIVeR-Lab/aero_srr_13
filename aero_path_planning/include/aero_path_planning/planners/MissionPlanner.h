/**
 * @file   MissionPlanner.h
 *
 * @date   May 10, 2013
 * @author parallels
 * @brief  \TODO
 */

#ifndef MISSIONPLANNER_H_
#define MISSIONPLANNER_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
//************ LOCAL DEPENDANCIES ****************//

//***********    NAMESPACES     ****************//

namespace aero_path_planning
{

/**
 * @author Adam Panzica
 * @brief Class for managing mission level tasks
 *
 * MissionPlanner is responsable for generating/monitoring mission level tasks. It generates mission goals for GlobalPlanner to use when re-planning. It consumes the CarrotPath produced by GlobalPlanenr and ensures that the goal point
 * used by LocalPlanner is updated correctly based on the current location of the robot. It also handles managing object of interest detections.
 */
class MissionPlanner
{
private:

public:
	/**
	 * @author Adam Panzica
	 * @param nh global node handle
	 * @param p_nh private node handle
	 */
	MissionPlanner(ros::NodeHandle& nh, ros::NodeHandle& p_nh);
};

};

#endif /* MISSIONPLANNER_H_ */
