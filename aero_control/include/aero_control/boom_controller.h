/*
 * boom_controller.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mdedonato
 */

#ifndef BOOM_CONTROLLER_H_
#define BOOM_CONTROLLER_H_

/* Define to debug without arm */
//#define DEBUG_WITHOUT_ARM
//#define PRINT_DEBUG_INFO
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <time.h>
#include <aero_srr_msgs/AeroState.h>
#include <aero_srr_msgs/StateTransitionRequest.h>
#include <aero_base/SetBoomPosition.h>

namespace aero_control
{

#define MAX_BOOM_PATH_STEPS 10

	class BoomController
	{
		public:
			BoomController(ros::NodeHandle nh, ros::NodeHandle param_nh);

		private:
			void SendBoomControl(aero_base::SetBoomPosition boom_position);
			void GoToPosition(double angle, double velocity,double delay);
			void GoHome(void);
			void AeroStateMSG(const aero_srr_msgs::AeroStateConstPtr& aero_state);

			typedef struct
			{
					double angle;
					double velocity;
					double delay;
			} boom_path_step_t;

			boom_path_step_t boom_path[MAX_BOOM_PATH_STEPS];

			int boom_path_steps;

			inline bool SetBoomPathStep(double angle, double velocity, double delay)
			{

				if (this->boom_path_steps < MAX_BOOM_PATH_STEPS)
				{
					/* Finger motion */
					this->boom_path[boom_path_steps].angle = angle;
					this->boom_path[boom_path_steps].velocity = velocity;
					this->boom_path[boom_path_steps].delay = delay;

					this->boom_path_steps++;
					return true;
				} else
				{
					ROS_WARN("Too many path points, you need to increase the maximum number of path points!");
					return false;

				}

			}

			/* Set path here */
			inline void PlanHorizontalPath(void)
			{
				boom_path_steps = 0;

				SetBoomPathStep(180,0.1,0.5);
				SetBoomPathStep(-180,0.1,0.5);

			}
			uint8_t boom_path_step_num;

			ros::Subscriber aero_state_sub;
			ros::ServiceClient aero_state_transition_srv_client;
			ros::ServiceClient boom_control_srv_client;
			bool active_state;
			bool pause_state;

	};

}

#endif /* BOOM_CONTROLLER_H_ */
