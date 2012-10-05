
#include "DriveManager.h"
#include <ros/console.h>

#include <unistd.h>

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
	if (msg->buttons[LEFT_BUTTON] <.1 && msg->buttons[RIGHT_BUTTON] <.1) {
		EposManager::GroupEPOSControl drive_msg;
		float scale = 1;


		//If one of the triggers is held, 1/2 speed
		 if ((msg->axes[LEFT_TRIGGER] >= -.5) || (msg->axes[RIGHT_TRIGGER]>= -.5))scale = .5;
		 if ((msg->axes[LEFT_TRIGGER] >= -.5) && (msg->axes[RIGHT_TRIGGER]	>= -.5))scale = .125;

		 //Initiate Countdown on A Button
		 if (msg->buttons[A_BUTTON] > 0){
			 system("sh /home/oryx/Desktop/speech &");
		 }

		 //Drive Straight Forward
		if (msg->axes[D_PAD_UP_DOWN] > 0){
			drive_msg = joyToDriveMessage(1, 1, scale);
		}
		//Drive Straight Backward
		else if (msg->axes[D_PAD_UP_DOWN] < 0){
			drive_msg = joyToDriveMessage(-1, -1, scale);
		}
		//Full Left Turn
		else if (msg->axes[D_PAD_LEFT_RIGHT] > 0){
			drive_msg = joyToDriveMessage(-1, 1, scale);
		}
		//Full Right Turn
		else if (msg->axes[D_PAD_LEFT_RIGHT] < 0){
			drive_msg = joyToDriveMessage(1, -1, scale);
		}
		//Else, use joy sticks
		else drive_msg = joyToDriveMessage(msg->axes[LEFT_VERTICAL_AXIS],msg->axes[RIGHT_VERTICAL_AXIS], scale);
		group_drive_publisher.publish(drive_msg);
	}
}


void motorInfoCallback(const EposManager::GroupMotorInfo::ConstPtr& msg){
	//TODO: Implement odometry
	vector<EposManager::MotorInfo> motor_group;
		motor_group = msg->motor_group;
		for(int i=0; i< motor_group.size();i++){

		}
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
	EposManager::GroupEPOSControl drive_msg;
	EposManager::EPOSControl front_right,front_left,back_right,back_left;

	front_right.node_id=FRONT_RIGHT_WHEEL;
	back_right.node_id=BACK_RIGHT_WHEEL;
	front_left.node_id=FRONT_LEFT_WHEEL;
	back_left.node_id=BACK_LEFT_WHEEL;

	front_right.control_mode=EposManager::EPOSControl::VELOCITY;
	back_right.control_mode=EposManager::EPOSControl::VELOCITY;
	front_left.control_mode=EposManager::EPOSControl::VELOCITY;
	back_left.control_mode=EposManager::EPOSControl::VELOCITY;

	double angularToLinearCoef = (baseWidth*baseWidth + baseLength*baseLength)/(2*baseWidth);
	long left_speed  = velocityToRPM(msg->linear.x - msg->angular.z*angularToLinearCoef);
	long right_speed = -velocityToRPM(msg->linear.x + msg->angular.z*angularToLinearCoef);

	front_right.setpoint=right_speed;
	back_right.setpoint=right_speed;
	front_left.setpoint=left_speed;
	back_left.setpoint=left_speed;

	drive_msg.motor_group.push_back(front_right);
	drive_msg.motor_group.push_back(front_left);
	drive_msg.motor_group.push_back(back_left);
	drive_msg.motor_group.push_back(back_right);

	group_drive_publisher.publish(drive_msg);
}



int main (int argc, char **argv){
	ros::init(argc, argv, "OryxManager");
	ros::NodeHandle n;

	ros::param::get("~Max_Velocity", maxVelocity);
	ros::param::get("~Base_Width", baseWidth);
	ros::param::get("~Base_Length", baseLength);

	maxRPM = velocityToRPM(maxVelocity);
	ros::Subscriber joy_subscriber = n.subscribe("joy", 1, joyCallback);
	ros::Subscriber driver_joy_subscriber = n.subscribe("DriverJoy", 1, joyCallback);
	ros::Subscriber drive_motor_twist_subscriber = n.subscribe("motors/Drive_Motors/Twist",1,twistCallback);
	group_drive_publisher = n.advertise<EposManager::GroupEPOSControl>("motors/Drive_Motors/Group_Motor_Control", 1);

	dynamic_reconfigure::Server<OryxManager::DriveManagerConfig> server;
	dynamic_reconfigure::Server<OryxManager::DriveManagerConfig>::CallbackType f;

	//f = boost::bind(reconfigureCallback, _1 ,_2);
//	server.setCallback(f);

	ros::spin();
}

int velocityToRPM(float velocity){
	velocity = velocity/WHEEL_DIAMETER/3.14159265*60*GEAR_RATIO;
	return (int) velocity;
}

EposManager::GroupEPOSControl joyToDriveMessage(float left_value, float right_value,float scale){
	EposManager::GroupEPOSControl drive_msg;
	EposManager::EPOSControl front_right, front_left, back_right, back_left;

	front_right.node_id=FRONT_RIGHT_WHEEL;
	back_right.node_id=BACK_RIGHT_WHEEL;
	front_left.node_id=FRONT_LEFT_WHEEL;
	back_left.node_id=BACK_LEFT_WHEEL;

	front_right.control_mode=EposManager::EPOSControl::VELOCITY;
	back_right.control_mode=EposManager::EPOSControl::VELOCITY;
	front_left.control_mode=EposManager::EPOSControl::VELOCITY;
	back_left.control_mode=EposManager::EPOSControl::VELOCITY;

	long left_speed,right_speed;
	left_speed  = left_value*maxRPM*scale;
	right_speed = -right_value*maxRPM*scale;

	if(abs(left_speed) < 600) left_speed=0;
	if(abs(right_speed) < 600) right_speed=0;
	front_right.setpoint=right_speed;
	back_right.setpoint=right_speed;
	front_left.setpoint=left_speed;
	back_left.setpoint=left_speed;

	drive_msg.motor_group.push_back(front_right);
	drive_msg.motor_group.push_back(front_left);
	drive_msg.motor_group.push_back(back_left);
	drive_msg.motor_group.push_back(back_right);

	return drive_msg;
}
