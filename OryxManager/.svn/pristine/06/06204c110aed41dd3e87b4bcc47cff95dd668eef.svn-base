/*
 * InertiaCube.cpp
 *
 *  Created on: Feb 14, 2012
 *      Author: oryx
 */


#include "Kinematics/InertiaCube.h"

void reconfigureCallback(OryxManager::IMUConfig &config, uint32_t level){
	if(config.Compass !=Compass){
		Compass = config.Compass;
		station.Compass=Compass;
	}
	if(config.Enhancement !=Enhancement){
		Enhancement=config.Enhancement;
		station.Enhancement=Enhancement;
	}
	if(config.Prediction != Prediction){
		Prediction=config.Prediction;
		station.Prediction = Prediction;
	}
	if(config.Timestamped != Timestamped){
		Timestamped=config.Timestamped;
		station.TimeStamped=Timestamped;
	}
	ISD_SetStationConfig(handle,&station,1,FALSE);
}

sensor_msgs::Imu getIMUData(){
	ISD_GetTrackingData(handle, &data);

	sensor_msgs::Imu imu_msg;

	double w 		= data.Station[0].Quaternion[0];
	double x 		= -data.Station[0].Quaternion[1];
	double y 		= data.Station[0].Quaternion[2];
	double z 		= -data.Station[0].Quaternion[3];
	imu_msg.header.frame_id = "imu";
	imu_msg.orientation.w 			= w;
	imu_msg.orientation.x 			= x;
	imu_msg.orientation.y 			= y;
	imu_msg.orientation.z 			= z;
	imu_msg.angular_velocity.x 		= data.Station[0].AngularVelBodyFrame[0];
	imu_msg.angular_velocity.y 		= data.Station[0].AngularVelBodyFrame[1];
	imu_msg.angular_velocity.z 		= data.Station[0].AngularVelBodyFrame[2];

	/*
	 * The linear acceleration data is useless for this IMU. Instead, we are
	 * filling it with the euler angles in degrees because it makes parsing
	 * angles easier. We did not want to convert between quaternions and euler.
	 */
	//TODO: Change back to linear acceleration.
	imu_msg.linear_acceleration.x 	= data.Station[0].Euler[2];
	imu_msg.linear_acceleration.y	= data.Station[0].Euler[1];
	imu_msg.linear_acceleration.z	= data.Station[0].Euler[0];

	imu_msg.orientation_covariance[0] = .01;
	imu_msg.orientation_covariance[3] = .01;
	imu_msg.orientation_covariance[6] = .01;

	imu_msg.linear_acceleration_covariance[0]=999999;
	imu_msg.linear_acceleration_covariance[3]=999999;
	imu_msg.linear_acceleration_covariance[6]=999999;

	imu_msg.angular_velocity_covariance[0]=1;
	imu_msg.angular_velocity_covariance[3]=1;
	imu_msg.angular_velocity_covariance[6]=1;
	imu_msg.header.stamp = ros::Time::now();
	return imu_msg;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "InertiaCube");
	ros::NodeHandle n;

	tf::TransformBroadcaster imu_tf_broadcaster;
	ros::Publisher imu_data_publisher = n.advertise<sensor_msgs::Imu> ("\imu", 1);

	ros::param::get("~Publish_Rate",publishRate);
	ros::Rate loopRate(publishRate);

	ISD_OpenAllTrackers(NULL,&handle,FALSE,FALSE);
	if (handle <= 0) {
		ROS_ERROR("Tracker Not Found. Try a Different Port");
	}
	else{
		ROS_INFO("Tracker Found");
	ISD_GetStationConfig(handle,&station,1,FALSE);

	if(n.hasParam("Compass")){
		n.getParam("Compass",Compass);
		station.Compass=Compass;
	}
	if(n.hasParam("Timestamped")){
		n.getParam("Timestamped",Timestamped);
		station.TimeStamped=Timestamped;
	}
	if(n.hasParam("Compass_Compensation")){
		n.getParam("Compass_Compensation",CompassCompensation);
		station.CompassCompensation=CompassCompensation;
	}
	if(n.hasParam("Enhancement")){
		n.getParam("Enhancement",Enhancement);
		station.Enhancement=Enhancement;
	}
	if(n.hasParam("Sensitivity")){
		n.getParam("Sensitivity",Sensitivity);
		station.Sensitivity=Sensitivity;
	}
	if(n.hasParam("Prediction")){
		n.getParam("Prediction", Prediction);
		station.Prediction=Prediction;
	}

	dynamic_reconfigure::Server<OryxManager::IMUConfig> server;
	dynamic_reconfigure::Server<OryxManager::IMUConfig>::CallbackType f;

	f = boost::bind(reconfigureCallback, _1 ,_2);
	server.setCallback(f);


	ISD_SetStationConfig(handle,&station,1,FALSE);
	ISD_ResetHeading(handle, 0);

	while(n.ok()){
		sensor_msgs::Imu imu_msg = getIMUData();
		imu_data_publisher.publish(getIMUData());
		geometry_msgs::TransformStamped imu_trans;
		imu_trans.header.stamp = imu_msg.header.stamp;
		imu_trans.header.frame_id = "world";
		imu_trans.child_frame_id = "imu";

		imu_trans.transform.translation.x = 0.0;
		imu_trans.transform.translation.y = 0.0;
		imu_trans.transform.translation.z = 0.0;
		imu_trans.transform.rotation.w = imu_msg.orientation.w;
		imu_trans.transform.rotation.x = imu_msg.orientation.x;
		imu_trans.transform.rotation.y = imu_msg.orientation.y;
		imu_trans.transform.rotation.z = imu_msg.orientation.z;
		imu_tf_broadcaster.sendTransform(imu_trans);
		ros::spinOnce();
		loopRate.sleep();
	}

	ISD_CloseTracker(handle);
	}


}
