/*
 * BeaconDetector.cpp
 *
 *  Created on: May 6, 2013
 *      Author: bpwiselybabu
 */

#include <beacon_detect/BeaconDetector.h>

#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>


using namespace cv;

BeaconDetector::BeaconDetector():it_(nh_)
{
	getRosParam();										//initialize all the ROS Parameters
	active_=false;										//keep the beacon detector inactive by default]
	init_=true;											//by default the tag system is not initialized,
														//you need to find the tag_base to the world

	//the robot state controls the state of this node to init_/active_
	//TODO: can you not listen to the robot states????
	ros::Subscriber sub = nh_.subscribe(robot_topic_.c_str(), 5, &BeaconDetector::systemCb, this);

	/* Start the camera video from the camera topic */
	if(!cam_topic_.empty())
	{
		subImg_ = it_.subscribeCamera(cam_topic_.c_str(), 1, &BeaconDetector::imageCb, this);
		ROS_INFO("Subscribing to: %s\n",cam_topic_.c_str());
	}
	else
		return;


	/* Set up the publisher for the result stamped pose */
	pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tag_visualization", 5);

	/*set up the publisher for the odom so the ekf filter can use it as visual odometry, need to remap it to /vo */
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("tag_odom",5);

	/*set up the client for the robot state service */
	state_client_ =	nh_.serviceClient<aero_srr_msgs::StateTransitionRequest>("aero/supervisor/state_transition_request");

	/*set up the action client for rotating the boom */
	//boom_client_ = boost::shared_ptr<BoomClient>(new BoomClient(nh_, "/camera_boom_control", true));

	/* rotate boom 360 */
	//rotateBoom();
}
void BeaconDetector::rotateBoom()
{
	device_driver_base::SetJointPositionGoal boom_position;
	boom_position.angle = 180;
	boom_position.max_velocity = 0.1;
	boom_client_->sendGoalAndWait(boom_position);
}
void BeaconDetector::imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info)
{
	if(!active_&&!init_)
		return;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
	   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
	}
	colorImg_=cv_ptr->image;

	double fx=cam_info->K[0];
	double fy=cam_info->K[4];
	double px=cam_info->K[2];
	double py=cam_info->K[5];

	std_msgs::Header img_header=cv_ptr->header; //the image header

	tf::Transform transform;									//the transform between the tag and the camera
	tf::Stamped<tf::Transform> tfbaseinworld;
	AprilTags::TagDetector tag_detector(AprilTags::tagCodes36h11);// the tag detector not this is set to a specific family TODO: expose this as a dynamic reconfig

	Mat gray;
	cv::cvtColor(colorImg_, gray, CV_BGR2GRAY);						//convert the color Image to gray imagePr

	detections_ = tag_detector.extractTags(gray);	//get tags if detected

	ROS_INFO("%d tag detected. \n",(int)detections_.size());									//DEBUG information
	if(detections_.size()==0)
		return;

	if(init_)
	{
		//calculate the average tf of the world location using the detections_
		tfbaseinworld=initProcess(fx,fy,px,py,img_header);
		//till test period is over
		if(!test_)
		{
			boost::thread world_broadcaster_( boost::bind( &BeaconDetector::publishWorld, this,  tfbaseinworld) );

			aero_srr_msgs::StateTransitionRequest state_transition;
			state_transition.request.requested_state.state = aero_srr_msgs::AeroState::SEARCH;
			state_transition.request.requested_state.header.stamp = ros::Time().now();

			if(state_client_.call(state_transition))
			{
				if(state_transition.response.success)
				{
					ROS_INFO("Aero successfully transitioned to the Search mode");
					active_=true;
					init_=false;
				}
				else
				{
					ROS_ERROR("%s",state_transition.response.error_message.c_str());
				}
			}
			else
			{
				ROS_ERROR("Aero state service is not running! Please start it");
			}
		}
	}
	if(active_)
	{
		runProcess(fx,fy,px,py,img_header);
	}
	if(show_)
		showResult(colorImg_);
}

void BeaconDetector::initConfiguration(string fname)
{
	FileStorage fs(fname,FileStorage::READ);
	if(!fs.isOpened())
	{
		ROS_ERROR("Cant open Configuration file!");
	}
	int no=fs["tag_no"];			//get the no of tags from the opencv xml file
	string name="tag";
	string id="tag_id";
	string type="tag_type";


	pair<int,string > temp;				//the temp pair to load
	int temp_type;						//the temp type to load
	for(int i=0;i<no;i++)
	{
		char tname[20];
		sprintf(tname,"%s%d",name.c_str(),i);
		temp.second=string(fs[tname]);
		sprintf(tname,"%s%d",id.c_str(),i);
		temp.first=fs[tname];
		constellation_.push_back(temp);
		sprintf(tname,"%s%d",type.c_str(),i);
		temp_type=fs[tname];
		tag_type_.push_back((temp_type>0)?true:false);
	}
}
void BeaconDetector::getRosParam()
{
	//Options to/exposed
	//cam_topic 		- the topic which has the camera images with the camera info streaming
	//tag_size			- the size of the tag in m
	//configuration		- configuration file name for determining the tag constelation
	//histeq			- histogram equalization or not
	//show_result 		- show results on screen or not
	//robot_topic		- system status message topic

	ros::NodeHandle pnh("~");							//handle to the local param list
	if(!pnh.getParam("cam_topic", cam_topic_))				//initialize var from launch file
	{
		ROS_ERROR("cam_topic not set in launch file\n");
	}
	if(!pnh.getParam("tag_size_big", tag_size_big_))
	{
		ROS_ERROR("Big tag size not set, will use 0.166");
		tag_size_big_ = 0.166;								//this is the default value when you print on A4
	}
	if(!pnh.getParam("tag_size_small", tag_size_small_))
	{
		ROS_ERROR("small tag size not set, will use 0.166");
		tag_size_small_ = 0.166;								//this is the default value when you print on A4
	}
	string cfile;
	if(!pnh.getParam("configuration",cfile ))			//the prefix for the tag frame
	{
		ROS_ERROR("Without specifing the configuration file that describes how the tags are arranged wrt to the world, this node will not work");
	}
	std::string path = ros::package::getPath("beacon_detect");
	initConfiguration(path+"/"+cfile);

	if(!pnh.getParam("robot_topic", robot_topic_))			//the prefix for the tag frame
	{
		ROS_WARN("The topic which publishes the robot state was not set");
	}
	if(!pnh.getParam("histeq", histeq_))						//initialize histogram option from launch file
	{
		ROS_WARN("By default histogram equalization is on\n");
		histeq_=true;
	}
	if(!pnh.getParam("show_result", show_))						//initialize if the result needs to be displayed
	{
		ROS_WARN("By default the result is not displayed as image n\n");
		show_=false;
	}
	if(!pnh.getParam("test_mode", test_))						//provision for test mode
	{
		ROS_WARN("By default the test mode is not on and the node expects robot state messages\n");
		test_=false;
	}
}
void BeaconDetector::systemCb(const aero_srr_msgs::AeroStateConstPtr& status)
{
	ROS_INFO("System state message recieved");

	if(status->state==aero_srr_msgs::AeroState::STARTUP)
	{
		init_=true;								//startup is the initialization stage
	}
	else
	{
		init_=false;							//dont initalize if the robot is in any other stage
	}
}
void BeaconDetector::histEq(cv::Mat &frame)
{
	Mat ycrcb;
	cvtColor(frame,ycrcb,CV_BGR2YCrCb);

	vector<Mat> channels;
	split(ycrcb,channels);
	equalizeHist(channels[0], channels[0]);

	Mat result;
	merge(channels,ycrcb);
	cvtColor(ycrcb,result,CV_YCrCb2BGR);

	frame=result;
}
string BeaconDetector::checkInConst(AprilTags::TagDetection tag, bool &tag_type)
{

	for(size_t i=0;i<constellation_.size();i++)	//loop through the tags in the consellation
	{
		if(tag.id==constellation_[i].first)		//if the id match then declare true
		{
			tag_type=tag_type_[i];
			return constellation_[i].second;						//return the tag id so you can do search and display
		}
	}
	return string();								//return empty string if its not in the constellation
}
void BeaconDetector::publishWorld(tf::Stamped<tf::Transform>  tf)
{
	while(ros::ok())
	{
		br_.sendTransform(tf::StampedTransform(tf, ros::Time::now()+ros::Duration(0.5), "/world","/tag_base"));
	}
	if(world_broadcaster_.joinable())
	{
		world_broadcaster_.join();
	}
}

void BeaconDetector::runProcess(double fx,double fy, double px, double py, std_msgs::Header imgheader)
{
	ROS_INFO("In processing");

	string tag;										//the tag name if it is valid
	bool tag_type;									//if small use the small size for cal else use big

	geometry_msgs::Pose tag_pose;					//the pose of the tag
	tf::Stamped<tf::Transform> 	world2base;


	geometry_msgs::PoseStamped res;					//to find the average pose that the robot needs to move in

	int sctr=0;										//the neumber of valid tags detected
	int bctr=0;										//the number of big tags detected

	for (size_t i=0; i<detections_.size(); i++)
	{
		tag=checkInConst(detections_[i],tag_type);			//check if it is the constellation if so whats is its name in home tf tree?
		if(tag.empty())
		{
			ROS_INFO("Found a tag not in the tag constellation");
			continue;
		}

		ROS_INFO( "  Id: %d --- Hamming distance: %d \n",detections_[i].id,  detections_[i].hammingDistance );

		// recovering the relative pose requires camera calibration;
		Eigen::Matrix4d T;
		if(tag_type)
		{
			T = detections_[i].getRelativeTransform(tag_size_big_, fx, fy, px, py);
			bctr++;
		}
		else
		{
			T = detections_[i].getRelativeTransform(tag_size_small_, fx, fy, px, py);
			sctr++;
		}

		// the orientation of the tag
		Eigen::Matrix3d rot = T.block(0,0,3,3);
		Eigen::Quaternion<double> final = Eigen::Quaternion<double>(rot);	//convert it to quaternion

		// the x,y,z location of the tag
		tag_pose.position.x=T(0,3);tag_pose.position.y=T(1,3);tag_pose.position.z=T(2,3);
		//set up the transform rotation
		tag_pose.orientation.x=final.x();tag_pose.orientation.y=final.y();tag_pose.orientation.z=final.z();tag_pose.orientation.w=final.w();

		//transmit the tf
		geometry_msgs::PoseStamped tag_robot,tag_camera;
		tag_camera.header=imgheader;tag_camera.pose=tag_pose;

		try
		{
			tf_lr_.transformPose("/base_footprint",tag_camera,tag_robot);
		}
		catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
		}
		pose_pub_.publish(tag_robot);

		tf::StampedTransform tag2world;
		try{
			tf_lr_.lookupTransform(string("/")+tag,"/world",imgheader.stamp,tag2world);
		}
		catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
		}

		//find base_footprint to tag_base and declare it as the world in the beacon tf tree
		//calculate the transform between the robot_base and the world
		tf::poseStampedMsgToTF(tag_robot,world2base);

		world2base*=(tag2world);

		//publish nav::odom message for ekf
		pubOdom(world2base,imgheader.stamp);

	}
	res.header.frame_id="/base_footprint";
	res.header.stamp=imgheader.stamp;
	res.pose.position.x=world2base.getOrigin().getX();res.pose.position.y=world2base.getOrigin().getY();res.pose.position.z=world2base.getOrigin().getZ();
	res.pose.orientation.x=world2base.getRotation().getX();
	res.pose.orientation.y=world2base.getRotation().getY();
	res.pose.orientation.z=world2base.getRotation().getZ();
	res.pose.orientation.w=world2base.getRotation().getW();
	pose_pub_.publish(res);				//publish the average solution
}

tf::Stamped<tf::Transform> BeaconDetector::initProcess(double fx,double fy, double px, double py, std_msgs::Header imgheader)
{
	ROS_INFO("In initialization");

	string tag;										//the tag name if it is valid
	bool tag_type;									//if small use the small size for cal else use big

	int sctr=0;										//the neumber of valid tags detected
	int bctr=0;										//the number of big tags detected

	geometry_msgs::Pose tag_pose;					//the pose of the tag
	tf::Stamped<tf::Transform> 	world2base;

	for (int i=0; i<detections_.size(); i++)
	{
		tag=checkInConst(detections_[i],tag_type);			//check if it is the constellation if so whats is its name in home tf tree?
		if(tag.empty())
		{
			ROS_INFO("Detected tag is not the part of the beacon constellation");
			continue;
		}

		ROS_INFO( "  Id: %d --- Hamming distance: %f \n",detections_[i].id,  detections_[i].hammingDistance );

		// recovering the relative pose requires camera calibration;
		Eigen::Matrix4d T;
		if(tag_type)
		{
			T = detections_[i].getRelativeTransform(tag_size_big_, fx, fy, px, py);
			bctr++;
		}
		else
		{
			T = detections_[i].getRelativeTransform(tag_size_small_, fx, fy, px, py);
			sctr++;
		}

		// the orientation of the tag
		Eigen::Matrix3d rot = T.block(0,0,3,3);
		Eigen::Quaternion<double> final = Eigen::Quaternion<double>(rot);	//convert it to quaternion

		// the x,y,z location of the tag
		tag_pose.position.x=T(0,3);tag_pose.position.y=T(1,3);tag_pose.position.z=T(2,3);
		//set up the transform rotation
		tag_pose.orientation.x=final.x();tag_pose.orientation.y=final.y();tag_pose.orientation.z=final.z();tag_pose.orientation.w=final.w();

		//transmit the tf
		geometry_msgs::PoseStamped tag_robot,tag_camera;
		tag_camera.header=imgheader;tag_camera.pose=tag_pose;
		try
		{
			tf_lr_.transformPose("/base_footprint",tag_camera,tag_robot);
		}
		catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
		}
		pose_pub_.publish(tag_robot);

		tf::StampedTransform tag2world;
		try{
			tf_lr_.lookupTransform(string("/")+tag,"/tag_base",imgheader.stamp,tag2world);
		}
		catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
		}

		//find base_footprint to tag_base and declare it as the world in the beacon tf tree
		//calculate the transform between the robot_base and the world

		tf::poseStampedMsgToTF(tag_robot,world2base);

		world2base*=(tag2world);

		ROS_INFO("x: %f y: %f z: %f",world2base.getOrigin().getX(),world2base.getOrigin().getY(),world2base.getOrigin().getZ());

		if(test_)
			br_.sendTransform(tf::StampedTransform(world2base, ros::Time::now()+ros::Duration(0.5), "/world","/tag_base"));

	}
	return(world2base);
}


void BeaconDetector::pubOdom(tf::Stamped<tf::Transform> pose, ros::Time time)
{
	nav_msgs::Odometry msg;
	msg.header.stamp = time;           // time of current measurement
    msg.header.frame_id = "base_footprint";        // the tracked robot frame
	msg.pose.pose.position.x = pose.getOrigin().getX();    // x measurement tag.
	msg.pose.pose.position.y = pose.getOrigin().getY();    // y measurement tag.
	msg.pose.pose.position.z = pose.getOrigin().getZ();    // z measurement tag.
    msg.pose.pose.orientation.x = pose.getRotation().getX();
	msg.pose.pose.orientation.y = pose.getRotation().getY();
	msg.pose.pose.orientation.z = pose.getRotation().getZ();
	msg.pose.pose.orientation.w = pose.getRotation().getW();

	ROS_INFO("x: %f y: %f z: %f",pose.getOrigin().getX(),pose.getOrigin().getY(),pose.getOrigin().getZ());

	msg.pose.covariance[0]=0.007;
	msg.pose.covariance[7]=0.007;
	msg.pose.covariance[14]=0.007;
	msg.pose.covariance[21]=0.007;
	msg.pose.covariance[28]=0.007;
	msg.pose.covariance[35]=0.007;
	odom_pub_.publish(msg);
}

void BeaconDetector::showResult(cv::Mat img)
{
	Mat image=colorImg_.clone();

	for (int i=0; i<detections_.size(); i++)
	{
		bool dummy;
		string tag=checkInConst(detections_[i],dummy);
		if(tag.empty())
			continue;
		// use corner points detected by line intersection
		std::pair<float, float> p1 = detections_[i].p[0];
		std::pair<float, float> p2 = detections_[i].p[1];
		std::pair<float, float> p3 = detections_[i].p[2];
		std::pair<float, float> p4 = detections_[i].p[3];

		// plot outline
		cv::line(image, cv::Point2f(p1.first, p1.second), cv::Point2f(p2.first, p2.second), cv::Scalar(255,0,0,0) );
		cv::line(image, cv::Point2f(p2.first, p2.second), cv::Point2f(p3.first, p3.second), cv::Scalar(0,255,0,0) );
		cv::line(image, cv::Point2f(p3.first, p3.second), cv::Point2f(p4.first, p4.second), cv::Scalar(0,0,255,0) );
		cv::line(image, cv::Point2f(p4.first, p4.second), cv::Point2f(p1.first, p1.second), cv::Scalar(255,0,255,0) );

		//just a check for determining max size of tag

		cv::circle(image, cv::Point2f(detections_[i].cxy.first, detections_[i].cxy.second), 8, cv::Scalar(0,0,255,0), 2);

		// print ID
		std::ostringstream strSt;
		strSt << "#" << detections_[i].id;
		cv::putText(image, strSt.str(),
	              cv::Point2f(detections_[i].cxy.first + 10, detections_[i].cxy.second + 10),
	              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
	}
	imshow("Result",image);
	waitKey(20);
}
BeaconDetector::~BeaconDetector()
{
}

