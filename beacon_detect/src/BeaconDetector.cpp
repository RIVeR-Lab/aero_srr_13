/*
 * BeaconDetector.cpp
 *
 *  Created on: May 6, 2013
 *      Author: bpwiselybabu
 */

#include <beacon_detect/BeaconDetector.h>

/* Headers April tag */
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>
#include <Eigen/Geometry>


using namespace cv;

BeaconDetector::BeaconDetector():it_(nh_)
{
	getRosParam();										//initialize all the Ros Parameters

	/* Start the camera video from the camera topic */
	if(!cam_topic_.empty())
	{
		subImg_ = it_.subscribeCamera(cam_topic_.c_str(), 1, &BeaconDetector::imageCb, this);
		ROS_INFO("Subscribing to: %s\n",cam_topic_.c_str());
	}
	else
		return;
	//thread the beacon detector function
	boost::thread detector_thread_( boost::bind( &BeaconDetector::detectBeacons, this ) );
}
void BeaconDetector::imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info)
{
	cv_bridge::CvImagePtr cv_ptr;
	boost::mutex::scoped_lock lock(imglock_);								//ensures that the variables being set here are independent of the race conditions
	try
	{
	   cv_ptr = cv_bridge::toCvCopy(msg);
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
	}
	colorImg_=cv_ptr->image.clone();

	intrinsic_=(cv::Mat_<float>(3,3)<<  cam_info->K[0],cam_info->K[1],cam_info->K[2],\
	                                    cam_info->K[3],cam_info->K[4],cam_info->K[5],\
					                    cam_info->K[6],cam_info->K[7],cam_info->K[8]);
	parent_frame_=cv_ptr->header.frame_id;


	newimg_=true;
}

void BeaconDetector::getRosParam()
{
	ros::NodeHandle pnh("~");							//handle to the local param list
	if(!pnh.getParam("cam_topic", cam_topic_))				//initialize var from launch file
	{
		ROS_ERROR("cam_topic not set in launch file\n");
	}
	if(!pnh.getParam("tag_size", tag_size_))
	{
		ROS_ERROR("tag_size not set, will use 0.166");
		tag_size_ = 0.166;								//this is the default value when you print on A4
	}
	if(!pnh.getParam("tag_frame_prefix", tag_frame_prefix_))			//the prefix for the tag frame
	{
		ROS_WARN("tag_size not set, will use Tag_ as the default prefix");
		tag_frame_prefix_ = "Tag_";
	}
	if(!pnh.getParam("robot_topic", robot_topic_))			//the prefix for the tag frame
	{
		ROS_WARN("The topic which publishes the robot state was not set");
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

void BeaconDetector::detectBeacons()
{
	Mat frame,gray;
	double fx,fy,px,py;
	bool process;
	tf::Transform transform;									//the transform between the tag and the camera
	AprilTags::TagDetector tag_detector(AprilTags::tagCodes36h11);// the tag detector not this is set to a specific family TODO: expose this as a dynamic reconfig

	while(ros::ok())
	{
	//load the variables locally
		{
			boost::mutex::scoped_lock lock(imglock_);				//make sure no one else is accessing these variables
			frame=colorImg_.clone();								//copy the image. = operater only copys the pointer so it will lead to race conditons

			process=newimg_;										//if its a new image you need to process it

			if(frame.empty())										//empty frame so cant process
			process=false;

			if(!intrinsic_.empty())									//copy the intrinsic params
			{
				fx=double(intrinsic_.at<float>(0,0));
				fy=double(intrinsic_.at<float>(1,1));
				px=double(intrinsic_.at<float>(0,2));
				py=double(intrinsic_.at<float>(1,2));
			}
			else
			{
				ROS_ERROR("Camera Info not being published");
				process=false;									//if you dont have camera config you cant publish transform so dont process
			}

		}

		if(process)
		{
			histEq(frame);										//TODO: with dynamic reconfigure expose this as a parameter pls

			cv::cvtColor(frame, gray, CV_BGR2GRAY);						//convert the color Image to gray image
			std::vector<AprilTags::TagDetection> detections = tag_detector.extractTags(gray);	//get tags if detected

			ROS_INFO("%d tag detected: \n",detections.size());									//DEBUG information

			for (int i=0; i<detections.size(); i++)
			{
				ROS_DEBUG( "  Id: %d --- Hamming distance: %f \n",detections[i].id,  detections[i].hammingDistance );

				// also highlight in the image
				//draw_detection(*frame, detections[i]);

				// recovering the relative pose requires camera calibration;
				Eigen::Matrix4d T = detections[i].getRelativeTransform(tag_size_, fx, fy, px, py);

				// the orientation of the tag
				Eigen::Matrix3d rot = T.block(0,0,3,3);
				Eigen::Quaternion<double> final = Eigen::Quaternion<double>(rot);	//convert it to quaternion

				// the x,y,z location of the tag
				transform.setOrigin( tf::Vector3(T(0,3),T(1,3), T(3,3)) );
				//set up the transform rotation
				transform.setRotation( tf::Quaternion(final.x(), final.y(), final.z(),final.w()) );
				char frameid[50];

				sprintf(frameid,"%s%d",tag_frame_prefix_.c_str(),detections[i].id );		//debug information
				//transmit the tf
				br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_.c_str(), frameid));

			}
			imshow("see",frame);
			process=false;
		}
	}
	if(detector_thread_.joinable())
	{
		detector_thread_.join();
	}
}
BeaconDetector::~BeaconDetector()
{
	if(detector_thread_.joinable())
	{
		detector_thread_.join();
	}
}

