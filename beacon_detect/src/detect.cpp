/*
 * detect.cpp
 *
 *  Created on: Feb 14, 2013
 *      Author: bpwiselybabu
 */

/* Headers - ROS*/
#include <beacon_detect/RosBridge.h>
#include <tf/transform_broadcaster.h>

/* Headers April tag */
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>
#include <Eigen/Geometry>

/*Headers Aero */
#include <aero_srr_msgs/AeroState.h>

/*Headers standard*/
#include <stdlib.h>

using namespace cv;
using namespace std;

const char* window_name = "apriltags_demo";
bool 		g_active=true;					//global variable to activate the Becon detector

// draw April tag detection on actual image
void draw_detection(cv::Mat& image, const AprilTags::TagDetection& detection) {
  // use corner points detected by line intersection
  std::pair<float, float> p1 = detection.p[0];
  std::pair<float, float> p2 = detection.p[1];
  std::pair<float, float> p3 = detection.p[2];
  std::pair<float, float> p4 = detection.p[3];

  // plot outline
  cv::line(image, cv::Point2f(p1.first, p1.second), cv::Point2f(p2.first, p2.second), cv::Scalar(255,0,0,0) );
  cv::line(image, cv::Point2f(p2.first, p2.second), cv::Point2f(p3.first, p3.second), cv::Scalar(0,255,0,0) );
  cv::line(image, cv::Point2f(p3.first, p3.second), cv::Point2f(p4.first, p4.second), cv::Scalar(0,0,255,0) );
  cv::line(image, cv::Point2f(p4.first, p4.second), cv::Point2f(p1.first, p1.second), cv::Scalar(255,0,255,0) );

  // mark center
  cv::circle(image, cv::Point2f(detection.cxy.first, detection.cxy.second), 8, cv::Scalar(0,0,255,0), 2);

  // print ID
  std::ostringstream strSt;
  strSt << "#" << detection.id;
  cv::putText(image, strSt.str(),
              cv::Point2f(detection.cxy.first + 10, detection.cxy.second + 10),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
}

void callback(const aero_srr_msgs::AeroStateConstPtr& status)
{
	//if the state of the robot is to look for home
	if(status->state==aero_srr_msgs::AeroState::HOME)
	{
		g_active=true;						//activate the beacon detector
	}
	else
	{
		g_active=false;						//deactivate the beacon detector
	}
}

int main(int argc, char **argv)
{
	/* initialize ROS */
	ros::init(argc, argv, "roscamera");					//ros init
	ros::NodeHandle nh;									//handle to the global param list
	ros::NodeHandle pnh("~");							//handle to the local param list

	/* Variables for the image capture */
	Mat *frame;											//output of the current frame
	double fps;											//the fps of the camera as reported by the IOImages
	Mat gray;											//the April tag algo works on grey images only

	/* Variables for ROS 	*/
	string base_frame;									//the frame of the camera which is the base for the april tags
	string tag_frame;									//the prefix name of the the frame on which the april tag will be published
	string topic;										//the topic of the input camera images, give it to the ROS bridge to get opencv images
	string robot_topic;									//the topic which publishes the robot states
	char frameid[40];									//the full name of the frame on which the tag tf will be published

	static tf::TransformBroadcaster br;					//the TF broadcaseter for ROS

	/* Settings for April tag */

	double tag_size = 0.166; 							// real side length in meters of square black frame
	double fx = 620.036171; 							// default camera focal, note it is BEN's Laptop camera
	double fy = 620.688638;								// default camera focal, note it is BEN's Laptop camera
	double px = 317.387991; 							// camera principal point,
	double py = 242.043315;								// camera principal point in y

	/*Set up the ros params */
	if(!pnh.getParam("cam_topic", topic))				//initialize var from launch file
	{
		ROS_ERROR("cam_topic not set in launch file\n");
		return -1;
	}

	if(!pnh.getParam("tag_size", tag_size))
	{
		ROS_ERROR("tag_size not set, will use 0.166");
		tag_size = 0.166;								//this is the default value when you print on A4
	}
	if(!pnh.getParam("tag_frame", tag_frame))			//the prefix for the tag frame
	{
		ROS_WARN("tag_size not set, will use Tag_ as the default prefix");
		tag_frame = "Tag_";
	}
	if(!pnh.getParam("robot_topic", robot_topic))			//the prefix for the tag frame
	{
		ROS_WARN("The topic which publishes the robot state was not set");
	}
	else
	{
		ros::Subscriber sub = nh.subscribe(robot_topic.c_str(), 1, callback);	//subscribe to the robot state so you know when to activate
	}

	/* Start the camera video from the camera topic */

	RosBridge roscamera(topic.c_str(),frame);				//initialize it with the topic and the name of the frame
	Mat K;
	int fail=0;
	ros::Rate timeout(5);
	while(!K.empty())
	{
		ros::spinOnce();
		K=roscamera.intrinsic();							//The camera instrincis parameters

		if(fail++>10)
		{
			break;
		}
		timeout.sleep();

	}
	if(!K.empty())
	{
			fx=double(K.at<float>(0,0));
			fy=double(K.at<float>(1,1));
			px=double(K.at<float>(0,2));
			py=double(K.at<float>(1,2));
	}
	else
	{
		ROS_WARN("The camera configs are not being published, Using defaults!");
	}

	base_frame=roscamera.frame_id();						//the name of the camera frame on which the tf for the tags is added

	/*AprilTag detector implemented by umich*/
	AprilTags::TagDetector tag_detector(AprilTags::tagCodes36h11);
	tf::Transform transform;								//the transform between the tag and the camera

	/* Debug information through ROS in case needed */
	ROS_DEBUG("Camera topic: %s",topic.c_str());
	ROS_DEBUG("Camera frame name: %s",base_frame.c_str());			//debug information
	ROS_DEBUG("Camera [fx=%f fy=%f px=%f py=%f]",fx,fy,px,py);

	ROS_DEBUG("Tag Size: %f", tag_size);
	ROS_DEBUG("Tag prefix: %s", tag_frame.c_str());

	/* Start Processing */
	//cout<<"WORKS HERE!"<<endl;
	//frame=roscamera.getNextFrame();							//get new data

	ros::Rate loop(5);										//rate of processing, can be changed, mayebe a param??

	while(ros::ok())										//loop till ROS dies
	{
		frame=roscamera.getNextFrame();



		//check if you are in HOME state
//		if(!g_active)
//		{
//			ros::spinOnce();
//		}
//		roscamera.startTime();								//to track processing time
//

		if(frame!=NULL)
		{
			cout<<"recieved a frame"<<endl;
			Mat ycrcb;

			cvtColor(*frame,ycrcb,CV_BGR2YCrCb);
	        vector<Mat> channels;
	        split(ycrcb,channels);
	        equalizeHist(channels[0], channels[0]);
	        Mat result;
	        merge(channels,ycrcb);
	        cvtColor(ycrcb,result,CV_YCrCb2BGR);
			cv::cvtColor(result, gray, CV_BGR2GRAY);		//convert the color Image to gray image
			std::vector<AprilTags::TagDetection> detections = tag_detector.extractTags(gray);	//get tags if detected

			ROS_DEBUG("%d tag detected: \n",detections.size());		//DEBUG information

			for (int i=0; i<detections.size(); i++)
			{
			      ROS_DEBUG( "  Id: %d --- Hamming distance: %f \n",detections[i].id,  detections[i].hammingDistance );

			      // also highlight in the image
			      draw_detection(*frame, detections[i]);

			      // recovering the relative pose requires camera calibration;
			      Eigen::Matrix4d T = detections[i].getRelativeTransform(tag_size, fx, fy, px, py);

			      // the orientation of the tag
			      Eigen::Matrix3d rot = T.block(0,0,3,3);
			      Eigen::Quaternion<double> final = Eigen::Quaternion<double>(rot);	//convert it to quaternion

			      // the x,y,z location of the tag
			      transform.setOrigin( tf::Vector3(T(0,3),T(1,3), T(3,3)) );
			      //set up the transform rotation
			      transform.setRotation( tf::Quaternion(final.x(), final.y(), final.z(),final.w()) );

			      sprintf(frameid,"%s%d",tag_frame.c_str(),detections[i].id );		//debug information
			      //transmit the tf
			      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frame.c_str(), frameid));

			}
			roscamera.showImage(window_name);									//dispaly the image
			fps=roscamera.endTime();											//end the time on the clock for processor
		}
		ros::spinOnce();									//process the callbacks
		waitKey(20);
//		loop.sleep();
	}
	return 0;
}
