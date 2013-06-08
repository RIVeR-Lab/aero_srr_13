#include <boom_face_track/FaceTrack.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>


using namespace cv;

FaceTrack::FaceTrack():it_(nh_)
{	
	cam_topic_=std::string("/lower_stereo_raw/right/image_raw");
	
	timer_=nh_.createTimer(ros::Duration(90), &FaceTrack::timerCallback,this,true);

	/* Start the camera video from the camera topic */
	if(!cam_topic_.empty())
	{
		subImg_ = it_.subscribeCamera(cam_topic_.c_str(), 1, &FaceTrack::imageCb, this);
		ROS_INFO("Subscribing to: %s\n",cam_topic_.c_str());
	}
	else
		return;


	/*set up the action client for rotating the boom */
	boom_client_ = boost::shared_ptr<BoomClient>(new BoomClient(nh_, "/camera_boom_control", true));
	boom_client_->waitForServer();

}
void FaceTrack::timerCallback(const ros::TimerEvent& event)
{
	device_driver_base::SetJointPositionGoal boom_position;
	boom_position.max_velocity = 0.5;
	boom_position.angle = 0.0;
	boom_client_->sendGoal(boom_position);
}
void FaceTrack::imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info)
{

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


	Mat gray;
	cv::cvtColor(colorImg_, gray, CV_BGR2GRAY);						//convert the color Image to gray imagePr


    vector <Rect> results;
    std::string path = ros::package::getPath("beacon_detect");
    std::string name= path+"/"+"lbpcascade_frontalface.xml";
	CascadeClassifier face(name.c_str());
	if(face.empty())
	{
		ROS_ERROR("Cascade not loaded");
	}
	face.detectMultiScale(gray,results);
	filter_.getMeasurment(results);
	cv::Rect res=filter_.track();
	double angle=0.0f;
	if(res.x!=-1)
	{
		angle=atan(abs(px-res.width/2+res.x)/fx);
		if(res.width/2+res.x<px)
			angle=-angle;
	}
	/*get the current orientation of the boom*/
	tf::StampedTransform boom2boombase;
	tf_lr_.lookupTransform("/boom","/boom_base",img_header.stamp,boom2boombase);
	tf::Quaternion q=boom2boombase.getRotation();
	double roll,pitch,yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	/*send the message to turn boom*/
	device_driver_base::SetJointPositionGoal boom_position;
	boom_position.max_velocity = 0.5;
	boom_position.angle = angle+yaw;
	boom_client_->sendGoal(boom_position);

	/*debug result to see the performance*/
	filter_.drawResult(&gray);
	imshow("face",gray);
	waitKey(2);
}

void FaceTrack::histEq(cv::Mat &frame)
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

FaceTrack::~FaceTrack()
{
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"facetracker");
	FaceTrack ft;
	ros::spin();
}
