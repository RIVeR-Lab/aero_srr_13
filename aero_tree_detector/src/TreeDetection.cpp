/** Tree detection - Tarek El-Gaaly, Turgay Senlet @ Rutgers University **/

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <tree_detector/Trees.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <cstdlib>
#include <math.h>
#include <vector>
#include <sstream>

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
RNG rng(12345);

//////////////////
class TreeDetectionCls
{
public:
	TreeDetectionCls();
	Mat FilterHorizontal(cv::Mat img, int l2, int l1, int w, double th);
private:
    ros::NodeHandle nh_;
    ros::Subscriber d_image_sub_;
    //ros::Subscriber rgb_image_sub_;
    ros::Publisher tree_locs_image_pub_;
    void imageCallback(const sensor_msgs::Image::ConstPtr& image);

};

TreeDetectionCls::TreeDetectionCls() {
	ROS_INFO("Starting tree detector");

	d_image_sub_ = nh_.subscribe<sensor_msgs::Image> ("camera/depth_image", 1, &TreeDetectionCls::imageCallback, this);
	//rgb_image_sub_ = nh_.subscribe<sensor_msgs::Image> ("camera/rgb_image", 1, &TreeDetectionCls::imageCallback, this);
	tree_locs_image_pub_ = nh_.advertise<tree_detector::Trees> ("tree_locs", 1);

	ROS_INFO("Tree detector started... \n");
}

Mat TreeDetectionCls::FilterHorizontal(Mat img, int l2, int l1, int w, double th)
{

	Mat fil = Mat::zeros(w,l1+l2+l1,CV_32F);//[zeros(1,l0) ones(1,l1) zeros(1,l0)];
	fil(cv::Range(0,w),cv::Range(l1,l1+l2))=1;

	Mat src;
	Mat filter;
	img.convertTo(src, CV_32F);
	fil.convertTo(filter, CV_32F);
	Mat res = Mat::zeros(Size(src.cols,src.rows),CV_32F);
	matchTemplate(src, filter, res, CV_TM_CCOEFF_NORMED);

	Mat res2 = Mat::zeros(Size(img.cols,img.rows),CV_32F);
	res.copyTo(res2(Rect(fil.cols/2,fil.rows/2,res.cols,res.rows)));//Range(0+round(fil.rows/2),src.rows-round(fil.rows/2)+1),Range(0+round(fil.cols/2),src.cols-round(fil.cols)+1)));// = res;

	for(int i=0; i<res2.rows; i++)
	{
		for(int j=0; j<res2.cols; j++)
		{
			//cout<<res.at<float>(i,j)<<endl;
			if(res2.at<float>(i,j) < 0)
				 res2.at<float>(i,j) = 0;
			else if(res2.at<float>(i,j) < th)
				res2.at<float>(i,j) = 0;
		}
	}

	return res2;
}

void TreeDetectionCls::imageCallback(const sensor_msgs::Image::ConstPtr& image)
{

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		  cv_ptr = cv_bridge::toCvCopy(image, enc::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		  ROS_WARN("cv_bridge exception: %s", e.what());
		  return;
	}

	Mat depth_img = cv_ptr->image;
	depth_img.convertTo(depth_img, CV_8U, 255.0, 0);

	double th = 0.42871;
	//3 scales of bar filtersx
	Mat n = FilterHorizontal(depth_img, 12, 5, 10, th);
	Mat m = FilterHorizontal(depth_img, 24, 7, 10, th);
	Mat o = FilterHorizontal(depth_img, 48, 10, 10, th);

	Mat nplusm;// = Mat::zeros(n.cols,n.rows,CV_32F);;
	add(n, m, nplusm);
	add(nplusm, o, nplusm);

	//set the peripheries to 0 as these are picked up as trees
	nplusm(cv::Range(0, nplusm.rows), cv::Range(0, 50)) = Scalar(0);
	nplusm(cv::Range(0, nplusm.rows), cv::Range(nplusm.cols - 49, nplusm.cols)) = Scalar(0);

	//maximal suppression to get min and max values
	double maxVal, minVal;
	minMaxLoc(nplusm, &minVal, &maxVal);

	Mat final;
	nplusm.convertTo(final, CV_8UC1, 255.0 / (maxVal - minVal), 0); // -minVal * 255.0/(maxVal - minVal));

	Mat label_image;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(final, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	vector<Point2f> center(contours.size());
	vector<float> radius(contours.size());

	int szTh = 130;
	for (int i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
		if (boundRect[i].width > szTh || boundRect[i].height > szTh)
			minEnclosingCircle((Mat) contours_poly[i], center[i], radius[i]);
	}

	tree_detector::Trees treeLocMsg;
	for(int i=0; i < center.size(); i++){
		float x =center[i].x;
		float y =center[i].y;
		float d = depth_img.at<float>(y,x);
		treeLocMsg.X[i] = x;
		treeLocMsg.Y[i] = y;
		treeLocMsg.depth[i] = d;
	}
	tree_locs_image_pub_.publish(treeLocMsg);

	ROS_INFO("%d trees found", center.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TreeDetector");
    TreeDetectionCls tree_detector;
    ros::spin();
}
