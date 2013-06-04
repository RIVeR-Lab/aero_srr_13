/** Snowfence detection - Tarek El-Gaaly, Turgay Senlet @ Rutgers University **/

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <aero_srr_snowfence_detector/Snowfence.h>

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
class SnowfenceDetectionCls {
public:
	SnowfenceDetectionCls();
private:
	ros::NodeHandle nh_;
	ros::Subscriber left_image_sub_;
	ros::Publisher snowfence_locs_image_pub_;
	void imageCallback(const sensor_msgs::Image::ConstPtr& image);

};

SnowfenceDetectionCls::SnowfenceDetectionCls() {
	ROS_INFO("Starting snowfence detector");

	left_image_sub_ = nh_.subscribe<sensor_msgs::Image>(
			"/stereo_camera/left/image_rect_color", 1,
			&SnowfenceDetectionCls::imageCallback, this);
	snowfence_locs_image_pub_ = nh_.advertise<aero_srr_snowfence_detector::Snowfence>("snowfence_locs", 1);

	ROS_INFO("Snowfence detector started... \n");
}

void SnowfenceDetectionCls::imageCallback(
		const sensor_msgs::Image::ConstPtr& image) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(image, enc::TYPE_32FC1);
	} catch (cv_bridge::Exception& e) {
		ROS_WARN("cv_bridge exception: %s", e.what());
		return;
	}

	Mat img = cv_ptr->image;
	Mat imgFl = Mat::zeros(img.rows, img.cols, CV_32FC3);
	img.convertTo(imgFl, CV_32FC3);
	divide(imgFl, 255, imgFl);

	vector<Mat> channels(3);
	split(imgFl, channels);

	Mat s, temp;
	add(channels[0], channels[1], temp);
	add(temp, channels[2], s);

	vector<Mat> norm_img(3);
	Mat norm_img1, norm_img2, norm_img3;
	divide(channels[2], s, norm_img1);
	divide(channels[1], s, norm_img2);
	s.copyTo(norm_img3);

	//theshold norm_final
	float val1, val2, val3;
	Mat img_th = Mat::zeros(norm_img1.rows, norm_img1.cols, norm_img1.type());
	cout << "r:" << norm_img1.rows << endl;
	cout << "c:" << norm_img1.cols << endl;
	for (int i = 0; i < norm_img1.rows; i++) {
		for (int j = 0; j < norm_img1.cols; j++) {
			val1 = norm_img1.at<float>(i, j);
			val2 = norm_img2.at<float>(i, j);
			val3 = norm_img3.at<float>(i, j);
			if (val1 > 0.42 && val1 < 0.6 && val2 < 0.3 && val3 > 0.9)
				img_th.at<float>(i, j) = 1.0;
		}
	}

	//morphological operations
	int morph_elem = 0;
	int morph_size = 5;
	int operation = 3; //2-opening, 3-closing
	Mat element = getStructuringElement(morph_elem,
			Size(2 * morph_size + 1, 2 * morph_size + 1),
			Point(morph_size, morph_size));
	/// Apply the specified morphology operation
	morphologyEx(img_th, img_th, operation, element);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat final;
	double maxVal, minVal;
	minMaxLoc(img_th, &minVal, &maxVal);
	img_th.convertTo(final, CV_8UC1, 255.0 / (maxVal - minVal), 0); // -minVal * 255.0/(maxVal - minVal));
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

	// Draw polygonal contour + bonding rects
	Mat drawing2 = Mat::zeros(img.size(), CV_8UC3);
	img.copyTo(drawing2);

	for (int i = 0; i < contours.size(); i++) {
		Scalar color = Scalar(255, 0, 0); // rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		Scalar color2 = Scalar(0, 255, 255);
		if (boundRect[i].width > szTh || boundRect[i].height > szTh) {
			drawContours(drawing2, contours_poly, i, color2, 4, 8,
					vector<Vec4i>(), 0, Point());
			rectangle(drawing2, boundRect[i].tl(), boundRect[i].br(), color, 4,
					8, 0);
		}
	}

	///// Show in a window
//	namedWindow("Contours2", CV_WINDOW_NORMAL);
//	imshow("Contours2", drawing2);
//	waitKey();

	aero_srr_snowfence_detector::Snowfence sfLocMsg;
	for(int i=0; i < center.size(); i++){
		float x =boundRect[i].x;
		float y =boundRect[i].y;
		float width = boundRect[i].width;
		float height = boundRect[i].height;
		sfLocMsg.X[i] = x;
		sfLocMsg.Y[i] = y;
		sfLocMsg.width[i] = width;
		sfLocMsg.height[i] = height;
	}
	snowfence_locs_image_pub_.publish(sfLocMsg);

	ROS_INFO("%d snowfences found", center.size());
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "SnowfenceDetector");
	SnowfenceDetectionCls snowfence_detector;
	ros::spin();
}

