/*
 * DisparityStage.cpp
 *
 *  Created on: Apr 10, 2013
 *      Author: srr
 */

#include <object_locator/vision_pipeline/DisparityStage.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(object_locator, DisparityStage,
		object_locator::DisparityStage, nodelet::Nodelet)


namespace enc = sensor_msgs::image_encodings;
using namespace object_locator;

void DisparityStage::onInit()
{
	NODELET_INFO_STREAM("Initializing Disparity Stage");
	loadParams();
	registerTopics();
	NODELET_INFO_STREAM("Disparity Stage Initialized");
}

void DisparityStage::loadParams()
{
	this->input_topic_="/sync_stage/rect_pair";
	this->output_topic_="disparity_stage/disparity";
	this->getPrivateNodeHandle().getParam(this->input_topic_,this->input_topic_);
	this->getPrivateNodeHandle().getParam(this->output_topic_,this->output_topic_);

	std::string minDisp("min_disp");
	minDisp_ = 0;      //0         //-128-32;
	this->getPrivateNodeHandle().getParam(minDisp,minDisp_);

	std::string numDisp("num_disp");
	numDisp_ = 192;       //80        //256+80;
	this->getPrivateNodeHandle().getParam(numDisp,numDisp_);

	std::string SADSize("SADSize");
	SADSize_ = 9;				//10
	this->getPrivateNodeHandle().getParam(SADSize,SADSize_);

	std::string P1("P1");
	P1_ =  8*SADSize_*SADSize_;
	this->getPrivateNodeHandle().getParam(P1,P1_);

	std::string P2("P2");
	P2_ = 32*SADSize_*SADSize_;
	this->getPrivateNodeHandle().getParam(P2,P2_);

	std::string disp12MaxDiff("disp12");
	disp12MaxDiff_ =  -1	; // 1;
	this->getPrivateNodeHandle().getParam(disp12MaxDiff,disp12MaxDiff_);

	std::string preFilterCap("preFilterCap");
	preFilterCap_ =   31; //  2;
	this->getPrivateNodeHandle().getParam(preFilterCap,preFilterCap_);

	std::string uniqueness("uniqueness");
	uniqueness_ = 5;
	this->getPrivateNodeHandle().getParam(uniqueness,uniqueness_);

	std::string specSize("specSize");
	specSize_ =   100; //50 //20;   //reduces noise
	this->getPrivateNodeHandle().getParam(specSize,specSize_);

	std::string specRange("specRange");
	specRange_ = 20  ;  //5 //1;
	this->getPrivateNodeHandle().getParam(specRange,specRange_);
}

void DisparityStage::registerTopics()
{
	this->sync_image_sub_ = this->getNodeHandle().subscribe(this->input_topic_,2,&DisparityStage::recieveImageCb,this);
	this->disp_image_pub_ = this->getNodeHandle().advertise<object_locator::SyncImagesAndDisparity>(this->output_topic_,2);

	this->dr_server_ = DRServerPtr(new DRServer_t(this->getPrivateNodeHandle()));
	this->dr_server_->setCallback(boost::bind(&DisparityStage::drCB, this, _1, _2));
}

void DisparityStage::recieveImageCb(const object_locator::SyncImageMsgConstPtr& msg)const
{

	int img_height = msg->left_image.height;
	int img_width = msg->left_image.width;
	Mat_t left_rect(img_height, img_width,CV_8U);
	Mat_t right_rect(img_height, img_width,CV_8U);
	Mat_t disparity(img_height,img_width, CV_16UC1);
	computeRectifiedImage(msg->left_image, msg->left_info,left_rect);
	computeRectifiedImage(msg->right_image, msg->right_info,right_rect);
	computeDisparity(left_rect, right_rect,disparity);
	object_locator::SyncImagesAndDisparityPtr message(new object_locator::SyncImagesAndDisparity);
	generateDispMsg(*msg, disparity, *message);
	this->disp_image_pub_.publish(message);

}

void DisparityStage::computeRectifiedImage(const sensor_msgs::Image& msg, const sensor_msgs::CameraInfo& cam_info, Mat_t& rectified_img)const
{
	cv_bridge::CvImagePtr img_ptr;
	try
	{
		img_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	rectified_img = img_ptr->image;
//	sensor_msgs::CameraInfo info(cam_info);
//	Mat_t K(3,3,CV_64F, info.K.elems);
//	Mat_t R(3,3,CV_64F, info.R.elems);
//	Mat_t P(3,4,CV_64F, info.P.elems);
//	Mat_t D(1,5,CV_64F, info.D.data());
//	cv::Size size;
//	size.height = cam_info.height;
//	size.width = cam_info.width;
//
//	Mat_t mx(size.height, size.width, CV_32FC1);
//	Mat_t my(size.height, size.width, CV_32FC1);
//
//	cv::initUndistortRectifyMap(K, D, R, P, size, CV_32FC1, mx, my);
//	Mat_t img_gray;
//	cv::cvtColor(img_ptr->image, img_gray, CV_BGR2GRAY);
//	remap(img_gray,rectified_img, mx, my, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

}

void DisparityStage::computeDisparity(const Mat_t& rectLeft, const Mat_t& rectRight, Mat_t& disparity)const
{
	Mat_t preDisp(rectLeft.rows,rectLeft.cols, CV_16S);

	cv::StereoSGBM stereoSGBM(minDisp_, numDisp_, SADSize_, P1_, P2_,
			disp12MaxDiff_, preFilterCap_, uniqueness_, specSize_, specRange_, true);
	stereoSGBM(rectLeft, rectRight, preDisp );
	preDisp.convertTo(disparity, -1,1.0/16);

//	normalize( preDisp, disparity, 0, 256, CV_MINMAX );

}

void DisparityStage::generateDispMsg(const object_locator::SyncImageMsg& raw_imgs, Mat_t& disparity, object_locator::SyncImagesAndDisparity& msg)const
{
	cv_bridge::CvImage carrier;
	carrier.image = disparity;
	sensor_msgs::Image disparity_image;
	carrier.toImageMsg(disparity_image);
	msg.disparity_image = disparity_image;
	msg.disparity_image.encoding  = enc::MONO16;
	msg.images          = raw_imgs;

}

void DisparityStage::drCB(Config_t &config, uint32_t level)
{
	this->disp12MaxDiff_ = config.disp12_max_diff;

	if(config.p1 < config.p2)
	{
		this->P1_        = config.p1;
		this->P2_        = config.p2;
	}
	else
	{
		NODELET_WARN_STREAM("Cannot set P1 ("<<config.p1<<") greater than P2 ("<<config.p2<<")");
	}

	if(config.sad_size%2!=0)
	{
		this->SADSize_   = config.sad_size;
	}
	else
	{
		NODELET_WARN_STREAM("SADsize must be an odd number, got "<<config.sad_size);
	}

	this->fullDp_        = config.full_dp;
	this->minDisp_       = config.min_disp;

	if(config.num_disp%16==0)
	{
		this->numDisp_   = config.num_disp;
	}
	else
	{
		NODELET_WARN_STREAM("Number of Disparities must be divisible by 16, got "<<config.num_disp);
	}

	this->preFilterCap_  = config.pre_filter_cap;
	this->specRange_     = config.speckle_range;
	this->specSize_      = config.speckle_window_size;
	this->uniqueness_    = config.uniqueness_ratio;
}
