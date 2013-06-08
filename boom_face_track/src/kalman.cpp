/*
 * kalman.cpp
 *
 *  Created on: Aug 19, 2012
 *      Author: benzun
 */

#include <boom_face_track/kalman.h>
#define X_NO 6
#define Z_NO 4
using namespace cv;

kalman::kalman()
{
	float t=1;
	ctr_=0;
	double A[X_NO][X_NO]={{1, 0, 0, 0, 1, 0},
						 {0, 1, 0, 0, 0, 1},
						 {0, 0, 1, 0, 1, 0},
						 {0, 0, 0, 1, 0, 1},
	                     {0, 0, 0, 0, 1, 0},
						 {0, 0, 0, 0, 0, 1}};

	double H[Z_NO][X_NO]={{1, 0, 0, 0, 0, 0},
			             {0, 1, 0, 0, 0, 0},
			             {0, 0, 1, 0, 0, 0},
			             {0, 0, 0, 1, 0, 0}};

	double Q[X_NO][X_NO]={{1/4, 0,   0  , 0, 1/2, 0},
			             {0  , 1/4, 0  , 0, 0  , 1/2},
			             {0  , 0  , 1/4, 0, 1/2, 0},
			             {0  , 0  , 0  , 1/4,0 , 1/2},
						 {1/2, 0  , 1/2, 0  ,1 , 0},
						 {0  , 1/2, 0  , 1/2,0 , 1}};


	A_= Mat(X_NO,X_NO,CV_64F);
	for(size_t i =0;i<X_NO;i++)
	{
		for(size_t j=0;j<X_NO;j++)
		{
			A_.at<double>(i,j)=A[i][j];
		}
	}

	B_=Mat::zeros(X_NO,X_NO,CV_64F);
	H_=Mat(Z_NO,X_NO,CV_64F);
	for(size_t i =0;i<Z_NO;i++)
	{
		for(size_t j=0;j<Z_NO;j++)
		{
			H_.at<double>(i,j)=H[i][j];
		}
	}

	Q_=Mat(X_NO,X_NO,CV_64F);

	for(size_t i =0;i<X_NO;i++)
	{
		for(size_t j=0;j<X_NO;j++)
		{
			Q_.at<double>(i,j)=Q[i][j]*0.1;
		}
	}

	R_=Mat::eye(Z_NO,Z_NO,CV_64F)*1;

	P_=Mat::eye(X_NO,X_NO,CV_64F)*0.0007;

	X_=Mat::zeros(X_NO,1,CV_64F);

	Z_=Mat(Z_NO,1,CV_64F);

	prX_=Mat::zeros(X_NO,1,CV_64F);
	prP_=Mat(X_NO,X_NO,CV_64F);

	prect_=Rect(0,0,0,0);
	track_=false;
	new_=false;
	failcount_=0;



}
#define PATH "/home/benzun/video/"

void kalman::drawResult(Mat *frame)
{
	Mat temp=frame->clone();
	rectangle(temp,rect_,Scalar(0,0,255),2);
	rectangle(temp,prect_,Scalar(0,255,0),1);
	char name[200];
	sprintf(name,"%s/%05d.jpg",PATH,ctr_);
    imwrite(name,temp);
	ctr_++; 
}
kalman::~kalman()
{

}
void showRect(Mat *img, vector<Rect > rectang)
{
	Mat temp=img->clone();
	for(size_t i=0;i<rectang.size();i++)
	{
		rectangle(temp,rectang[i],Scalar(0,0,255));
	}
	imshow("results",temp);
}
void kalman::calcPriori()
{

	X_=(A_*X_);//+(B_*Mat::zeros(X_NO,1,CV_32F));
	//std::cout<<X_<<" out"<<std::endl;
	P_=(A_*P_*A_.t())+Q_;
}
void kalman::calcGain()
{
	Mat temp=((H_*P_*H_.t())+R_);
	K_=P_*H_.t()*temp.inv(DECOMP_CHOLESKY);
}
void kalman::calcCorrection()
{
	X_=X_+K_*(Z_-H_*X_);
	P_=(Mat::eye(X_NO,X_NO,CV_64F)-K_*H_)*P_;
}
cv::Rect kalman::track()
{

	if(track_)
	{

		calcPriori();
		std::cout<<X_<<std::endl;

		if(new_)
		{
			calcGain();
			calcCorrection();
			prect_=rect_;
		}
		rect_.x=X_.at<double>(0,0);
		rect_.y=X_.at<double>(1,0);
		rect_.width=(X_.at<double>(2,0)-X_.at<double>(0,0));
		rect_.height=(X_.at<double>(3,0)-X_.at<double>(1,0));
		if(sqrt(P_.at<double>(0,0))<20)
			return(rect_);
		else
			return(cv::Rect(-1,-1,-1,-1));
	}
	return(cv::Rect(-1,-1,-1,-1));
}
/**
 * @brief This function detects faces using
 * @param img
 * @return
 */
void kalman::getMeasurment(vector <Rect> results)
{
	if(results.size()==1)
	{
		rect_=results[0];
		if(!track_)
		{
			std::cout<<"Entered!!"<<std::endl;
			X_.at<double>(0,0)=(float)rect_.x;
			X_.at<double>(1,0)=(float)rect_.y;
			X_.at<double>(2,0)=(float)rect_.x+rect_.width;
			X_.at<double>(3,0)=(float)rect_.y+rect_.height;
			X_.at<double>(4,0)=0.0;
			X_.at<double>(5,0)=0.0;
			track_=true;
		}
		Z_.at<double>(0,0)=rect_.x;
		Z_.at<double>(1,0)=rect_.y;
		Z_.at<double>(2,0)=(float)rect_.x+rect_.width;
		Z_.at<double>(3,0)=(float)rect_.y+rect_.height;
		new_=true;
	}
	else
	{
		new_=false;
	}
	/*else
	{
		failcount_++;
		if(failcount_>5)
		{
			track_=false;
			failcount_=0;
		}
	}*/
}

