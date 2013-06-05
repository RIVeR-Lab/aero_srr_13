#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

class LaserHillRemoverNode
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;

  // Components for publishing
  sensor_msgs::LaserScan msg_;
  ros::Publisher output_pub_;
  ros::Subscriber sub_;

  int window_;
  double threshold_;
  int width_;

public:
  void loadROSParams()
  {
	  ros::NodeHandle pnh("~");							//handle to the local param list
	  if(!pnh.getParam("window_size", window_))				//initialize var from launch file
	{
		ROS_WARN("Using default window_size of 2");
		window_=2;
	}
	if(!pnh.getParam("threshold", threshold_))
	{
		ROS_WARN("Using default threshold of 0.05");
		threshold_ = 0.05;								//this is the default value when you print on A4
	}
	if(!pnh.getParam("width", width_))
	{
		ROS_WARN("Using default width of 40");
		width_ = 0.05;								//this is the default value when you print on A4
	}
  }
  // Callback
  void callback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
  {
	  vector<float> laserscan = msg_in->ranges;//ranges in meters
	  float maxRange = msg_in->range_max;
	  float infinity = maxRange + 1;

	  int scanSz = laserscan.size();
	  float* values = new float[scanSz];
	  float* result = new float[scanSz];
	  float* smoothed = new float[scanSz];

	  float maxVal=values[0], minVal=values[0];
	  for(int i=0; i<scanSz; i++)
	  {
		  smoothed[i] = infinity;
		  result[i] = infinity;
		  values[i] = laserscan.at(i);
		  if(values[i] > maxVal)
			  maxVal = values[i];
		  if(values[i] < minVal)
			  minVal = values[i];
	  }
	  ROS_INFO_STREAM("max laser range: " << maxVal<< ", min laser range: " << minVal);

	  //smooth
	  int d=window_;//todo: convert to param (the window size over scan values)
	  float th=threshold_;//todo - convert to param (diff between scans in meters)
	  int counter=0;
	  int big = width_;
	  int maxCounter = 0;

	  for(int i=d; i<scanSz-d; i++)
	  {
		  int c = 0;
		  for(int j=-d; j<=d; j++)
		  {
			  smoothed[i]+=values[i+j];
		  }
		  smoothed[i] /= (2*d+1);
	  }

	  //filter
	  //float32 range_min, range_max
	  //float32[] ranges in meters
	  for(int i=d; i<scanSz-d; i++)
	  {
		  result[i]=values[i];
		  float diff = abs(values[i] - values[i+1]);
		  //float diff = abs(values[i] - smoothed[i+1]);
		  if(diff > th)
		  {
			  if(counter>big)//todo: benzooon convert to param (sequence size of scans - if large then remove)
			  {
				  for(int j=0; j<counter ; j++)
				  {
					  result[i-j-1]=infinity;
				  }
			  }
			  counter=0;
		  }
		  else
		  {
			  counter++;
			  if (maxCounter < counter)
			  {
				  maxCounter = counter;
			  }
		  }
	  }
	  //remove last large sequence of values if big
	  if(counter>big)//todo: benzooon convert to param (sequence size of scans - if large then remove)
	  {
		  for(int j=0; j<counter ; j++)
		  {
			  result[(scanSz-d-1)-j-1]=infinity;
		  }
	  }
	  counter=0;

	//  for(int i=0; i<scanSz; i++)
	 // {
	// 	 ROS_INFO_STREAM("No: " << i << ", val: " << values[i] << ", diff: " << abs(values[i] - values[i+1]) << ", result: " << result[i]);
	 // }
//int iii=0;
//cin>>iii;
	  ROS_INFO_STREAM("Size: " << scanSz << ", max counter" << maxCounter);

	  //convert to laserscan msg
	  sensor_msgs::LaserScan resultScan;
	  resultScan = *msg_in;
	  resultScan.ranges.clear();
	  for(int i=0; i<scanSz; i++)
	  {
		  resultScan.ranges.push_back(result[i]);
	  }
	  output_pub_.publish(resultScan);
	  delete[] result;
	  delete[] smoothed;
	  delete[] values;
  }

  // Constructor
  LaserHillRemoverNode()
  {
	  threshold_=0;
	  width_=0;
	  window_=0;
	  loadROSParams();
    // Configure filter chain
    sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, &LaserHillRemoverNode::callback,this);
    // Advertise output
    output_pub_ = nh_.advertise<sensor_msgs::LaserScan>("filtered_scan", 1000);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Hill_removal_node");

  LaserHillRemoverNode t;
  ros::spin();

  return 0;
}

