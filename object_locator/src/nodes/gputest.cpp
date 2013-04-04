/*
 * gputest.cpp
 *
 *  Created on: Mar 27, 2013
 *      Author: srr
 */

#include <object_locator/ros2cv.h>
#include "opencv2/gpu/gpu.hpp"

using namespace object_locator;
using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
int  num;

num = gpu::getCudaEnabledDeviceCount();
cout << num << endl;
}

