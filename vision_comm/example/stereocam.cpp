/*
 * monocam.cpp
 *
 *  Created on: Jan 25, 2013
 *      Author: bpwiselybabu
 */

#include "vision_comm/StereoCam.h"
#include <stdlib.h>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{

	Mat framel,framer;
	double fps;

	StereoCam webcams(1,2);

	if(!webcams.check())
	{
		cout<<"Could not start webcams Sorry"<<endl;
		return -1;
	}

	char ch='\n';
	while(ch!='q')
	{

		webcams.startTime();
		webcams.getNextStereoFrame(framel,framer);
		fps=webcams.endTime();
		//cout<<&framel<<" : "<<&framer<<endl;
		//webcams.showImage("Result");
		ch=waitKey(30);
	}

	return 0;
}
