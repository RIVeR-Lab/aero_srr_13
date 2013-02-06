/*
 * monocam.cpp
 *
 *  Created on: Jan 25, 2013
 *      Author: bpwiselybabu
 */

#include "vision_comm/MonoCam.h"
#include <stdlib.h>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{

	Mat *frame;
	double fps;
	int no=0;
	if(argc==2)
		no=atoi(argv[1]);

	MonoCam webcam(no);

	if(!webcam.check())
	{
		cout<<"Could not start webcam Sorry"<<endl;
		return -1;
	}

	char ch='\n';
	while(ch!='q')
	{
		webcam.startTime();
		frame=webcam.getNextFrame();
		fps=webcam.endTime();
		webcam.showImage("Result");
		ch=waitKey(30);
	}

	return 0;
}
