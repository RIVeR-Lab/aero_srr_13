/*
 * frameSave.cpp
 *
 *  Created on: Mar 9, 2013
 *      Author: srr
 */

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char **argv) {

  CvCapture* capture;
  Mat frame;
  int i;
  vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);
  //-- 2. Read the video stream
  capture = cvCaptureFromCAM( -1 );
  if( capture )
  {
    for(i =0;i<30;i++)
    {
      stringstream stream;
      stream << "samples/cap"<<i<<".jpg";
      cout<<stream.str()<<endl;
      frame = cvQueryFrame( capture );
      cout<<"gotFrame"<<endl;
      imwrite(stream.str(),frame, compression_params);
      cout<<"wroteFrame"<<endl;
//      int c = waitKey();
//      if( (char)c == 'c' ) { break; }
      cout <<"done"<<endl;
    }
  }
  return 0;
}
