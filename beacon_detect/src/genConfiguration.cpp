/*
 * genConfiguration.cpp
 *
 *  Created on: May 23, 2013
 *      Author: bpwiselybabu
 */
/*
 * gensettings.cpp
 *
 *  Created on: Mar 29, 2013
 *      Author: bpwiselybabu
 */
#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{

	FileStorage fs("configuration.xml",FileStorage::WRITE);
	std::string name="tag";
	std::string id="tag_id";
	cout<<"Enter no of tags: ";
	int no_tags;
	int tag_id;
	char tag_name[20];
	cin>>no_tags;
	fs<<string("tag_no")<<no_tags;
	for(int i=0;i<no_tags;i++)
	{
		cout<<"Enter tag Name: ";
		cin>>tag_name;
		cout<<"Enter tag id: ";
		cin>>tag_id;
		char tname[20];
		sprintf(tname,"%s%d",name.c_str(),i);
		fs<<string(tname)<<string(tag_name);
		sprintf(tname,"%s%d",id.c_str(),i);
		fs<<string(tname)<<tag_id;
	}
	fs.release();
	return(0);
}






