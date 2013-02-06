/*
 * PicSet.h
 *
 *  Created on: Jan 25, 2013
 *      Author: bpwiselybabu
 */

#ifndef PICSET_H_
#define PICSET_H_
#include <vision_comm/IOImages.h>
#include <vision_comm/main.h>

class PicSet : public IOImages{
	int 		start_;
	int 		end_;
	std::string	location_;
	std::string	prefix_;
	std::string	ext_;

public:
	PicSet(int begin, int end, std::string location, std::string prefix ,std::string ext=std::string(".jpg"));
	virtual ~PicSet();
	cv::Mat* getNextFrame();
};

#endif /* PICSET_H_ */
