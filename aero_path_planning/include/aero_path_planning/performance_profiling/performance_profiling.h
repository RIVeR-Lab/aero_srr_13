/**
 * @file   performance_profiling.h
 *
 * @date   Jul 3, 2013
 * @author Adam Panzica
 * @brief  \TODO
 */

/*
 * Copyright (c) 2013, RIVeR Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PERFORMANCE_PROFILING_H_
#define PERFORMANCE_PROFILING_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include <ros/ros.h>
//************ LOCAL DEPENDANCIES ****************//

//***********    NAMESPACES     ****************//
namespace performance_profiling
{
	class DurationAverager
	{
	private:
		int count;
		ros::Duration time;
	public:
		DurationAverager(void): count(0),time(0)
		{

		}

		inline void reset(void)
		{
			this->count = 0;
            this->time.fromSec(0);
		}

		inline void add(ros::Duration measurement)
		{
			this->time += measurement;
			this->count++;
		}

		inline ros::Duration getAverage(void)
		{
			int sec = this->time.sec/this->count;
			int nsec= this->time.nsec/this->count;
			return ros::Duration(sec,nsec);
		}
	};
};


#ifdef PERFORMANCE_PROFILING_ENABLE
#define PERFORMANCE_PROFILING_PROFILE_CODE(code, duration)\
	ros::Time start_time(ros::Time::now);\
	code;\
	duration = ros::Time::now-start_time;
#else
#define PERFORMANCE_PROFILING_PROFILE_CODE(code, duration)\
		code;
#endif

#ifdef PERFORMANCE_PROFILING_ENABLE
#define PERFORMANCE_PROFILING_PROFILE_CODE_OUTPUT(code)\
	ros::Duration duration;\
	PERFORMANCE_PROFILING_PROFILE_CODE(code, duration);\
	ROS_INFO_STREAM("PERF_PRO: Code took "<<duration.toSec()<<" seconds to execute");
#else
#define PERFORMANCE_PROFILING_PROFILE_CODE_OUTPUT(code)\
	code;
#endif


#endif /* PERFORMANCE_PROFILING_H_ */
