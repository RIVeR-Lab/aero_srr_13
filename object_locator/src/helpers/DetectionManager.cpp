/**
 * @file   DetectionManager.cpp
 *
 * @date   Mar 22, 2013
 * @author Adam Panzica
 * @brief  Implementation of the DetectionManager class
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <boost/foreach.hpp>
//************ LOCAL DEPENDANCIES ****************//
#include <object_locator/DetectionManager.h>
//***********    NAMESPACES     ****************//

using namespace object_locator;

DetectionManager::DetectionManager(double threshold_dist, double growth_rate, double shrink_rate, double threshold_det):
								threshold_dist_(threshold_dist),
								growth_rate_(growth_rate),
								shrink_rate_(shrink_rate),
								threshold_det_(threshold_det),
								max_condifdence_(1.0)
{

}

DetectionManager::size_type DetectionManager::size() const
{
	if(!this->detections_.empty())
	{
		return this->detections_.size();
	}
	else
	{
		return 0;
	}
}

bool DetectionManager::isEmpty() const
{
	return this->detections_.empty();
}

double DetectionManager::averageConfidance() const
{
	if(this->size()!=0)
	{

		double sum = 0;
		BOOST_FOREACH(DetectionArray_t::value_type item, this->detections_)
		{
			sum += item->second;
		}

		return sum/(double)this->size();
	}
	else
	{
		return 0;
	}
}

void DetectionManager::addDetection(const tf::Point& detection)
{
	bool growth = false;
	BOOST_FOREACH(DetectionArray_t::value_type item, this->detections_)
	{
		if(item->first.distance(detection) <= this->threshold_dist_ )
		{
			if(item->second < this->max_condifdence_)
			{

				item->first = (item->first+detection)/2;
				item->second += this->growth_rate_;
			}
			growth = true;
		}

		if(growth)
		{
			break;
		}
	}

	if(!growth)
	{
		this->detections_.push_back(DetectionPtr(new Detection_t(detection, this->growth_rate_)));
	}
}

void DetectionManager::shrink()
{

		for(DetectionArray_t::iterator itr = this->detections_.begin(); itr!=this->detections_.end(); itr++)
		{

			(*itr)->second-= this->shrink_rate_;
//			if((*itr)->second <= 0)
//			{
//				this->detections_.erase(itr);
//			}

		}

}

bool DetectionManager::getDetection(tf::Point& detection, double& confidence) const
{
	if(this->size() > 0)
	{
		DetectionPtr best_conf(new Detection_t(detection, 0));
		BOOST_FOREACH(DetectionArray_t::value_type item, this->detections_)
		{
			if((item->second > this->threshold_det_) && (item->second > best_conf->second))
			{
				best_conf = item;
			}
		}

		if(best_conf->second > this->threshold_det_)
		{
			detection = best_conf->first;
			confidence= best_conf->second;
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

void DetectionManager::clear()
{
	this->detections_.clear();
}
