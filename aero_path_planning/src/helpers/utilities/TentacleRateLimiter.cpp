/**
 * @file   TentacleRateLimiter.cpp
 *
 * @date   Apr 26, 2013
 * @author Adam Panzica
 * @brief  Implementation of TentacleRateLimiter class
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <cstdlib>
#include <iostream>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/utilities/TentacleRateLimiter.h>
//***********    NAMESPACES     ****************//

using namespace aero_path_planning;

TentacleRateLimiter::TentacleRateLimiter(int number_of_tentacles, int rate_limit):
					num_tent_(number_of_tentacles),
					last_tentacle_(-1),
					rate_limit_(rate_limit),
					enabled_(true)
{
	this->origin_ = number_of_tentacles/2+1;
}

void TentacleRateLimiter::setRateLimit(int rate_limit)
{
	this->rate_limit_ = rate_limit;
}

int TentacleRateLimiter::getRateLimit() const
{
	return this->rate_limit_;
}

int TentacleRateLimiter::nextTentacle(int raw_index)
{
	int next_tentacle = raw_index;
	if(this->enabled_)
	{
		if(this->last_tentacle_!=-1)
		{
			int difference = 0;
			//If the last tentacle and the raw tentacle are on the same side, we can just add the difference, rate-limited. If rate-limited we are essentially on the same side do the same
			if(this->sameSide(raw_index))
			{
				difference = raw_index - this->last_tentacle_;
				if(std::abs(difference) > this->rate_limit_)
				{
					//Need to make sure we retain the sign of difference so it works on both sides of the origin
					difference = this->rate_limit_*(difference/std::abs(difference));
				}
				next_tentacle = this->last_tentacle_ + difference;
			}
			//otherwise we have to calculate what the correct tentacle should be
			else
			{
				int post_switch_offset = 0;
				int pre_switch_offset  = 0;
				//Are we crossing left->right or right->left?
				//Left->right
				if(raw_index>this->origin_)
				{

					post_switch_offset = this->num_tent_ - raw_index;
					pre_switch_offset  = this->origin_ - this->last_tentacle_;
					difference         = post_switch_offset + pre_switch_offset;

					//If pre_switch_offset is over rate_limit, we don't need to calculate the switch
					if(pre_switch_offset>this->rate_limit_)
					{
						next_tentacle = this->last_tentacle_+this->rate_limit_;
					}
					//If the switch exceeds the rate limit, clamp it and do more calculations
					else if(difference > this->rate_limit_)
					{
						//The remaining difference will be rate_limit minus the pre_switch offset
						difference = rate_limit_ - pre_switch_offset;
						//We then subtract that from the number of tentacles to get our new one
						next_tentacle = this->num_tent_ - difference;

					}
					//Otherwise we can just pass it through
					else
					{
						next_tentacle = raw_index;
					}
				}
				//Right->left
				else
				{
					post_switch_offset = this->origin_ - raw_index;
					pre_switch_offset  = this->num_tent_ - this->last_tentacle_;
					difference         = post_switch_offset + pre_switch_offset;
					//If pre_switch_offset is over rate_limit, we don't need to calculate the switch
					if(pre_switch_offset>this->rate_limit_)
					{
						next_tentacle = this->last_tentacle_+this->rate_limit_;
					}
					//If the switch exceeds the rate limit, clamp it and do more calculations
					else if(difference > this->rate_limit_)
					{
						//The remaining difference will be rate_limit minus the pre_switch offset
						difference = rate_limit_ - pre_switch_offset;
						//We then subtract that from the origin of tentacles to get our new one
						next_tentacle = this->origin_ - difference;

					}
					//Otherwise we can just pass it through
					else
					{
						next_tentacle = raw_index;
					}
				}
			}

		}
	}
	//Store what the last selected tentacle was
	this->last_tentacle_ = next_tentacle;
	return next_tentacle;
}

void TentacleRateLimiter::disableLimit(bool disable)
{
	if(disable)
	{
		this->enabled_ = false;
	}
	else
	{
		this->enabled_ = true;
	}
}

bool TentacleRateLimiter::sameSide(int raw_index) const
{
	bool both_left  = (raw_index<this->origin_)&&(this->last_tentacle_< this->origin_);
	bool both_right = (raw_index>=this->origin_)&&(this->last_tentacle_>= this->origin_);
	return both_left || both_right;
}

