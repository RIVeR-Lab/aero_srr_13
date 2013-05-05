/**
 * @file   StateTable.cpp
 *
 * @date   May 2, 2013
 * @author parallels
 * @brief  \TODO
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include<boost/lexical_cast.hpp>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_srr_supervisor/StateTable.h>
//***********    NAMESPACES     ****************//


using namespace aero_srr_supervisor;

void StateTable::stateToString(const state_t state, std::string& string) const
{
	if(this->state_representations_.count(state) == 1)
	{
		string = this->state_representations_.at(state);
	}
	else
	{
		string = boost::lexical_cast<std::string>(state);
	}
}

void StateTable::addStateStringRepresentation(const state_t state, const std::string& representaiton)
{
	this->state_representations_[state] = representaiton;
}

void StateTable::addTranstion(const state_t from_state, const state_t to_state, bool reversilbe)
{
	this->transition_map_[from_state].push_back(to_state);
	if(reversilbe)
	{
		this->addTranstion(to_state, from_state, false);
	}
}

bool StateTable::isValidTransition(const state_t from_state, const state_t to_state) const
{
	bool valid = false;
	if(this->transition_map_.count(from_state) == 1)
	{
		BOOST_FOREACH(const svec_t::value_type state, this->transition_map_.at(from_state))
		{
			if(state == to_state)
			{
				valid = true;
				break;
			}
		}
	}

	return valid;
}

bool StateTable::getTransitionList(const state_t for_state, svec_t& transition_list) const
{
	if(this->transition_map_.count(for_state) == 1)
	{
		transition_list = this->transition_map_.at(for_state);
		return true;
	}
	return false;
}
