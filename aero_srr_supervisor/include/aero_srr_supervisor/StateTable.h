/**
 * @file   StateTable.h
 *
 * @date   May 2, 2013
 * @author parallels
 * @brief  \TODO
 */

#ifndef STATETABLE_H_
#define STATETABLE_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include<boost/unordered_map.hpp>
#include<boost/foreach.hpp>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_srr_supervisor/SupervisorUtilities.h>
//***********    NAMESPACES     ****************//

namespace as = aero_srr_supervisor;
namespace aero_srr_supervisor
{

/**
 * @author Adam Panzica
 * @brief  Simple class for maintaining a state transition table
 */

class StateTable
{

private:
	typedef uint8_t state_t;
	typedef std::vector<state_t>  svec_t;
	typedef boost::unordered_map<state_t, svec_t>      tmap_t;
	typedef boost::unordered_map<state_t, std::string> smap_t;
	tmap_t transition_map_;          	///Map of all valid state transitions
	smap_t state_representations_;      ///Map of states to string representations

public:

	/**
	 * @author Adam Panzica
	 * @brief Adds a string representation of a state, useful for debugging
	 * @param [in] state The state we're creating a representation of
	 * @param [in] representation The string we want to represent the state
	 */
	void addStateStringRepresentation(const state_t state, const std::string& representation);

	/**
	 * @author Adam Panzica
	 * @brief Adds a new valid state transition to the state table
	 * @param [in] from_state The state being transitioned out of
	 * @param [in] to_state The state being transitioned into
	 * @param [in] reversible If opposite transition (to_state -> from_state) should also be created. Defaults to false
	 */
	void addTranstion(const state_t from_state, const state_t to_state, const bool reversible = false);

	/**
	 * @author Adam Panzica
	 * @brief  Checks if a transition between to states is valid
	 * @param [in] from_state The state being transitioned out of
	 * @param [in] to_state The state being transitioned into
	 * @return True if the transition is valid, else false
	 */
	bool isValidTransition(const state_t from_state, const state_t to_state) const;

	/**
	 * @author Adam Panzica
	 * @param [in]  for_state The state to get the transition list for
	 * @param [out] transition_list A vector to fill with the transition list
	 * @return True if the state existed in the StateTable, else false
	 */
	bool getTransitionList(const state_t for_state, std::vector<state_t>& transition_list) const;

	/**
	 * @author Adam Panzica
	 * @param [in] state the state to convert to string
	 * @param [out] string The string to write it to
	 *
	 * This method will fill the output string with the string representation of the state if one exists, otherwise it will be the numeric equivalent of the given state enum
	 */
	void stateToString(const state_t state, std::string& string) const;

};

}
#endif /* STATETABLE_H_ */
