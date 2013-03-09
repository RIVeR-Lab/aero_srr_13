/**
 * @file FitnessQueue.cpp
 *
 * @date Mar 8, 2013
 * @author Adam Panzica
 * @brief Implementation of FitnessQueue class
 */

//********************** SYSTEM DEPENDANCIES **********************//
//********************** LOCAL  DEPENDANCIES **********************//
#include<aero_path_planning/FitnessQueue.h>

using namespace aero_path_planning;

template<class Fitness, class Data>
bool FitnessQueue<Fitness, Data>::comparitor::operator ()(const FitnessQueue::data_pair_ptr_t& a, const FitnessQueue::data_pair_ptr_t& b) const
{
	return a->first<b->first;
}

template<class Fitness, class Data>
bool FitnessQueue<Fitness, Data>::empty() const
{
	return this->queue.empty();
}

template<class Fitness, class Data>
unsigned int FitnessQueue<Fitness, Data>::size() const
{
	return this->queue.size();
}

template<class Fitness, class Data>
void FitnessQueue<Fitness, Data>::push (const Fitness& fitness, const Data& data)
{
	ScopedLock lck(lock); // locks the mutex
	data_pair_ptr_t item(new data_pair_t(fitness, data));
	this->queue.push(item);
	// automatically releases the lock when lck goes out of scope.
}

template<class Fitness, class Data>
const Data& FitnessQueue<Fitness, Data>::top() const
{
	ScopedLock lck(lock); // locks the mutex
	return this->queue.top()->second;
	// automatically releases the lock when lck goes out of scope.
}

template<class Fitness, class Data>
void FitnessQueue<Fitness, Data>::pop()
{
	ScopedLock lck(lock);//locks the mutex
	this->queue.pop();
	// automatically releases the lock when lck goes out of scope.
}
