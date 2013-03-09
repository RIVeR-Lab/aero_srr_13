/**
 * @file TentacleQueue.h
 * @author Adam Panzica
 * @date Mar 8, 2013
 * @brief  Provides an interface to a thread-safe priority queue for storing item fitness
 */

#ifndef TENTACLEQUEUE_H_
#define TENTACLEQUEUE_H_

//********************** SYSTEM DEPENDANCIES **********************//
#include<queue>
#include<boost/function.hpp>
#include<boost/shared_ptr.hpp>
#ifdef _OPENMP
# include <omp.h>
#endif
//********************** LOCAL  DEPENDANCIES **********************//
namespace aero_path_planning
{


#ifdef _OPENMP
/**
 * Mutex code provided by  Joel Yliluoma, http://bisqwit.iki.fi/story/howto/openmp/
 */
struct MutexType
{
	MutexType() { omp_init_lock(&lock); }
	~MutexType() { omp_destroy_lock(&lock); }
	void Lock() { omp_set_lock(&lock); }
	void Unlock() { omp_unset_lock(&lock); }

	MutexType(const MutexType& ) { omp_init_lock(&lock); }
	MutexType& operator= (const MutexType& ) { return *this; }
public:
	omp_lock_t lock;
};
#else
struct MutexType
{
	void Lock() {}
	void Unlock() {}
};
#endif

/* An exception-safe scoped lock-keeper. */
/**
 * ScopedLock code provided by  Joel Yliluoma, http://bisqwit.iki.fi/story/howto/openmp/
 */
struct ScopedLock
{
	explicit ScopedLock(MutexType& m) : mut(m), locked(true) { mut.Lock(); }
	~ScopedLock() { Unlock(); }
	void Unlock() { if(!locked) return; locked=false; mut.Unlock(); }
	void LockAgain() { if(locked) return; mut.Lock(); locked=true; }
private:
	MutexType& mut;
	bool locked;
private: // prevent copying the scoped lock.
	void operator=(const ScopedLock&);
	ScopedLock(const ScopedLock&);
};

/**
 * @author Adam Panzica
 * @brief  Provides a thread-safe priority_queue for storing data by fitness of data
 * @param Fitness The type of the fitness value to sort by. Must be comparable via the < operator
 * @param Data    The type of the data to store. Must be copy-assignable
 */
template <class Fitness, class Data>
class FitnessQueue
{
private:
	typedef std::pair<Fitness, Data> data_pair_t;			///Typedef for the data-fitness pair
	typedef boost::shared_ptr<data_pair_t> data_pair_ptr_t; ///Typedef for a shared_ptr to the data contained in the queue

	class comparitor
	{
		/**
		 * @author Adam Panzica
		 * @brief  Function for comparing two data-fitness pairs
		 * @param [in] a pointer to the first value to compare
		 * @param [in] b pointer to the second value to compare
		 * @return (a->first < b->first)
		 */
		bool operator() (const data_pair_ptr_t& a, const data_pair_ptr_t& b) const;

	};


	typedef std::priority_queue<data_pair_ptr_t, std::vector<data_pair_ptr_t>, comparitor > priority_queue;

	priority_queue queue;
	MutexType lock;

public:

	/**
	 * @author Adam Panzica
	 * @return True if the queue is empty, else false
	 */
	bool empty() const;

	/**
	 * @author Adam Panzica
	 * @return The number of elements in the queue
	 */
	unsigned int size() const;

	/**
	 * @author Adam Panzica
	 * @brief Pushes a new set onto the priority queue
	 * @param [in] fitness The fitness value of the data being placed on the queue
	 * @param [in] data    The data value being pushed onto the queue
	 */
	void push (const Fitness& fitness, const Data& data);

	/**
	 * @author Adam Panzica
	 * @brief  Returns the Data on the top of the queue
	 * @return Returns the Data on the top of the queue. Does not remove it from the queue
	 */
	const Data& top() const;

	/**
	 * @author Adam Panzica
	 * @brief Removes the item on the top of the queue. Calls its destructor
	 */
	void pop();

};

};

#endif /* TENTACLEQUEUE_H_ */
