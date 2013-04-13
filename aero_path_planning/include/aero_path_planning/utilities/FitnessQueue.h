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
#include<boost/shared_ptr.hpp>
#include<algorithm>
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
	MutexType() { omp_init_lock(&lock_); }
	~MutexType() { omp_destroy_lock(&lock_); }
	void Lock() { omp_set_lock(&lock_); }
	void Unlock() { omp_unset_lock(&lock_); }

	MutexType(const MutexType& ) { omp_init_lock(&lock_); }
	MutexType& operator= (const MutexType& ) { return *this; }
private:
	omp_lock_t lock_;
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
public:
	explicit ScopedLock(MutexType& m) : mut(m), locked(true) { mut.Lock(); }
	~ScopedLock() { Unlock(); }
	void Unlock() { if(!locked) return; locked=false; mut.Unlock(); }
	void LockAgain() { if(locked) return; mut.Lock(); locked=true; }
private:
	MutexType& mut;
	bool locked;
    // prevent copying the scoped lock.
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
	typedef std::pair<Fitness, Data> DataPair_t;			///Typedef for the data-fitness pair
	typedef boost::shared_ptr<DataPair_t> DataPairPtr_t; ///Typedef for a shared_ptr to the data contained in the queue

	class comparitor
	{
	public:
		/**
		 * @author Adam Panzica
		 * @brief  Function for comparing two data-fitness pairs
		 * @param [in] a pointer to the first value to compare
		 * @param [in] b pointer to the second value to compare
		 * @return (a->first < b->first)
		 */
		bool operator() (const DataPairPtr_t& a, const DataPairPtr_t& b) const
		{
			return a->first<b->first;
		}

	};


	typedef std::vector<DataPairPtr_t> PriorityQueue_t;

	PriorityQueue_t  queue_; ///The backing heap used to hold the data in the FitnessQueue
	mutable MutexType lock_;  ///The mutex used to lock the queue. Needs to be mutable to allow Top to be const, which it is in terms of queue data
	comparitor       comp_;
public:

	typedef typename PriorityQueue_t::iterator       iterator;
	typedef typename PriorityQueue_t::const_iterator const_iterator;

	FitnessQueue()
	{
		std::make_heap(this->queue_.begin(), this->queue_.end(), comp_);
	}

	/**
	 * @author Adam Panzica
	 * @return True if the queue is empty, else false
	 */
	bool empty() const
	{
		return this->queue_.empty();
	}

	/**
	 * @author Adam Panzica
	 * @return The number of elements in the queue
	 */
	unsigned int size() const
	{
		return this->queue_.size();
	}

	/**
	 * @author Adam Panzica
	 * @brief Pushes a new set onto the priority queue
	 * @param [in] fitness The fitness value of the data being placed on the queue
	 * @param [in] data    The data value being pushed onto the queue
	 */
	void push (const Fitness& fitness, const Data& data)
	{
		ScopedLock lck(this->lock_); // locks the mutex
		DataPairPtr_t item(new DataPair_t(fitness, data));
		this->queue_.push_back(item);
		std::push_heap(this->queue_.begin(), this->queue_.end(), comp_);
		// automatically releases the lock when lck goes out of scope.
	}

	/**
	 * @author Adam Panzica
	 * @brief  Returns the Data on the top of the queue
	 * @return Returns the Data on the top of the queue. Does not remove it from the queue
	 */
	const Data& top() const
	{
		ScopedLock lck(this->lock_); // locks the mutex
		return this->queue_.front()->second;
		// automatically releases the lock when lck goes out of scope.
	}

	/**
	 * @author Adam Panzica
	 * @brief Removes the item on the top of the queue. Calls its destructor
	 */
	void pop()
	{
		ScopedLock lck(this->lock_);//locks the mutex
		std::pop_heap(this->queue_.begin(), this->queue_.end(), comp_);
		this->queue_.pop_back();
		// automatically releases the lock when lck goes out of scope.
	}

	iterator begin()
	{
		return this->queue_.begin();
	}

	iterator end()
	{
		return this->queue_.end();
	}

	const_iterator c_begin() const
	{
		return this->queue_.begin();
	}
	const_iterator c_end() const
	{
		return this->queue_.end();
	}

};


};

#endif /* TENTACLEQUEUE_H_ */
