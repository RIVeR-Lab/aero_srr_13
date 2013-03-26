/**
 * @file   DetectionManager.h
 *
 * @date   Mar 22, 2013
 * @author Adam Panzica
 * @brief  Class definition for DetectionManager
 */

#ifndef DETECTIONMANAGER_H_
#define DETECTIONMANAGER_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include <ros/ros.h>
#include <tf/tf.h>
#include <boost/shared_ptr.hpp>
//************ LOCAL DEPENDANCIES ****************//

//***********    NAMESPACES     ****************//
namespace object_locator
{

class DetectionManager
{
private:
	typedef std::pair<tf::Point, double> Detection_t;
	typedef boost::shared_ptr<Detection_t> DetectionPtr;
	typedef std::list<DetectionPtr> DetectionArray_t;
public:
	typedef DetectionArray_t::size_type size_type;

	/**
	 * @author Adam Panzica
	 * @param threshold_dist The maximum distance to consider two detections equal
	 * @param growth_rate    The rate to increase confidence in a detection by
	 * @param shrink_rate    The rate to decrease confidence in a detection by
	 * @param threshold_det  The minimum confidence to consider a detection a positive one
	 */
	DetectionManager(double threshold_dist, double growth_rate, double shrink_rate, double threshold_det);

	/**
	 * @author Adam Panzica
	 * @brief Gets the number of detections in the manager
	 * @return The number of detections in the manager
	 */
	size_type size() const;

	/**
	 * @author Adam Panzica
	 * @return True if empty, else false
	 */
	bool isEmpty() const;

	/**
	 * @author Adam Panzica
	 * @brief  Computes the average confidence of detections in the manager
	 * @return The average confidence of detections in the manager
	 */
	double averageConfidance() const;


	/**
	 * @author Adam Panzica
	 * @brief  Adds a detection to the manager
	 * @param [in] detection The detection to add. Should be in the same frame as all other added detections
	 *
	 * This method processes a new detection. If the detection is within the specified threshold distance of another detection,
	 * it will increase the confidence of that detection, otherwise it will create a new detection with a default starting confidance
	 */
	void addDetection(const tf::Point& detection);

	/**
	 * @author Adam Panzica
	 * @brief  Reduces the confidence in all detections by the shrink rate
	 */
	void shrink();

	/**
	 * @author Adam Panzica
	 * @brief  Gets the most confident detection that passes the detection threshold
	 * @param [out] detection  Point to write the best detection to, if any
	 * @param [out] confidence The confidence value of the returned detection, if any
	 * @return True if there was a point that was past the confidence threshold, else false
	 */
	bool getDetection(tf::Point& detection, double& confidence) const;

	/**
	 * @author Adam Panzica
	 * @brief  Clears all detections in the manager
	 */
	void clear();

private:

	double threshold_dist_;
	double growth_rate_;
	double shrink_rate_;
	double threshold_det_;
	double max_condifdence_;
	DetectionPtr     highest_confidance_;
	DetectionArray_t detections_;
};

};


#endif /* DETECTIONMANAGER_H_ */
