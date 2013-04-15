/**
 * @file   AeroLaserFilterUtilities.h
 *
 * @date   Apr 14, 2013
 * @author parallels
 * @brief  \TODO
 */

#ifndef AEROLASERFILTERUTILITIES_H_
#define AEROLASERFILTERUTILITIES_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <pcl/ros/conversions.h>
//************ LOCAL DEPENDANCIES ****************//

//***********    NAMESPACES     ****************//

namespace aero_laser_filter
{
typedef pcl::PointXYZ                   Point_t;
typedef pcl::PointCloud<Point_t>        PointCloud_t;
typedef boost::shared_ptr<PointCloud_t> PointCloudPtr_t;
};


#endif /* AEROLASERFILTERUTILITIES_H_ */
