/**
 * @file	TypeDefinitions.h
 * @date	Nov 1, 2012
 * @author	Adam Panzica
 * @brief	Common typedefs for the oryx_path_planning namespace
 */

#ifndef TYPEDEFINITIONS_H_
#define TYPEDEFINITIONS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace oryx_path_planning{

//*********************** TYPEDEFS ******************************//
///Typedef to allow for easier to read code
typedef pcl::PointXYZRGBA Point;

///Typedef to allow for convenient sharing of a PointCloud<pcl::PointXYZRGBA>
typedef pcl::PointCloud<Point> PointCloud;

///Typedef to allow for convenient sharing of a pcl::PointXYZRGBA via pointer
typedef boost::shared_ptr<Point> PointPtr;

///Typedef to allow for convenient sharing of a PointCloud<pcl::PointXYZRGBA> > via pointer
typedef boost::shared_ptr<pcl::PointCloud<Point> > PointCloudPtr;

}; /* oryx_path_planning */


#endif /* TYPEDEFINITIONS_H_ */
