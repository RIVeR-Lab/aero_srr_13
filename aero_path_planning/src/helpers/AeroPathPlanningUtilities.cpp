/**
 * @file AeroPathPlanningUtilities.cpp
 *
 * @date   Mar 2, 2013
 * @author parallels
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//

//*****************LOCAL DEPENDANCIES**************************//
#include <aero_path_planning/utilities/AeroPathPlanningUtilities.h>
//**********************NAMESPACES*****************************//

using namespace aero_path_planning;

ChainableException::ChainableException():
				std::runtime_error(""),
				cause_()
{
}


ChainableException::ChainableException(aero_path_planning::ChainableException& exception):
				std::runtime_error(exception),
				cause_(exception.cause_)
{
}


ChainableException::ChainableException(std::string& message):
				std::runtime_error(message),
				cause_()
{
}

ChainableException::ChainableException(std::string& message, std::exception& cause):
				std::runtime_error(genMessage(message, cause)),
				cause_(cause)
{
}

ChainableException::~ChainableException() throw(){};


std::exception& ChainableException::getCause()
{
	return this->cause_;
}


double aero_path_planning::roundToFrac(double raw, double frac)
{
	return std::floor(raw/frac)*frac;
}

int aero_path_planning::roundToGrid(double raw, double resolution)
{
	return (int)std::floor(raw/resolution);
}

double aero_path_planning::gridToReal(int grid, double resolution)
{
	return (double)grid*resolution;
}

void aero_path_planning::castLine(const Point& startPoint, const Point& endPoint, const int& rgba, PointCloud& cloud)
{
	//If we got the same start and end point, just push on the start point and finish
	if(startPoint.getVector4fMap() == endPoint.getVector4fMap())
	{
		cloud.push_back(startPoint);
	}
	else
	{
		int x_0 = startPoint.x, x_f= endPoint.x, x, delta_x, step_x;
		int y_0 = startPoint.y, y_f= endPoint.y, y, delta_y, step_y;
		int z_0 = startPoint.z, z_f= endPoint.z, z, delta_z, step_z;
		bool s_xy, s_xz;
		int error_xy, error_xz;
		int cx, cy, cz;
		int initial_size = cloud.size();


		//Figure out if the line is 'steep' along the xy plane
		s_xy = std::abs(y_f-y_0)>std::abs(x_f-x_0);
		if(s_xy){
			std::swap<int>(x_0, y_0);
			std::swap<int>(x_f, y_f);
		}
		//Figure out if the line is 'steep' along the xz plane
		s_xz = std::abs(z_f-z_0)>std::abs(x_f-x_0);
		if(s_xz){
			std::swap<int>(x_0, z_0);
			std::swap<int>(x_f, z_f);
		}

		//Calculate the delta in each axis
		delta_x = std::abs(x_f-x_0);
		delta_y = std::abs(y_f-y_0);
		delta_z = std::abs(z_f-z_0);

		//Calculate starting error values
		error_xy = delta_x/2;
		error_xz = delta_x/2;

		//Determine line direction
		(x_0>x_f)?(step_x=-1):(step_x=1);
		(y_0>y_f)?(step_y=-1):(step_y=1);
		(z_0>z_f)?(step_z=-1):(step_z=1);

		//Set up initial point
		y = y_0;
		z = z_0;

		//ROS_INFO("Calculated Line Parameters: s_xy=%s, s_xz=%s, delta_x=%d, delta_y=%d, delta_z=%d, step_x=%d, step_y=%d, step_z=%d",
		//(s_xy)?"true":"false",(s_xz)?"true":"false", delta_x, delta_y, delta_z, step_x, step_y, step_z);

		//Iterate across the line
		for(x = x_0; x!=x_f; x+=step_x){
			//store x,y,z for un-swapping
			cx = x; cy = y; cz = z;

			//unswap if needed
			if(s_xz) std::swap<int>(cx, cz);
			if(s_xy) std::swap<int>(cx, cy);

			//Write out the point
			Point point;
			point.x = cx;
			point.y = cy;
			point.z = cz;
			point.rgba = rgba;
			//PRINT_POINT("Line Calculated", point);
			cloud.push_back(point);

			//update the y and z axies
			error_xy -= delta_y;
			error_xz -= delta_z;

			//step y
			if(error_xy < 0){
				y+= step_y;
				error_xy += delta_x;
			}

			//step z
			if(error_xz < 0){
				z+= step_z;
				error_xz += delta_x;
			}
		}
		cloud.push_back(endPoint);

		//ROS_INFO("Line Generated, editing first/last point");
		//Set the RGBA values of the first and last point to their correct values
		cloud.at(cloud.size()-(cloud.size()-initial_size)).rgba = startPoint.rgba;
		cloud.at(cloud.size()-1).rgba = endPoint.rgba;
	}
}

void aero_path_planning::castArc(const int& radius, const double& sweep_angle, const int& rgba, const Point& origin, PointCloud& cloud, int quadrant, int plane)
{
	int x = 0, y = radius;
	int g = 3 - 2*radius;
	int diagonalInc = 10 - 4*radius;
	int rightInc = 6;
	//Calculate the halfway cuttoff (we can use symmetry after this point to speed up calculations)
	double halfCutoff = std::atan2(1.0,1.0);
	//Calculate the cutoff point. If it's greater than pi/4, this won't actually matter
	double fullCutoff = aero_path_planning::constants::PI()/2.0 - sweep_angle;
	//Calculate initial swept angle
	double sweptAngle = std::atan2(y-origin.y, x-origin.x);
	//Draw the first half of the arc, or the whole arc if the sweep angle is less than pi/4
	while (sweptAngle>fullCutoff&&sweptAngle>halfCutoff) {
		//Build the point.
		Point point;
		point.x = x;
		point.y = y;
		point.z = 0;
		point.rgba = rgba;
		//Calculate the currently swept angle
		sweptAngle = std::atan2(point.y, point.x);
		//Adjust the sign of the x/y values to account for quadrant changes
		switch(quadrant){
		case 2:
			point.x = -point.x;
			break;
		case 3:
			point.x = -point.x;
			point.y = -point.y;
			break;
		case 4:
			point.y = -point.y;
			break;
		}
		/*Point pointBottom;
	    pointBottom.x = point.y;
	    pointBottom.y = point.x;
	    pointBottom.z = point.z;
	    pointBottom.rgba = rgba;*/
		switch(plane){
		case 1:
			std::swap<float>(point.x, point.z);
			//std::swap<float>(pointBottom.x, pointBottom.z);
			break;
		case 2:
			std::swap<float>(point.y, point.z);
			//std::swap<float>(pointBottom.y, pointBottom.z);
			break;
		}
		if (g >=  0) {
			g += diagonalInc;
			diagonalInc += 8;
			y -= 1;
		}
		else {
			g += rightInc;
			diagonalInc += 4;
		}
		rightInc += 4;
		x += 1;
		//cloud.push_back(pointTop);
		//PRINT_POINT("Arc Generated Bottom Point", point);
		cloud.push_back(point);
	}
	//ROS_INFO("Reached Swept Angle %f, Goal Angle%f", sweptAngle, sweep_angle);
	//If the full arc was greater than pi/4, build the rest of the arc from symmetry
	if(sweep_angle>halfCutoff){
		for(int index=cloud.size()-1; index>=0; index--){
			Point& point = cloud.at(index);
			//otherwise push back the symmetric point
			Point pointSym;
			switch(plane){
			case 1:
				pointSym.x = point.x;
				pointSym.y = point.z;
				pointSym.z = point.y;
				break;
			case 2:
				pointSym.x = point.z;
				pointSym.y = point.y;
				pointSym.z = point.x;
				break;
			case 0:
			default:
				pointSym.x = point.y;
				pointSym.y = point.x;
				pointSym.z = point.z;
				break;
			}
			pointSym.rgba = point.rgba;
			//PRINT_POINT("Arc Generated Top Point", pointSym);
			cloud.push_back(pointSym);
			//ROS_INFO("Reached Swept Angle %f, Goal Angle%f", std::atan2(point.y, point.x), sweep_angle);
			//If the next point is past the sweep angle, than we're done. break
			sweptAngle = std::atan2(pointSym.y, pointSym.x);
			if(sweptAngle>sweep_angle){
				//ROS_INFO("I'm breaking cuz I got to the angle I wanted");
				break;
			}
		}
	}
	//If we're sweeping more than PI/2, need to use additional symmetry to finish the arc
	if(sweep_angle>aero_path_planning::constants::PI()/2){
		ROS_INFO("I'm doing the arc past pi/2");
		double remainingSweep = sweep_angle-aero_path_planning::constants::PI()/2;
		for(int index = cloud.size()-1; index>=0; index--){
			Point pointFinal;
			Point& point = cloud.at(index);
			switch(quadrant){
			case 2:
				break;
			case 3:
				break;
			case 4:
				break;
			case 0:
			default:
				pointFinal.x = point.x;
				pointFinal.y = -point.y;
				pointFinal.z = point.z;
				pointFinal.rgba = point.rgba;
				break;
			}
			cloud.push_back(pointFinal);
			Point checkPoint;
			checkPoint.getVector4fMap()=pointFinal.getVector4fMap()-origin.getVector4fMap();
			if(std::abs(std::atan(checkPoint.y/checkPoint.x))>remainingSweep) break;
		}
	}
}

