/**
 * @file RRTCarrot.h
 *
 * @date   Feb 19, 2013
 * @author Adam Panzica
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//

//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/CarrotPathFinder.h>
//**********************NAMESPACES*****************************//

#ifndef RRTCARROT_H_
#define RRTCARROT_H_

namespace aero_path_planning
{

class RRTCarrot: public CarrotPathFinder
{
public:
	RRTCarrot();
	RRTCarrot(const RRTCarrot& copy);
	virtual RRTCarrot();

	virtual bool setCarrotDelta(int delta);
	virtual bool setSearchMap(const aero_path_planning::OccupancyGrid& map);
	virtual bool setCollision(collision_func_& collision_checker);
	virtual bool search(const aero_path_planning::Point& start_point, const aero_path_planning::Point& goal_point, std::queue& result_path);
	virtual bool getType(std::string& type) const;

	RRTCarrot& operator=(RRTCarrot const &copy);

private:
	void isInialized();


	bool initialized_;
	bool has_delta_;
	bool has_coll_;
	bool has_map_;
	int delta_;

	aero_path_planning::OccupancyGrid map_;
	collision_func_ collision_checker_;
};

};
#endif /* RRTCARROT_H_ */
