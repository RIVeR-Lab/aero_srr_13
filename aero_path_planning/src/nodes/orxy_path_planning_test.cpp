/**
 * @file	orxy_path_planning_test.cpp
 * @date	Oct 11, 2012
 * @author	Adam Panzica
 * @brief	Simple test node
 */
#include <ros/ros.h>
#include <aero_path_planning/utilities/OryxPathPlanning.h>
using namespace aero_path_planning;
void printSpeedSet(int xDim, int yDim, double resoltuion, SpeedSet& speedSet);

int main(int argc, char **argv) {
	ros::init(argc, argv, "oryx_path_planning_test");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");


	ROS_INFO("Testing Tentacle Generator...");
	//*****************Configuration Parameters*******************//
	//Minimum update rate expected of occupancy grid
	std::string p_up_rate("occupancy/update_rate");
	double update_rate = 0.2;
	std::string up_rate_msg("");
	up_rate_msg+= boost::lexical_cast<double>(update_rate);
	up_rate_msg+="s";

	//x dimension of occupancy grid
	std::string p_x_dim("occupancy/x_dimension");
	double xDim = 20;
	std::string xDim_msg("");
	xDim_msg+= boost::lexical_cast<double>(xDim);
	xDim_msg+="m";

	//y dimension of occupancy grid
	std::string p_y_dim("occupancy/y_dimension");
	double yDim = 20;
	std::string yDim_msg("");
	yDim_msg+= boost::lexical_cast<double>(yDim);
	yDim_msg+="m";

	//z dimension of occupancy grid
	std::string p_z_dim("occupancy/z_dimension");
	double zDim = 0;
	std::string zDim_msg("");
	zDim_msg+= boost::lexical_cast<double>(zDim);
	zDim_msg+="m";

	//resolution occupancy grid
	std::string p_res("occupancy/grid_resolution");
	double res = .1;
	std::string p_res_msg("");
	p_res_msg+= boost::lexical_cast<double>(res);
	p_res_msg+="m";

	//x coord of the origin of the occupancy grids
	std::string p_x_ori("occupancy/x_origin");
	double x_ori = 0;
	std::string p_x_ori_msg("");
	p_x_ori_msg+= boost::lexical_cast<double>(x_ori);
	p_x_ori_msg+="m";

	//z coord of the origin of the occupancy grids
	std::string p_z_ori("occupancy/z_origin");
	double z_ori = 0;
	std::string p_z_ori_msg("");
	p_z_ori_msg+= boost::lexical_cast<double>(z_ori);
	p_z_ori_msg+="m";

	//y coord of the origin of the occupancy grids
	std::string p_y_ori("occupancy/y_origin");
	double y_ori = yDim/2;
	std::string p_y_ori_msg("");
	p_y_ori_msg+= boost::lexical_cast<double>(y_ori);
	p_y_ori_msg+="m";


	//number of tentacles per speed set
	std::string p_numTent("tentacles/number");
	int numTent = 81;
	std::string p_numTent_msg("");
	p_numTent_msg+= boost::lexical_cast<double>(numTent);
	p_numTent_msg+=" Tentacles";

	//Exponential Factor to use for generating seed radii
	std::string p_expFact("tentacles/exp_factor");
	double expFact = 1.15;
	std::string p_expFact_msg("");
	p_numTent_msg+= boost::lexical_cast<double>(expFact);
	p_numTent_msg+=" Tentacles";

	//number of tentacles per speed set
	std::string p_numSpeedSet("speed_set/number");
	int numSpeedSet = 15;
	std::string p_numSpeedSet_msg("");
	p_numSpeedSet_msg+= boost::lexical_cast<double>(numSpeedSet);
	p_numSpeedSet_msg+= " Speed Sets";

	//Max Speed
	std::string p_maxSpeed("speed_set/max_speed");
	double maxSpeed = 1;
	std::string p_maxSpeed_msg("");
	p_maxSpeed_msg+= boost::lexical_cast<double>(maxSpeed);
	p_maxSpeed_msg+="m/s";

	//Min Speed
	std::string p_minSpeed("speed_set/min_speed");
	double minSpeed = 1;
	std::string p_minSpeed_msg("");
	p_minSpeed_msg+= boost::lexical_cast<double>(minSpeed);
	p_minSpeed_msg+="m/s";

	if(!nh.getParam(p_up_rate,	update_rate))	PARAM_WARN(p_up_rate,	up_rate_msg);
	if(!nh.getParam(p_x_dim,	xDim))			PARAM_WARN(p_x_dim,		xDim_msg);
	if(!nh.getParam(p_y_dim,	yDim))			PARAM_WARN(p_y_dim,		yDim_msg);
	if(!nh.getParam(p_z_dim,	zDim))			PARAM_WARN(p_z_dim,		zDim_msg);
	if(!nh.getParam(p_res,		res))			PARAM_WARN(p_res,		p_res_msg);
	if(!nh.getParam(p_numTent,	numTent))		PARAM_WARN(p_numTent,	p_numTent_msg);
	if(!nh.getParam(p_expFact,	expFact))		PARAM_WARN(p_expFact,	p_expFact_msg);
	if(!nh.getParam(p_numSpeedSet, numSpeedSet))PARAM_WARN(p_numSpeedSet,	p_numSpeedSet_msg);
	if(!nh.getParam(p_maxSpeed,	maxSpeed))		PARAM_WARN(p_maxSpeed,	p_maxSpeed_msg);
	if(!nh.getParam(p_minSpeed,	minSpeed))		PARAM_WARN(p_minSpeed,	p_minSpeed_msg);

	//Build The Tentacles
	int firstSpeedSet=0;
	int lastSpeedSet=numSpeedSet;
	TentacleGenerator generator(minSpeed, maxSpeed, 2, numTent, expFact, res, xDim, yDim/2);
	ROS_INFO("Tentacles Generated. Printing Speed Sets...");
	if(!nh.getParam("first_speed_set", firstSpeedSet))ROS_WARN("First Speed Set Not Set! Using Default %d", firstSpeedSet);
	if(!nh.getParam("last_speed_set", lastSpeedSet))ROS_WARN("Last Speed Set Not Set! Using Default %d", lastSpeedSet);

	//Print out the Speed Sets
	for(int s=0/*firstSpeedSet*/; s<1/*lastSpeedSet*/; s++){
		SpeedSet speedSet = generator.getSpeedSet(s);
		//printSpeedSet(xDim, yDim, res, speedSet);
	}

	ROS_INFO("Speed Sets Printed!");

	/*//Test a Tentacle Traverser
	ROS_INFO("Testing Tentacle Traversal");
	try{
		Tentacle workingTentacle = generator.getTentacle(firstSpeedSet, 0);
		Tentacle::TentacleTraverser traverser(workingTentacle);
		while(traverser.hasNext()){
			aero_path_planning::Point travPoint = traverser.next();
			ROS_INFO("Traversed Point <%f, %f>, total length = %f", travPoint.x, travPoint.y, traverser.lengthTraversed());
		}
	}catch (std::exception& e){
		ROS_ERROR("%s",e.what());
	}

	//Test some exceptions
	ROS_INFO("Test Some Exceptions:");

	aero_path_planning::SpeedSetAccessException testE1(100);
	aero_path_planning::TentacleAccessException testE2(1, 100, *(new std::string("Testing of Chained Exceptions")), testE1);

	try{
		throw testE2;
	}catch (std::exception& e){
		ROS_ERROR("%s", e.what());
	}

	//Test Point Converter
	ROS_INFO("Testing Point Converter...");
	aero_path_planning::Point unConvertedPoint;
	unConvertedPoint.x = 100;
	unConvertedPoint.y = 50;
	unConvertedPoint.z = 500;
	unConvertedPoint.rgba = 0;
	aero_path_planning::PointConverter converter(.001);
	aero_path_planning::Point convertedPoint;
	converter.convertToEng(unConvertedPoint, convertedPoint);
	PRINT_POINT("Unconverted:", unConvertedPoint);
	PRINT_POINT("Converted:", convertedPoint);*/

	//Build the base occupancy grid
	ROS_INFO("Testing Occupancy Grid...");
	aero_path_planning::Point origin;
	origin.x=0;
	origin.y=yDim/2;
	origin.z=0;
	aero_path_planning::OccupancyGrid testGrid(xDim,yDim, res, origin, aero_path_planning::FREE_LOW_COST);
	ROS_INFO("Occupancy Grid Built");

	try{
//		//Place a single point on the grid
//		//ROS_INFO("\n%s", testGrid.toString(2,0).get()->c_str());
//		ROS_INFO("Testing setPoint...");
//		aero_path_planning::Point testPoint;
//		testPoint.x = 0;
//		testPoint.y = origin.y;
//		testPoint.z = 0;
//		testPoint.rgba = aero_path_planning::ROBOT;
//		PRINT_POINT("Placing Point", testPoint);
//		testGrid.setPoint(testPoint, false);
//		//ROS_INFO("Resulting Grid:\n%s", testGrid.toString(0,0)->c_str());
//
//		//Place a line of points on the grid
//		ROS_INFO("Placing some test points on the grid");
//		PointCloud lineCloud;
//		Point lineStart;
//		lineStart.x = 0;
//		lineStart.y = yDim-origin.y;
//		lineStart.z = 0;
//		lineStart.rgba = aero_path_planning::OBSTACLE;
//		PRINT_POINT("Start Point", lineStart);
//		Point lineEnd;
//		lineEnd.x = xDim;
//		lineEnd.y = 0;
//		lineEnd.z = 0;
//		lineEnd.rgba = aero_path_planning::GOAL;
//		PRINT_POINT("End Point", lineEnd);
//		castLine(lineStart, lineEnd, aero_path_planning::OBSTACLE, lineCloud);
//		ROS_INFO("Line Generated, placing on grid...");
//		for(PointCloud::iterator line_itr = lineCloud.begin(); line_itr<lineCloud.end(); line_itr++){
//			//PRINT_POINT("Line Point", (*line_itr));
//			testGrid.setPointTrait(*line_itr, (aero_path_planning::PointTrait_t)line_itr->rgba);
//		}
//		//ROS_INFO("\n%s", testGrid.toString(2,0).get()->c_str());

		//Test arc generator
		ROS_INFO("Testing arc generation...");
		PointCloud arcCloud;
		Point arcCenter;
		arcCenter.x = 0;
		arcCenter.y = -55;
		arcCenter.z = 0;
		arcCenter.rgba = aero_path_planning::TENTACLE;
		castArc(48, 2.312500, aero_path_planning::OBSTACLE, arcCenter, arcCloud);
		ROS_INFO("Arc generated, placing on grid...");
		for(PointCloud::iterator arc_itr = arcCloud.begin(); arc_itr<arcCloud.end(); arc_itr++){
			PRINT_POINT("Arc Point", (*arc_itr));
			testGrid.setPointTrait(*arc_itr, (aero_path_planning::PointTrait_t)arc_itr->rgba);
		}
		//PRINT_POINT("Setting a Point on the Grid at Arc Origin", arcCenter);
		testGrid.setPointTrait(arcCenter, (aero_path_planning::PointTrait_t)arcCenter.rgba);
		ROS_INFO("\n%s", testGrid.toString(2,0).get()->c_str());
//		//Place a known straight line on the grid
//		for(double x=0; x<xDim; x+=res){
//			testGrid.setPointTrait(x, -origin.y, 0.0, aero_path_planning::OBSTACLE);
//		}
//
//		//Test copying occupancy grids
//		ROS_INFO("Testing Occupancy Grid Copy...");
//		OccupancyGrid copyGrid(testGrid);
//		//ROS_INFO("Copied Grid:\n%s", copyGrid.toString(0,0)->c_str());
//
//		//Test building a grid from an existing point cloud
//		ROS_INFO("Testing Build From Point Cloud...");
//		OccupancyGrid cloudGrid(xDim,yDim,0.0,res,origin, copyGrid.getGrid());
//		//ROS_INFO("PC Built Grid:\n%s", cloudGrid.toString(0,0)->c_str());
//		ROS_INFO("Data at 10,10,0 <%x>", cloudGrid.getPointTrait(10,10,0));
//
//
		//Overlay tentacles on grid
		ROS_INFO("Testing Tentacle Overlay...");
		for(unsigned int index =0; index<generator.getSpeedSet(0).getNumTentacle(); index++){
			const Tentacle& tentacle = generator.getSpeedSet(0).getTentacle(index);
			for(unsigned int pi = 0; pi<tentacle.getPoints().size(); pi++){
				const Point& point = tentacle.getPoints().at(pi);
				if(std::abs(point.x)<xDim && std::abs(point.y)<yDim){
					//ROS_INFO("Placing Point <%f,%f,%f,%x>", point.x, point.y, point.z, aero_path_planning::TENTACLE);
					try{
						testGrid.setPointTrait(point, aero_path_planning::TENTACLE);
					}catch(std::exception& e){

					}
				}
			}
		}
		ROS_INFO("Tentacle Overlay:\n%s", testGrid.toString(0,0)->c_str());
	}catch(std::exception& e){
		ROS_ERROR(e.what());
	}
	return 0;
}


void printSpeedSet(int xDim, int yDim, double resolution, SpeedSet& speedSet){
	int xSize = xDim;
	int ySize = yDim;
	unsigned int numTent = speedSet.getNumTentacle();
	std::vector<std::string> occGrid(xSize+1, std::string(ySize+1, '_'));
	ROS_INFO("Speed Set Gird Generated");
	for(int t=0; t<numTent; t++){
		Tentacle tentacle = speedSet.getTentacle(t);
		//ROS_INFO("Got Tentacle %d", t);
		for(unsigned int p=0; p<tentacle.getPoints().size(); p++){
			aero_path_planning::Point point(tentacle.getPoints().at(p));
			//ROS_INFO("Got Point at <%f, %f>", point.x, point.y);
			point.y+=ySize/2;
			//ROS_INFO("Placing Point at <%f, %f>", point.x, point.y);
			if(point.x<xDim&&point.y<yDim&&point.x>=0&&point.y>=0)occGrid.at(point.x).replace(point.y,1,"T");
		}
		//ROS_INFO("Done with Tentacle %d", t);
	}
	std::string output;

	for(unsigned int i=0;i<occGrid.size(); i++){
		output+=occGrid.at(i);
		output+="\r\n";
	}

	ROS_INFO("\n%s", output.c_str());
}
