/*
 * Tentacle.cpp
 *
 *  Created on: Oct 11, 2012
 *      Author: parallels
 */

#include "Tentacles.h"


#if oryx_path_planner_VERBOSITY
#define PRINTER ROS_INFO
#else
#define PRINTER ROS_DEBUG
#endif


namespace oryx_path_planning{

//***************************** TENTACLE *********************************//
/**
 * This constructor just makes an empty tentacle
 */
Tentacle::Tentacle(){
	this->radius = 0;
	this->velocity = 0;
}

Tentacle::~Tentacle(){};

Tentacle::Tentacle(double expFact, double seedRad, int index, int numTent, double resolution, double xDim, double yDim, double velocity){

	this->velocity= velocity;
	PRINTER("Generating Tentacle %d", index);
	int halfwayIndex	= numTent/2;
	//Calculate tentacle radius. Formula taken from 'von Hundelshausen et al.: Integral Structures for Sensing and Motion'
	if(index<halfwayIndex){
		//Tentacle is to the right of halfway
		this->radius = std::pow(expFact,index)*seedRad;
	}else if(index>halfwayIndex){
		//Tentacle is to the left of halfway
		this->radius = -std::pow(expFact,index-(halfwayIndex+1))*seedRad;
	}else{
		//Tentacle is exactly at halfway
		this->radius = std::numeric_limits<double>::infinity();
	}
	PRINTER("Calculated Tentacle Radius=%f", this->radius);

	//Check for special case of an effectively straight line
	if(this->radius > straightThreshold || this->radius < -straightThreshold){
		this->radius = std::numeric_limits<double>::infinity();
		int numSteps = std::floor(yDim/resolution);
		for(int i = 0; i<numSteps; i++){
			tf::Point coord;
			coord.setX(std::floor(i));
			coord.setY(0);
			coord.setZ(0);
			this->points.push_back(coord);
		}
	}
	else{
		//Tracks the last coordinate so that we don't get duplicates
		tf::Point lastCoord;
		lastCoord.setZero();
		//The amount to increment theta by
		double thetaIncrement = oryx_path_planning::PI/360;
		//Push the first coordinate on
		this->points.push_back(lastCoord);
		//Calculate the X and Y coord along the tentacle
		if(this->radius>0){
			for(double t=PI; t>0; t-=thetaIncrement){
				tf::Point newCoord;
				Tentacle::calcCoord(this->radius, t, this->radius,resolution, newCoord);
				if(!(newCoord==lastCoord)){
					if(newCoord.getX()<0||newCoord.getY()>yDim) break;
					this->points.push_back(newCoord);
					lastCoord = newCoord;
				}
			}
		}
		else{
			for(double t=0; t<PI; t+=thetaIncrement){
				tf::Point newCoord;
				Tentacle::calcCoord(-this->radius, t, -this->radius,resolution, newCoord);
				if(!(newCoord==lastCoord)){
					if(newCoord.getX()<0||newCoord.getY()>yDim) break;
					this->points.push_back(newCoord);
					lastCoord = newCoord;
				}
			}
		}
	}
	PRINTER("Calculated a Tentacle with Number of Points=%d",(int)this->points.size());
}

std::vector<tf::Point >& Tentacle::getPoints(){
	return this->points;
}

/**
 * Calculated via the following formula:
 * @f[ y = floor(\frac{radius \times \cosine(theta)}{scale}+rshift) @f]
 * @f[ x = floor(\frac{radius \times \sine(theta)}{scale}) @f]
 */
void Tentacle::calcCoord(double radius, double theta, double rshift, double scale, tf::Point& result){
	result.setY(std::floor(radius*std::cos(theta)/scale+rshift));
	result.setX(std::floor(radius*std::sin(theta)/scale));
	result.setZ(0);
}


//***************************** SPEED SET *********************************//
/**
 * This constructor just makes an empty set of tentacles
 */
SpeedSet::SpeedSet(){
}


SpeedSet::SpeedSet(double expFact, double seedRad, int numTent, double resolution, double xDim, double yDim, double velocity){
	PRINTER("Generating a Speed Set with the Parameters <SRad=%f, Vel=%f, NumTent=%d, expF=%f>", seedRad, velocity, numTent, expFact);
	for(int t=0; t<numTent; t++){
		this->tentacles.push_back(Tentacle(expFact, seedRad, t, numTent, resolution, xDim, yDim, velocity));
	}
}

SpeedSet::~SpeedSet(){};

unsigned int SpeedSet::getNumTentacle(){
	return this->tentacles.size();
}

Tentacle& SpeedSet::getTentacle(int index){
	return this->tentacles.at(index);
}


//***************************** TENTACLE GENERATOR *********************************//
TentacleGenerator::TentacleGenerator(int numTentacles, double expFact, double resolution, double xDim, double yDim, std::vector<double>& speedSets){
	this->expFact 		= expFact;
	this->numTentacles	= numTentacles;
	ROS_INFO("Generating Speed Sets...");



	//Generate the SpeedSets
	for(unsigned int v=0; v<speedSets.size(); v++){
		this->speedSets.push_back(SpeedSet(expFact, calcSeedRad(v, speedSets.size()), numTentacles, resolution, xDim, yDim, speedSets.at(v)));
	}
	ROS_INFO("Speed Sets Complete!");
}

TentacleGenerator::~TentacleGenerator(){};

Tentacle& TentacleGenerator::getTentacle(int speedSet, int index){
	return this->speedSets.at(speedSet).getTentacle(index);
}

SpeedSet& TentacleGenerator::getSpeedSet(int speedSet){
	return this->speedSets.at(speedSet);
}

/**
 * calculate seed radius for the speed set. Formula taken from 'von Hundelshausen et al.: Integral Structures for Sensing and Motion'
 */
double TentacleGenerator::calcSeedRad(int speedSet, int numSpeedSet){
	double qdom = (double)(numSpeedSet-1);
	double dphi = 1.2*PI/2.0;
	double q	= ((double) speedSet)/qdom;
	double l	= 8+33.5*q*std::pow(q, 1.2);

	return l/(dphi*(1-std::pow(q,0.9)));
}

};
