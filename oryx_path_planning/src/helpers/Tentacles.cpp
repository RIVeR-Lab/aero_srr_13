/**
 * @file Tentacle.cpp
 *
 * @date Oct 11, 2012
 * @author Adam Panzica
 */

//************************************************ INCLUDES ***************************************//
#include "Tentacles.h"
#include"OryxPathPlannerConfig.h"

//************************************************ MACROS ***************************************//
#if oryx_path_planner_VERBOSITY
#define PRINTER ROS_INFO
#else
#define PRINTER ROS_DEBUG
#endif

#ifndef MIN_TENTACLE_LENGTH
#define MIN_TENTACLE_LENGTH 8.0
#else
#error MIN_TENTACLE_LENGTH is already defined!
#endif

#ifndef EXP_TENTACLE_LENGTH_BASE
#define EXP_TENTACLE_LENGTH_BASE 33.5
#else
#error EXP_TENTACLE_LENGTH_BASE is already defined!
#endif

#ifndef EXP_TENTACLE_LENGTH_FACTOR
#define EXP_TENTACLE_LENGTH_FACTOR 1.2
#else
#error EXP_TENTACLE_LENGTH_FACTOR is already defined!
#endif

#ifndef TENTACLE_SWEEP_ANGLE
#define TENTACLE_SWEEP_ANGLE (1.2)*PI/2.0
#else
#error TENTACLE_SWEEP_ANGLE is already defined!
#endif

#ifndef THETA_INCREMENT
#define THETA_INCREMENT PI/1800.0
#else
#error THETA_INCREMENT is already defined!
#endif

typedef oryx_path_planning::Tentacle::TentacleTraverser TentTrav;	///Namespace declaration to make implementation of TentacleTraverser easier

//************************************************ IMPLEMENTATION ***************************************//
namespace oryx_path_planning{

//******************** TENTACLE::TENTACLETRAVERSER ***********************//
/**
 * Sets both lastPoint and nextPoint to points.at(0), unless the points vector is empty in which case empty is set to true
 */
TentTrav::TentacleTraverser(Tentacle& tentacle){
	//If we have a normal set of points, initialize as normal
	if(tentacle.getPoints().size() > 1){
		this->start		= tentacle.getPoints().begin();
		this->end		= tentacle.getPoints().end();
		this->lastPoint	= &tentacle.getPoints().at(0);
		this->nextPoint	= &tentacle.getPoints().at(0);
		this->length	= 0;
		this->empty		= false;
	}
	//If we only got a single point, still initialize the point values, but set empty true
	else if(tentacle.getPoints().size() == 1){
		this->start		= tentacle.getPoints().begin();
		this->end		= tentacle.getPoints().end();
		this->lastPoint = &tentacle.getPoints().at(0);
		this->nextPoint = &tentacle.getPoints().at(0);
		this->empty 	= true;
	}
	//If we got a completely empty vector, set empty true, do not initialize point values
	else {
		this->empty = true;
		this->nextPoint = NULL;
		this->lastPoint = NULL;
	}
}

TentTrav::~TentacleTraverser(){
};

bool TentTrav::hasNext(){
	return !this->empty;
}

/**
 * Calling this method also updates the traversed length.
 * The traversed length is calculated using linear approximation:
 * @f[ l_t = \sum\limits_{n=1}^k d(p_(n-1), p_n) @f]
 * Where @f$ d(p_(k-1), p_k) @f$ is the linear distance between points @f$ p_(n-1) \text{ and } p_n @f$ .
 *
 */
const tf::Point& TentTrav::next(){
	//If they're not equal, and we're not empty, we're already in the traversal, process the next point
	if((*this->nextPoint != *this->lastPoint)&&!this->empty){
		//Update the calculated length along the Tentacle we've traversed
		this->length += tf::tfDistance(*this->lastPoint, *this->nextPoint);

		//Set lastPoint to nextPoint so that nextPoint can be updated if possible
		this->lastPoint = this->nextPoint;

		//Increment the traversal and check for end
		this->start++;
		if(this->start>= this->end){
			this->empty = true;
		}
		//Otherwise update nextPoint
		else{
			this->nextPoint = &*this->start;
		}

		//Return lastPoint, which was nextPoint when this method was originally called unless the traversal was empty
		return *this->lastPoint;
	}
	//If we're empty, just return lastPoint which will be the final point in the traversal
	else if(empty){
		return *this->lastPoint;
	}
	//If they're equal and we're not empty, we haven't started the traversal, so start it
	else{
		if(this->start != this->end){
			this->start++;
			this->nextPoint = &*this->start;
		} else this->empty = true;
		return *this->lastPoint;
	}
}


double TentTrav::lengthTraversed(){
	return this->length;
}

//***************************** TENTACLE *********************************//
/**
 * This constructor just makes an empty tentacle
 */
Tentacle::Tentacle(){
	this->radius = 0;
	this->velocity = 0;
}

Tentacle::~Tentacle(){};

Tentacle::Tentacle(double expFact, double seedRad, int index, int numTent, double resolution, double xDim, double yDim, double velocity) throw (TentacleGenerationException){

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
	if(this->radius > this->straightThreshold || this->radius < -this->straightThreshold){
		this->radius = std::numeric_limits<double>::infinity();
		for(double i = 0; i<xDim; i+=resolution){
			tf::Point coord;
			coord.setX(i);
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
		double thetaIncrement	= PI/((5.0/resolution)*std::floor(std::abs(this->radius)));
		double sweepAngle		= TENTACLE_SWEEP_ANGLE;
		//Push the first coordinate on
		this->points.push_back(lastCoord);
		//Calculate the X and Y coord along the tentacle
		if(this->radius>0){
			double startAngle = PI/2.0;
			for(double t=startAngle; t>(startAngle-sweepAngle); t-=thetaIncrement){
				tf::Point newCoord;
				newCoord.setY(oryx_path_planning::roundToFrac(this->radius*std::sin(t)-this->radius, resolution));
				newCoord.setX(oryx_path_planning::roundToFrac(this->radius*std::cos(t), resolution));
				newCoord.setZ(0);
				//If we've hit the top of the occupancy grid, break
				if(newCoord.getX()>xDim||std::abs(newCoord.getY())>yDim) break;
				//Otherwise push_back the next point if it's not the same as the previous point
				if(!((newCoord==lastCoord)||(newCoord.getX()<0)/*||(std::abs(newCoord.getY()>yDim))*/)){
					this->points.push_back(newCoord);
					lastCoord = newCoord;
				}
			}
		}
		else{
			double startAngle = PI/2.0;
			for(double t=startAngle; t<(startAngle+sweepAngle); t+=thetaIncrement){
				tf::Point newCoord;
				newCoord.setY(oryx_path_planning::roundToFrac(this->radius*std::sin(t)-radius, resolution));
				newCoord.setX(oryx_path_planning::roundToFrac(this->radius*std::cos(t), resolution));
				newCoord.setZ(0);
				//If we've hit the top of the occupancy grid, break
				if(newCoord.getX()>xDim||std::abs(newCoord.getY())>yDim) break;
				//Otherwise push_back the next point if it's not the same as the previous point
				if(!((newCoord==lastCoord)||(newCoord.getX()<0)/*||(std::abs(newCoord.getY()>yDim))*/)){
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
/*void Tentacle::calcCoord(double radius, double theta, double rshift, double scale, tf::Point& result){

}*/


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

Tentacle& SpeedSet::getTentacle(int index)throw(oryx_path_planning::TentacleAccessException){
	if(index<0||index>(int)getNumTentacle()) throw new oryx_path_planning::TentacleAccessException(index, 0);
	try{
		return this->tentacles.at(index);
	}catch (std::exception& e){
		std::string message("Something went wrong!");
		throw new oryx_path_planning::TentacleAccessException(index, 0, message, e);
	}
}


//***************************** TENTACLE GENERATOR *********************************//
TentacleGenerator::TentacleGenerator(double minSpeed, double maxSpeed, int numSpeedSet, int numTentacles, double expFact, double resolution, double xDim, double yDim){
	this->expFact 		= expFact;
	this->numTentacles	= numTentacles;
	this->numSpeedSet	= numSpeedSet;
	ROS_INFO("Generating Speed Sets...");

	double q = 0;
	//Generate the SpeedSets
	for(int v=0; v<numSpeedSet; v++){
		q = calcQ(v);
		ROS_INFO("Calculated q=%f",q);
		this->speedSets.push_back(SpeedSet(expFact, calcSeedRad(v, q), numTentacles, resolution, xDim, yDim, calcSpeedSetVel(minSpeed, maxSpeed, q)));
	}
	ROS_INFO("Speed Sets Complete!");
}

TentacleGenerator::~TentacleGenerator(){};

int TentacleGenerator::getNumSpeedSets(){
	return this->speedSets.size();
}

Tentacle& TentacleGenerator::getTentacle(int speedSet, int index) throw (oryx_path_planning::TentacleAccessException, oryx_path_planning::SpeedSetAccessException){
	if(speedSet<0||speedSet>(int)this->speedSets.size()) throw new oryx_path_planning::SpeedSetAccessException(speedSet);
	if(index<0||index>(int)this->speedSets.at(speedSet).getNumTentacle()) throw new oryx_path_planning::TentacleAccessException(index, speedSet);
	try{
		return this->speedSets.at(speedSet).getTentacle(index);
	}catch(std::exception& e){
		std::string message("Something Went Wrong!");
		throw new oryx_path_planning::TentacleAccessException(index, speedSet, message, e);
	}
}

SpeedSet& TentacleGenerator::getSpeedSet(int speedSet){
	return this->speedSets.at(speedSet);
}

/**
 * calculate seed radius for the speed set. Formula taken from 'von Hundelshausen et al.: Integral Structures for Sensing and Motion'
 * Equations are as follows:
 * @f[ delta phi = TSA \times \frac{pi}{2} @f]
 * @f[ l = l_min + E_b \times q^(E_f) @f]
 * @f[ R_j = \frac{l}{delta phi \times (1-q^(0.9))} @f]
 *
 * Where @f$ R_j = \text{Seed Radius for Speed Set j} @f$, @f$ delta phi =@f$ the arc swept by the smallest tentacle,
 * @f$ l=@f$ length of the longest tentacle in the set, and @f$ TSA, E_b, E_f@f$ are all tuning constants determined empirically.
 */
double TentacleGenerator::calcSeedRad(int speedSet, double q){
	double dphi = TENTACLE_SWEEP_ANGLE;
	double l	= MIN_TENTACLE_LENGTH+EXP_TENTACLE_LENGTH_BASE*std::pow(q, EXP_TENTACLE_LENGTH_FACTOR);
	ROS_INFO("Calculated dphi=%f, l=%f",dphi, l);
	return l/(dphi*(1-std::pow(q,0.9)));
}

/**
 * calculate velocity for the speed set. Formula taken from 'von Hundelshausen et al.: Integral Structures for Sensing and Motion'
 * Equations are as follows:
 * @f[ V_j = V_s+q^(1.2) \times (V_e-V_s) @f]
 *
 * Where @f$ V_j = \text{Velocity for Speed Set j} @f$, @f$ delta phi =@f$ the arc swept by the smallest tentacle,
 * @f$ l=@f$ length of the longest tentacle in the set, and @f$ TSA, E_b, E_f@f$ are all tuning constants determined empirically.
 */
double TentacleGenerator::calcSpeedSetVel(double minSpeed, double maxSpeed, double q){
	return minSpeed+std::pow(q, 1.2)*(maxSpeed-minSpeed);
}

/**
 * Calculates 'magic constant' q. Formula taken from 'von Hundelshausen et al.: Integral Structures for Sensing and Motion'
 */
double TentacleGenerator::calcQ(int speedSet){
	return (double)speedSet/((double)this->numSpeedSet-1);
}

};

//************************************************ UNDEFS ***************************************//
#undef MIN_TENTACLE_LENGTH
#undef EXP_TENTACLE_LENGTH_BASE
#undef EXP_TENTACLE_LENGTH_FACTOR
#undef TENTACLE_SWEEP_ANGLE
#undef THETA_INCREMENT
