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
	if(tentacle.points.size() > 1){
		this->start		= tentacle.points.begin();
		this->end		= tentacle.points.end();
		this->lastPoint	= &tentacle.points.at(0);
		this->nextPoint	= &tentacle.points.at(0);
		this->length	= 0;
		this->empty		= false;
	}
	//If we only got a single point, still initialize the point values, but set empty true
	else if(tentacle.points.size() == 1){
		this->start		= tentacle.points.begin();
		this->end		= tentacle.points.end();
		this->lastPoint = &tentacle.points.at(0);
		this->nextPoint = &tentacle.points.at(0);
		this->empty 	= true;
	}
	//If we got a completely empty vector, set empty true, do not initialize point values
	else {
		this->empty = true;
		this->nextPoint = NULL;
		this->lastPoint = NULL;
	}
}

/**
 * Sets both lastPoint and nextPoint to points.at(0), unless the points vector is empty in which case empty is set to true
 */
TentTrav::TentacleTraverser(const Tentacle& tentacle){
	//If we have a normal set of points, initialize as normal
	if(tentacle.points.size() > 1){
		this->start		= tentacle.points.begin();
		this->end		= tentacle.points.end();
		this->lastPoint	= &tentacle.points.at(0);
		this->nextPoint	= &tentacle.points.at(0);
		this->length	= 0;
		this->empty		= false;
	}
	//If we only got a single point, still initialize the point values, but set empty true
	else if(tentacle.points.size() == 1){
		this->start		= tentacle.points.begin();
		this->end		= tentacle.points.end();
		this->lastPoint = &tentacle.points.at(0);
		this->nextPoint = &tentacle.points.at(0);
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

bool TentTrav::hasNext() const{
	return !this->empty;
}

/**
 * Calling this method also updates the traversed length.
 * The traversed length is calculated using linear approximation:
 * @f[ l_t = \sum\limits_{n=1}^k d(p_(n-1), p_n) @f]
 * Where @f$ d(p_(k-1), p_k) @f$ is the linear distance between points @f$ p_(n-1) \text{ and } p_n @f$ .
 *
 */
const oryx_path_planning::Point& TentTrav::next(){
	//If they're not equal, and we're not empty, we're already in the traversal, process the next point
	if((this->nextPoint != this->lastPoint)&&!this->empty){
		//Update the calculated length along the Tentacle we've traversed
		Eigen::Vector4f lastPM(this->lastPoint->getVector4fMap());
		Eigen::Vector4f nextPM(this->nextPoint->getVector4fMap());
		this->length += pcl::distances::l2(lastPM, nextPM);

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


double TentTrav::lengthTraversed() const{
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

Tentacle::Tentacle(const Tentacle& Tentacle):
				points(Tentacle.points){
	this->radius = Tentacle.radius;
	this->velocity = Tentacle.velocity;
}

Tentacle::~Tentacle(){};

Tentacle::Tentacle(double expFact, double seedRad, int index, int numTent, double resolution, int xDim, int yDim, double velocity) throw (TentacleGenerationException):
					points(){

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
		for(double i = 0; i<xDim; i++){
			oryx_path_planning::Point coord;
			coord.x=i;
			coord.y=0;
			coord.z=0;
			this->points.push_back(coord);
		}
	}
	else{
		//Convert the radius, which will be in engineering units, into grid coordinates
		int workingRadius = roundToGrid(radius, resolution);
	}
//	else{
//		//Tracks the last coordinate so that we don't get duplicates
//		oryx_path_planning::Point lastCoord;
//		lastCoord.x = 0;
//		lastCoord.y = 0;
//		lastCoord.z = 0;
//		//The amount to increment theta by
//		double thetaIncrement	= PI/((100.0/resolution)*std::abs(this->radius));
//		double sweepAngle		= TENTACLE_SWEEP_ANGLE;
//		//Push the first coordinate on
//		this->points.push_back(lastCoord);
//		//Calculate the X and Y coord along the tentacle
//		if(this->radius>0){
//			double startAngle = PI/2.0;
//			for(double t=startAngle; t>(startAngle-sweepAngle); t-=thetaIncrement){
//				oryx_path_planning::Point newCoord;
//				newCoord.y = (oryx_path_planning::roundToFrac(this->radius*std::sin(t)-this->radius, resolution));
//				newCoord.x = (oryx_path_planning::roundToFrac(this->radius*std::cos(t), resolution));
//				newCoord.z = 0;
//				//If we've hit the top of the occupancy grid, break
//				if(newCoord.x>xDim||std::abs(newCoord.y)>yDim) break;
//				//Otherwise push_back the next point if it's not the same as the previous point
//				if(!((((Eigen::Vector4f)newCoord.getVector4fMap())==(((Eigen::Vector4f)lastCoord.getVector4fMap())))||(newCoord.x<0)/*||(std::abs(newCoord.getY()>yDim))*/)){
//					this->points.push_back(newCoord);
//					lastCoord = newCoord;
//				}
//			}
//		}
//		else{
//			double startAngle = PI/2.0;
//			for(double t=startAngle; t<(startAngle+sweepAngle); t+=thetaIncrement){
//				oryx_path_planning::Point newCoord;
//				newCoord.y = (float)(oryx_path_planning::roundToFrac(this->radius*std::sin(t)-radius, resolution));
//				newCoord.x = (float)(oryx_path_planning::roundToFrac(this->radius*std::cos(t), resolution));
//				newCoord.z = 0;
//				//If we've hit the top of the occupancy grid, break
//				if(newCoord.x>xDim||std::abs(newCoord.y)>yDim) break;
//				//Otherwise push_back the next point if it's not the same as the previous point
//				if(!((((Eigen::Vector4f)newCoord.getVector4fMap())==(((Eigen::Vector4f)lastCoord.getVector4fMap())))||(newCoord.x<0)/*||(std::abs(newCoord.getY()>yDim))*/)){
//					this->points.push_back(newCoord);
//					lastCoord = newCoord;
//				}
//			}
//		}
//	}
	PRINTER("Calculated a Tentacle with Number of Points=%d",(int)this->points.size());
}

const Tentacle::TentaclePointCloud& Tentacle::getPoints() const{
	return this->points;
}

/**
 * Calculated via the following formula:
 * @f[ y = floor(\frac{radius \times \cosine(theta)}{scale}+rshift) @f]
 * @f[ x = floor(\frac{radius \times \sine(theta)}{scale}) @f]
 */
/*void Tentacle::calcCoord(double radius, double theta, double rshift, double scale, tf::Point& result){

}*/

Tentacle::iterator Tentacle::begin(){
	return this->points.begin();
}

Tentacle::iterator Tentacle::end(){
	return this->points.end();
}

Tentacle::const_iterator Tentacle::begin() const{
	return this->points.begin();
}

Tentacle::const_iterator Tentacle::end() const{
	return this->points.end();
}


//***************************** SPEED SET *********************************//
/**
 * This constructor just makes an empty set of tentacles
 */
SpeedSet::SpeedSet(){
	this->seedRad = 0;
	this->velocity = 0;
}

SpeedSet::SpeedSet(const SpeedSet& SpeedSet):
					tentacles(SpeedSet.tentacles){
	this->seedRad  = SpeedSet.seedRad;
	this->velocity = SpeedSet.velocity;
}


SpeedSet::SpeedSet(double expFact, double seedRad, int numTent, double resolution, int xDim, int yDim, double velocity){
	this->seedRad  = seedRad;
	this->velocity = velocity;
	PRINTER("Generating a Speed Set with the Parameters <SRad=%f, Vel=%f, NumTent=%d, expF=%f>", seedRad, velocity, numTent, expFact);
	for(int t=0; t<numTent; t++){
		this->tentacles.push_back(Tentacle(expFact, seedRad, t, numTent, resolution, xDim, yDim, velocity));
	}
}

SpeedSet::~SpeedSet(){};

unsigned int SpeedSet::getNumTentacle() const{
	return this->tentacles.size();
}

double SpeedSet::getVelocity() const{
	return this->velocity;
}

double SpeedSet::getSeedRad() const{
	return this->seedRad;
}

const Tentacle& SpeedSet::getTentacle(int index)const throw(oryx_path_planning::TentacleAccessException){
	if(index<0||index>(int)getNumTentacle()) throw new oryx_path_planning::TentacleAccessException(index, 0);
	try{
		return this->tentacles.at(index);
	}catch (std::exception& e){
		std::string message("Something went wrong!");
		throw new oryx_path_planning::TentacleAccessException(index, 0, message, e);
	}
}

SpeedSet::iterator SpeedSet::begin(){
	return this->tentacles.begin();
}

SpeedSet::iterator SpeedSet::end(){
	return this->tentacles.end();
}

SpeedSet::const_iterator SpeedSet::begin() const{
	return this->tentacles.begin();
}

SpeedSet::const_iterator SpeedSet::end() const{
	return this->tentacles.end();
}


//***************************** TENTACLE GENERATOR *********************************//
TentacleGenerator::TentacleGenerator(){
	this->numTentacles	= 0;
	this->numSpeedSet	= 0;
	this->expFact	= 0;
}

TentacleGenerator::TentacleGenerator(TentacleGenerator& TentacleGenerator):
		speedSets(TentacleGenerator.speedSets),
		velocityKeys(TentacleGenerator.velocityKeys){
	this->numSpeedSet = TentacleGenerator.numSpeedSet;
	this->numTentacles= TentacleGenerator.numTentacles;
	this->expFact     = TentacleGenerator.expFact;
}

TentacleGenerator::TentacleGenerator(const TentacleGenerator& TentacleGenerator):
		speedSets(TentacleGenerator.speedSets),
		velocityKeys(TentacleGenerator.velocityKeys){
	this->numSpeedSet = TentacleGenerator.numSpeedSet;
	this->numTentacles= TentacleGenerator.numTentacles;
	this->expFact     = TentacleGenerator.expFact;
}

TentacleGenerator::TentacleGenerator(double minSpeed, double maxSpeed, int numSpeedSet, int numTentacles, double expFact, double resolution, int xDim, int yDim){
	this->expFact 		= expFact;
	this->numTentacles	= numTentacles;
	this->numSpeedSet	= numSpeedSet;
	PRINTER("Generating Speed Sets...");

	double q = 0;
	double vel = 0;
	//Generate the SpeedSets
	for(int v=0; v<numSpeedSet; v++){
		q = calcQ(v);
		vel = calcSpeedSetVel(minSpeed, maxSpeed, q);
		PRINTER("Calculated q=%f",q);
		this->speedSets.push_back(SpeedSetPtr(new SpeedSet(expFact, calcSeedRad(v, q), numTentacles, resolution, xDim, yDim, vel)));
		this->velocityKeys.push_back(vel);
	}
	PRINTER("Speed Sets Complete!");
}

TentacleGenerator::~TentacleGenerator(){};

int TentacleGenerator::getNumSpeedSets() const{
	return this->speedSets.size();
}

const Tentacle& TentacleGenerator::getTentacle(int speedSet, int index) const throw (oryx_path_planning::TentacleAccessException, oryx_path_planning::SpeedSetAccessException){
	if(speedSet<0||speedSet>(int)this->speedSets.size()) throw new oryx_path_planning::SpeedSetAccessException(speedSet);
	if(index<0||index>(int)this->speedSets.at(speedSet)->getNumTentacle()) throw new oryx_path_planning::TentacleAccessException(index, speedSet);
	try{
		return this->speedSets.at(speedSet)->getTentacle(index);
	}catch(std::exception& e){
		std::string message("Something Went Wrong!");
		throw new oryx_path_planning::TentacleAccessException(index, speedSet, message, e);
	}
}

const SpeedSet& TentacleGenerator::getSpeedSet(int speedSet) const{
	return *this->speedSets.at(speedSet);
}

/**
 * Finds the speed set based on a nearest neighbor search
 */
const SpeedSet& TentacleGenerator::getSpeedSet(double velocity) const{
	double lowSpeed		= 0;
	double highSpeed	= 0;
	int index			= 0;
	//Iterate across keys looking for closest neighbors
	for(std::vector<double>::const_iterator itr = this->velocityKeys.begin(); itr<this->velocityKeys.end(); itr++){
		if(velocity>=*itr){
			lowSpeed = *itr;
			index++;
		}else{
			highSpeed = *itr;
			break;
		}
	}
	//If the key was between two values, calculate the closest neighbor
	if(highSpeed!=0){
		//If closer to low speed, select that index
		if((velocity-lowSpeed)>(highSpeed-velocity)){
			return *this->speedSets.at(index);
		}else{
			return *this->speedSets.at(index+1);
		}
	}else return *this->speedSets.at(index);
}

TentacleGenerator::iterator TentacleGenerator::begin(){
	return this->speedSets.begin();
}

TentacleGenerator::iterator TentacleGenerator::end(){
	return this->speedSets.end();
}

TentacleGenerator::const_iterator TentacleGenerator::begin() const{
	return this->speedSets.begin();
}

TentacleGenerator::const_iterator TentacleGenerator::end() const{
	return this->speedSets.end();
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
double TentacleGenerator::calcSeedRad(int speedSet, double q) const{
	double dphi = TENTACLE_SWEEP_ANGLE;
	double l	= MIN_TENTACLE_LENGTH+EXP_TENTACLE_LENGTH_BASE*std::pow(q, EXP_TENTACLE_LENGTH_FACTOR);
	PRINTER("Calculated dphi=%f, l=%f",dphi, l);
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
double TentacleGenerator::calcSpeedSetVel(double minSpeed, double maxSpeed, double q) const{
	return minSpeed+std::pow(q, 1.2)*(maxSpeed-minSpeed);
}

/**
 * Calculates 'magic constant' q. Formula taken from 'von Hundelshausen et al.: Integral Structures for Sensing and Motion'
 */
double TentacleGenerator::calcQ(int speedSet) const{
	return (double)speedSet/((double)this->numSpeedSet-1);
}

};

//************************************************ UNDEFS ***************************************//
#undef MIN_TENTACLE_LENGTH
#undef EXP_TENTACLE_LENGTH_BASE
#undef EXP_TENTACLE_LENGTH_FACTOR
#undef TENTACLE_SWEEP_ANGLE
#undef THETA_INCREMENT
