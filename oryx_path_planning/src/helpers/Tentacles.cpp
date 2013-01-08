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
#define MIN_TENTACLE_LENGTH 0.8
#else
#error MIN_TENTACLE_LENGTH is already defined!
#endif

#ifndef EXP_TENTACLE_LENGTH_BASE
#define EXP_TENTACLE_LENGTH_BASE 3.5
#else
#error EXP_TENTACLE_LENGTH_BASE is already defined!
#endif

#ifndef EXP_TENTACLE_LENGTH_FACTOR
#define EXP_TENTACLE_LENGTH_FACTOR 1.2
#else
#error EXP_TENTACLE_LENGTH_FACTOR is already defined!
#endif

#ifndef THETA_INCREMENT
#define THETA_INCREMENT PI/1800.0
#else
#error THETA_INCREMENT is already defined!
#endif



typedef oryx_path_planning::Tentacle::TentacleTraverser TentTrav;	///Namespace declaration to make implementation of TentacleTraverser easier

//************************************************ IMPLEMENTATION ***************************************//
namespace oryx_path_planning
{

//******************** TENTACLE::TENTACLETRAVERSER ***********************//
/**
 * Sets both lastPoint and nextPoint to points.at(0), unless the points vector is empty in which case empty is set to true
 */
TentTrav::TentacleTraverser(Tentacle& tentacle)
{
	//If we have a normal set of points, initialize as normal
	if(tentacle.points_.size() > 1)
	{
		this->start_		= tentacle.points_.begin();
		this->end_		= tentacle.points_.end();
		this->last_point_	= &tentacle.points_.at(0);
		this->next_point_	= &tentacle.points_.at(0);
		this->length_	= 0;
		this->delta_length_ = 0;
		this->empty_		= false;
	}
	//If we only got a single point, still initialize the point values, but set empty true
	else if(tentacle.points_.size() == 1){
		this->start_		= tentacle.points_.begin();
		this->end_		= tentacle.points_.end();
		this->last_point_ = &tentacle.points_.at(0);
		this->next_point_ = &tentacle.points_.at(0);
		this->empty_ 	= true;
	}
	//If we got a completely empty vector, set empty true, do not initialize point values
	else {
		this->empty_ = true;
		this->next_point_ = NULL;
		this->last_point_ = NULL;
	}
}

/**
 * Sets both lastPoint and nextPoint to points.at(0), unless the points vector is empty in which case empty is set to true
 */
TentTrav::TentacleTraverser(const Tentacle& tentacle)
{
	//If we have a normal set of points, initialize as normal
	if(tentacle.points_.size() > 1){
		this->start_		= tentacle.points_.begin();
		this->end_		= tentacle.points_.end();
		this->last_point_	= &tentacle.points_.at(0);
		this->next_point_	= &tentacle.points_.at(0);
		this->length_	= 0;
		this->delta_length_ = 0;
		this->empty_		= false;
	}
	//If we only got a single point, still initialize the point values, but set empty true
	else if(tentacle.points_.size() == 1){
		this->start_		= tentacle.points_.begin();
		this->end_		= tentacle.points_.end();
		this->last_point_ = &tentacle.points_.at(0);
		this->next_point_ = &tentacle.points_.at(0);
		this->empty_ 	= true;
	}
	//If we got a completely empty vector, set empty true, do not initialize point values
	else {
		this->empty_ = true;
		this->next_point_ = NULL;
		this->last_point_ = NULL;
	}
}

TentTrav::~TentacleTraverser()
{
};

bool TentTrav::hasNext() const
{
	return !this->empty_;
}

/**
 * Calling this method also updates the traversed length.
 * The traversed length is calculated using linear approximation:
 * @f[ l_t = \sum\limits_{n=1}^k d(p_(n-1), p_n) @f]
 * Where @f$ d(p_(k-1), p_k) @f$ is the linear distance between points @f$ p_(n-1) \text{ and } p_n @f$ .
 *
 */
const oryx_path_planning::Point& TentTrav::next()
{
	//If they're not equal, and we're not empty, we're already in the traversal, process the next point
	if((this->next_point_ != this->last_point_)&&!this->empty_)
	{
		//Update the calculated length along the Tentacle we've traversed
		Eigen::Vector4f lastPM(this->last_point_->getVector4fMap());
		Eigen::Vector4f nextPM(this->next_point_->getVector4fMap());
		this->delta_length_ = pcl::distances::l2(lastPM, nextPM);
		this->length_ += this->delta_length_;

		//Set lastPoint to nextPoint so that nextPoint can be updated if possible
		this->last_point_ = this->next_point_;

		//Increment the traversal and check for end
		this->start_++;
		if(this->start_>= this->end_)
		{
			this->empty_ = true;
		}
		//Otherwise update nextPoint
		else
		{
			this->next_point_ = &*this->start_;
		}

		//Return lastPoint, which was nextPoint when this method was originally called unless the traversal was empty
		return *this->last_point_;
	}
	//If we're empty, just return lastPoint which will be the final point in the traversal
	else if(empty_)
	{
		return *this->last_point_;
	}
	//If they're equal and we're not empty, we haven't started the traversal, so start it
	else
	{
		if(this->start_ != this->end_)
		{
			this->start_++;
			this->next_point_ = &*this->start_;
		}
		else this->empty_ = true;
		return *this->last_point_;
	}
}


double TentTrav::lengthTraversed() const
{
	return this->length_;
}

double TentTrav::deltaLength() const
{
	return this->delta_length_;
}

//***************************** TENTACLE *********************************//
/**
 * This constructor just makes an empty tentacle
 */
Tentacle::Tentacle()
{
	this->radius_ = 0;
	this->velocity_ = 0;
}

Tentacle::Tentacle(const Tentacle& Tentacle):
						points_(Tentacle.points_)
{
	this->radius_ = Tentacle.radius_;
	this->velocity_ = Tentacle.velocity_;
}

Tentacle::~Tentacle(){};

Tentacle::Tentacle(double expFact, double seedRad, double seedLength, int index, int numTent, double resolution, int xDim, int yDim, double velocity) throw (TentacleGenerationException):
							points_()
{

	this->velocity_= velocity;
	PRINTER("Generating Tentacle %d", index);
	int halfway_index	= numTent/2;
	//Calculate tentacle radius. Formula taken from 'von Hundelshausen et al.: Integral Structures for Sensing and Motion'
	if(index<halfway_index)
	{
		//Tentacle is to the right of halfway
		this->radius_ = std::pow(expFact,index)*seedRad;
	}
	else if(index>halfway_index)
	{
		//Tentacle is to the left of halfway
		this->radius_ = -std::pow(expFact,index-(halfway_index+1))*seedRad;
	}
	else
	{
		//Tentacle is exactly at halfway
		this->radius_ = std::numeric_limits<double>::infinity();
	}
	PRINTER("Calculated Tentacle Radius=%f", this->radius_);
	//Calculate the working length of the tentacle
	int working_length;

	if(index<halfway_index)
	{
		//ROS_INFO("Small Index %d, raw working length is %f",index, seedLength+2*std::sqrt((double)index/(double)halfwayIndex));
		working_length = roundToGrid(seedLength+2*std::sqrt((double)index/(double)halfway_index), resolution);
	}
	else{
		//ROS_INFO("Large Index %d, raw working length is %f",index, seedLength+2*std::sqrt(((double)index-(double)halfwayIndex)/(double)halfwayIndex));
		working_length = roundToGrid(seedLength+2*std::sqrt(((double)index-(double)halfway_index)/(double)halfway_index), resolution);
	}

	//Check for special case of an effectively straight line
	if(this->radius_ > this->straight_threshold_ || this->radius_ < -this->straight_threshold_)
	{
		this->radius_ = std::numeric_limits<double>::infinity();
		for(double i = 0; i<working_length; i++)
		{
			oryx_path_planning::Point coord;
			coord.x=i;
			coord.y=0;
			coord.z=0;
			this->points_.push_back(coord);
		}
	}
	else
	{
		//Convert the radius, which will be in engineering units, into grid coordinates
		int working_radius = roundToGrid(radius_, resolution);
		//Calculate the sweep angle for this tentacle
		double sweep_angle = std::abs((double)working_length/(double)working_radius);
		//ROS_INFO("Tentacle %d, Working Length %d, Working Radius %d, Sweep Angle %f", index, working_length, working_radius, sweep_angle);
		//		//Place the origin of the tentacle, which will be at x=0 and y=-workingRadius
		//		Point tentacleOrigin;
		//		tentacleOrigin.x = 0;
		//		tentacleOrigin.y = -workingRadius;
		//		tentacleOrigin.z = 0;
		//		tentacleOrigin.rgba = oryx_path_planning::TENTACLE;
		//
		//		workingRadius = std::abs(workingRadius);
		//		if(this->radius>0){
		//			castArc(workingRadius, sweepAngle, oryx_path_planning::TENTACLE, tentacleOrigin, this->points);
		//		}
		//		else{
		//			castArc(workingRadius, sweepAngle, oryx_path_planning::TENTACLE, tentacleOrigin, this->points, 3);
		//		}
		//	}
		//	else{
		//Tracks the last coordinate so that we don't get duplicates
		oryx_path_planning::Point last_coord;
		last_coord.x = 0;
		last_coord.y = 0;
		last_coord.z = 0;
		//The amount to increment theta by
		double theta_increment	= oryx_path_planning::constants::PI()/((100.0/resolution)*std::abs(this->radius_));
		//Push the first coordinate on
		this->points_.push_back(last_coord);
		//Calculate the X and Y coord along the tentacle
		if(this->radius_>0)
		{
			double start_angle = oryx_path_planning::constants::PI()/2.0;
			for(double t=start_angle; t>(start_angle-sweep_angle); t-=theta_increment)
			{
				oryx_path_planning::Point new_coord;
				new_coord.y = (oryx_path_planning::roundToGrid(this->radius_*std::sin(t)-this->radius_, resolution));
				new_coord.x = (oryx_path_planning::roundToGrid(this->radius_*std::cos(t), resolution));
				new_coord.z = 0;
				//If we've hit the top of the occupancy grid, break
				if(new_coord.x>xDim||std::abs(new_coord.y)>yDim) break;
				//Otherwise push_back the next point if it's not the same as the previous point
				if(!((((Eigen::Vector4f)new_coord.getVector4fMap())==(((Eigen::Vector4f)last_coord.getVector4fMap())))||(new_coord.x<0)/*||(std::abs(newCoord.getY()>yDim))*/))
				{
					this->points_.push_back(new_coord);
					last_coord = new_coord;
				}
			}
		}
		else
		{
			double start_angle = oryx_path_planning::constants::PI()/2.0;
			for(double t=start_angle; t<(start_angle+sweep_angle); t+=theta_increment){
				oryx_path_planning::Point new_coord;
				new_coord.y = (float)(oryx_path_planning::roundToGrid(this->radius_*std::sin(t)-radius_, resolution));
				new_coord.x = (float)(oryx_path_planning::roundToGrid(this->radius_*std::cos(t), resolution));
				new_coord.z = 0;
				//If we've hit the top of the occupancy grid, break
				if(new_coord.x>xDim||std::abs(new_coord.y)>yDim) break;
				//Otherwise push_back the next point if it's not the same as the previous point
				if(!((((Eigen::Vector4f)new_coord.getVector4fMap())==(((Eigen::Vector4f)last_coord.getVector4fMap())))||(new_coord.x<0)/*||(std::abs(newCoord.getY()>yDim))*/)){
					this->points_.push_back(new_coord);
					last_coord = new_coord;
				}
			}
		}
	}
	PRINTER("Calculated a Tentacle with Number of Points=%d",(int)this->points_.size());
}

double Tentacle::getRad() const
{
	return this->radius_;
}

double Tentacle::getVel() const
{
	return this->velocity_;
}

const Tentacle::TentaclePointCloud& Tentacle::getPoints() const
{
	return this->points_;
}

/**
 * Calculated via the following formula:
 * @f[ y = floor(\frac{radius \times \cosine(theta)}{scale}+rshift) @f]
 * @f[ x = floor(\frac{radius \times \sine(theta)}{scale}) @f]
 */
/*void Tentacle::calcCoord(double radius, double theta, double rshift, double scale, tf::Point& result){

}*/

Tentacle::iterator Tentacle::begin()
{
	return this->points_.begin();
}

Tentacle::iterator Tentacle::end()
{
	return this->points_.end();
}

Tentacle::const_iterator Tentacle::cbegin() const
{
	return this->points_.begin();
}

Tentacle::const_iterator Tentacle::cend() const
{
	return this->points_.end();
}


//***************************** SPEED SET *********************************//
/**
 * This constructor just makes an empty set of tentacles
 */
SpeedSet::SpeedSet()
{
	this->seed_rad_ = 0;
	this->velocity_ = 0;
}

SpeedSet::SpeedSet(const SpeedSet& SpeedSet):
							tentacles_(SpeedSet.tentacles_)
{
	this->seed_rad_  = SpeedSet.seed_rad_;
	this->velocity_ = SpeedSet.velocity_;
}


SpeedSet::SpeedSet(double expFact, double seedRad, double seedLength, int numTent, double resolution, int xDim, int yDim, double velocity)
{
	this->seed_rad_  = seedRad;
	this->velocity_ = velocity;
	PRINTER("Generating a Speed Set with the Parameters <SRad=%f, Vel=%f, NumTent=%d, expF=%f>", seedRad, velocity, numTent, expFact);
	for(int t=0; t<numTent; t++)
	{
		this->tentacles_.push_back(Tentacle(expFact, seedRad, seedLength, t, numTent, resolution, xDim, yDim, velocity));
	}
}

SpeedSet::~SpeedSet(){};

unsigned int SpeedSet::getNumTentacle() const
{
	return this->tentacles_.size();
}

double SpeedSet::getVelocity() const
{
	return this->velocity_;
}

double SpeedSet::getSeedRad() const
{
	return this->seed_rad_;
}

const Tentacle& SpeedSet::getTentacle(int index)const throw(oryx_path_planning::TentacleAccessException)
				{
	if(index<0||index>(int)getNumTentacle()) throw new oryx_path_planning::TentacleAccessException(index, 0);
	try
	{
		return this->tentacles_.at(index);
	}
	catch (std::exception& e)
	{
		std::string message("Something went wrong!");
		throw new oryx_path_planning::TentacleAccessException(index, 0, message, e);
	}
				}

SpeedSet::iterator SpeedSet::begin()
{
	return this->tentacles_.begin();
}

SpeedSet::iterator SpeedSet::end()
{
	return this->tentacles_.end();
}

SpeedSet::const_iterator SpeedSet::cbegin() const
{
	return this->tentacles_.begin();
}

SpeedSet::const_iterator SpeedSet::cend() const
{
	return this->tentacles_.end();
}


//***************************** TENTACLE GENERATOR *********************************//
TentacleGenerator::TentacleGenerator()
{
	this->num_tentacles_	= 0;
	this->num_speed_set_	= 0;
	this->exp_fact_	= 0;
}

TentacleGenerator::TentacleGenerator(TentacleGenerator& TentacleGenerator):
				speed_sets_(TentacleGenerator.speed_sets_),
				velocity_keys_(TentacleGenerator.velocity_keys_)
{
	this->num_speed_set_ = TentacleGenerator.num_speed_set_;
	this->num_tentacles_= TentacleGenerator.num_tentacles_;
	this->exp_fact_     = TentacleGenerator.exp_fact_;
}

TentacleGenerator::TentacleGenerator(const TentacleGenerator& TentacleGenerator):
				speed_sets_(TentacleGenerator.speed_sets_),
				velocity_keys_(TentacleGenerator.velocity_keys_)
{
	this->num_speed_set_ = TentacleGenerator.num_speed_set_;
	this->num_tentacles_= TentacleGenerator.num_tentacles_;
	this->exp_fact_     = TentacleGenerator.exp_fact_;
}

TentacleGenerator::TentacleGenerator(double minSpeed, double maxSpeed, int numSpeedSet, int numTentacles, double expFact, double resolution, int xDim, int yDim)
{
	this->exp_fact_ 		= expFact;
	this->num_tentacles_	= numTentacles;
	this->num_speed_set_	= numSpeedSet;
	PRINTER("Generating Speed Sets...");

	double q = 0;
	double vel = 0;
	double l = 0;
	//Generate the SpeedSets
	for(int v=0; v<numSpeedSet; v++)
	{
		q = calcQ(v);
		l = calcL(q);
		vel = calcSpeedSetVel(minSpeed, maxSpeed, q);
		PRINTER("Calculated q=%f",q);
		this->speed_sets_.push_back(SpeedSetPtr(new SpeedSet(expFact, calcSeedRad(v, l, q), l, numTentacles, resolution, xDim, yDim, vel)));
		this->velocity_keys_.push_back(vel);
	}
	PRINTER("Speed Sets Complete!");
}

TentacleGenerator::~TentacleGenerator(){};

int TentacleGenerator::getNumSpeedSets() const
{
	return this->speed_sets_.size();
}

const Tentacle& TentacleGenerator::getTentacle(int speedSet, int index) const throw (oryx_path_planning::TentacleAccessException, oryx_path_planning::SpeedSetAccessException)
				{
	if(speedSet<0||speedSet>(int)this->speed_sets_.size()) throw new oryx_path_planning::SpeedSetAccessException(speedSet);
	if(index<0||index>(int)this->speed_sets_.at(speedSet)->getNumTentacle()) throw new oryx_path_planning::TentacleAccessException(index, speedSet);
	try
	{
		return this->speed_sets_.at(speedSet)->getTentacle(index);
	}
	catch(std::exception& e)
	{
		std::string message("Something Went Wrong!");
		throw new oryx_path_planning::TentacleAccessException(index, speedSet, message, e);
	}
				}

const SpeedSet& TentacleGenerator::getSpeedSet(int speedSet) const
{
	return *this->speed_sets_.at(speedSet);
}

/**
 * Finds the speed set based on a nearest neighbor search
 */
const SpeedSet& TentacleGenerator::getSpeedSet(double velocity) const
{
	double low_speed	= 0;
	double high_speed	= 0;
	int index			= 0;
	//Iterate across keys looking for closest neighbors
	for(std::vector<double>::const_iterator itr = this->velocity_keys_.begin(); itr<this->velocity_keys_.end(); itr++){
		if(velocity>=*itr)
		{
			low_speed = *itr;
			index++;
		}
		else
		{
			high_speed = *itr;
			break;
		}
	}
	//If the key was between two values, calculate the closest neighbor
	if(high_speed!=0)
	{
		//If closer to low speed, select that index
		if((velocity-low_speed)>(high_speed-velocity))
		{
			return *this->speed_sets_.at(index);
		}
		else
		{
			return *this->speed_sets_.at(index+1);
		}
	}
	else return *this->speed_sets_.at(index);
}

TentacleGenerator::iterator TentacleGenerator::begin()
{
	return this->speed_sets_.begin();
}

TentacleGenerator::iterator TentacleGenerator::end()
{
	return this->speed_sets_.end();
}

TentacleGenerator::const_iterator TentacleGenerator::cbegin() const
{
	return this->speed_sets_.begin();
}

TentacleGenerator::const_iterator TentacleGenerator::cend() const
{
	return this->speed_sets_.end();
}

double TentacleGenerator::calcL(double q)
{
	return  MIN_TENTACLE_LENGTH+EXP_TENTACLE_LENGTH_BASE*std::pow(q, EXP_TENTACLE_LENGTH_FACTOR);
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
double TentacleGenerator::calcSeedRad(int speedSet, double l, double q) const
{
	double dphi = 1.2*oryx_path_planning::constants::PI()/2.0;
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
double TentacleGenerator::calcSpeedSetVel(double minSpeed, double maxSpeed, double q) const
{
	return minSpeed+std::pow(q, 1.2)*(maxSpeed-minSpeed);
}

/**
 * Calculates 'magic constant' q. Formula taken from 'von Hundelshausen et al.: Integral Structures for Sensing and Motion'
 */
double TentacleGenerator::calcQ(int speedSet) const
{
	return (double)speedSet/((double)this->num_speed_set_-1);
}

};

//************************************************ UNDEFS ***************************************//
#undef MIN_TENTACLE_LENGTH
#undef EXP_TENTACLE_LENGTH_BASE
#undef EXP_TENTACLE_LENGTH_FACTOR
#undef TENTACLE_SWEEP_ANGLE
#undef THETA_INCREMENT
