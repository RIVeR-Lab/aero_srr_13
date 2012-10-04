/*
 * CPUTempFinder.cpp
 *
 *  Created on: Feb 14, 2012
 *      Author: oryx
 */

#include <sensors/sensors.h>
#include <string>
#include <OryxMessages/Temperature.h>
#include <ros/ros.h>
ros::Publisher temperaturePublisher;
double dangerTemp,warningTemp;


using namespace std;

void getTemps(){
	sensors_chip_name const *cn;
    int c = 0;

    //This is the name of the subfeature we are looking for
	std::string tempName = "temp1_input";

	//This finds each of the cores
    while ((cn = sensors_get_detected_chips(0, &c)) != 0){
        sensors_feature const *feat;
        int f = 0;

        // Because libsensors doesn't know what each sensor is, features
        // stores the type (temp, voltage, fan, etc) of each sensor
        while ((feat = sensors_get_features(cn, &f)) != 0) {

            sensors_subfeature const *subf;
            int s = 0;

            //Subfeatures are the different calls available to each feature
            //These include temperature, max temperature, etc.
            while ((subf = sensors_get_all_subfeatures(cn, feat, &s)) != 0) {
               if(subf->name == tempName){
                double val;
                if (subf->flags & SENSORS_MODE_R) {

                	//Here is where the temperature is grabbed
                    int rc = sensors_get_value(cn, subf->number, &val);
                    if (rc < 0) {
                       ROS_WARN_STREAM("Invalid Sensor reading: " << rc);
                    } else {
                        OryxMessages::Temperature msg;
                        msg.temperature_node=c+10;
                        //Convert temperatures to fahrenheit
                        msg.warning_Temp=warningTemp;
                        msg.danger_Temp=dangerTemp;
                        msg.temperature=val*9/5+32;
                        temperaturePublisher.publish(msg);
                    }
                }
               }

            }
        }
    }
}


int main(int argc, char** argv){
	ros::init(argc, argv, "cpuTemps");
	ros::NodeHandle nh;
	double publishRate=1;
	dangerTemp = 80*9/5+32;
	warningTemp = 70*9/5+32;

	ros::param::get("~Warning_Temp",warningTemp);
	ros::param::get("~Danger_Temp",dangerTemp);
	ros::param::get("~Publish_Rate",publishRate);

	temperaturePublisher = nh.advertise<OryxMessages::Temperature>("Temps",10);
	ros::Rate loopRate(publishRate);

    sensors_cleanup();
    sensors_init(NULL);

    while(nh.ok()){

    	getTemps();
    	loopRate.sleep();
    	ros::spinOnce();
    }


}

