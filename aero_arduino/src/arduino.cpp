#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#define LIGHT_ON LOW
#define LIGHT_OFF HIGH

const int LIGHT_PIN = 13;
const int PAUSE_PIN = 12;

unsigned long last_invert = 0;
float flash_rate = 0;

void rateCb(const std_msgs::Float32& status_rate){
  flash_rate = status_rate.data;
  //last_invert = 0;//make sure we make a change next time
}

std_msgs::Bool bool_msg;
ros::NodeHandle nh;
ros::Publisher pause_pub("/arduino_pause", &bool_msg);
ros::Subscriber<std_msgs::Float32> rate_sub("/arduino_rate", &rateCb);



void setup(){
  nh.initNode();
  nh.advertise(pause_pub);
  nh.subscribe(rate_sub);

  pinMode(PAUSE_PIN, INPUT);
  digitalWrite(PAUSE_PIN, HIGH);//enable pullup
  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, LIGHT_ON);
}

void loop(){
  unsigned long pulse_length = pulseIn(PAUSE_PIN, HIGH);
  bool_msg.data = pulse_length>1400;//will not pause if pulseIn timed out (no pulse = 0)
  pause_pub.publish( &bool_msg );

  //if(nh.connected()){
   if( !bool_msg.data)
{
      unsigned long time = millis();
      if(time-last_invert>=(1000/1/2)){
	digitalWrite(LIGHT_PIN, !digitalRead(LIGHT_PIN));
	last_invert = time;
    }
}

  //  }
    else
{
      digitalWrite(LIGHT_PIN, LIGHT_ON);
  }
 // else
 //   digitalWrite(LIGHT_PIN, LIGHT_ON);
  

  delay(50);

  nh.spinOnce();
}
