#include "hd_driver/hd_motor_controller.h"
#include <stdio.h>

using namespace serial_driver;

namespace hd_driver{

HDMotorController::HDMotorController(){
}

HDMotorController::~HDMotorController(){
  if(is_open())
    close();
}


void HDMotorController::open(std::string port){
  serial_port_.open(port, (speed_t)B9600, 8, serial_parity_none);
}

void HDMotorController::close(){
  serial_port_.close();
}


}
