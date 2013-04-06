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
  //TODO increase baud rate
}

void HDMotorController::close(){
  serial_port_.close();
}


void HDMotorController::set_state(amplifier_state_t state){
  set(memory_bank_ram, variable_amplifier_state, state);
}

void HDMotorController::set_position(int32_t position){
  set(memory_bank_ram, variable_position_profile_type, position_profile_absolute_trapazoidal);
  set(memory_bank_ram, variable_position_command, position);
  set_state(amplifier_servo_trajectory_gen);
  trajectory(trajectory_update_move);
}
int HDMotorController::get_position(){
  return get(memory_bank_ram, variable_motor_position);
}

uint32_t HDMotorController::get_status(){
  return get(memory_bank_ram, variable_status_register);
}

uint32_t HDMotorController::get_trajectory_status(){
  return get(memory_bank_ram, variable_trajectory_register);
}



/*
 * PRIVATE RAW FUNCTIONS
 */
//Defined as macros so that exception macros give proper line numbers
#define parse_error(command, buf)			\
  int error_code;\
  if(sscanf(buf, "e %d", &error_code)==1)\
    SERIAL_DRIVER_EXCEPT(Exception, "Error doing "#command", got error code %d", error_code);\
  SERIAL_DRIVER_EXCEPT(CorruptDataException, "Error doing "#command", invalid response")

#define parse_ok_or_error(command, buf)		\
  if(streq("ok", buf))\
    return;\
  parse_error("set", buf)

#define parse_intval_or_error(command, buf)		\
  int value;\
  if(sscanf(buf, "v %d", &value)==1)\
    return value;\
  parse_error("set", buf)

void HDMotorController::set(memory_bank_t memory_bank, variable_t variable_id, memory_value_t value){
  serial_port_.writef(10, "s %c0x%x %d\r", memory_bank, variable_id, value);
  char buf[20];
  serial_port_.read_until(buf, sizeof(buf), '\r', 100);
  if(streq("ok", buf))
    return;
  parse_ok_or_error("set", buf);
}

HDMotorController::memory_value_t HDMotorController::get(memory_bank_t memory_bank, variable_t variable_id){
  serial_port_.writef(10, "g %c0x%x\r", memory_bank, variable_id);
  char buf[20];
  serial_port_.read_until(buf, sizeof(buf), '\r', 100);

  parse_intval_or_error("get", buf);
}

void HDMotorController::copy(memory_bank_t source_memory_bank, variable_t variable_id){
  serial_port_.writef(10, "c %c0x%x\r", source_memory_bank, variable_id);
  char buf[20];
  serial_port_.read_until(buf, sizeof(buf), '\r', 100);
  parse_ok_or_error("copy", buf);
}


void HDMotorController::reset(){
  serial_port_.writef(10, "r\r");
}

void HDMotorController::trajectory(trajectory_command_t command){
  serial_port_.writef(10, "t %d\r", command);
  char buf[20];
  serial_port_.read_until(buf, sizeof(buf), '\r', 100);
  parse_ok_or_error("trajectory", buf);
}

void HDMotorController::set_register(register_t register_id, register_value_t value){
  serial_port_.writef(10, "i r%d %d\r", register_id, value);
  char buf[20];
  serial_port_.read_until(buf, sizeof(buf), '\r', 100);
  if(streq("ok", buf))
    return;
  parse_ok_or_error("set_register", buf);
}

HDMotorController::register_value_t HDMotorController::get_register(register_t register_id){
  serial_port_.writef(10, "i r%d\r", register_id);
  char buf[20];
  serial_port_.read_until(buf, sizeof(buf), '\r', 100);

  parse_intval_or_error("get_register", buf);
}


}
