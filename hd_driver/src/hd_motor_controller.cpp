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


void HDMotorController::set_state(amplifier_state_t state){
  set(memory_bank_ram, variable_amplifier_state, state);
}






/*
 * PRIVATE RAW FUNCTIONS
 */
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
  serial_port_.writef(10, "s %c0x%x %d\n", memory_bank, variable_id, value);
  char buf[20];
  serial_port_.read_until(buf, sizeof(buf), '\n', 10);
  if(streq("ok", buf))
    return;
  parse_ok_or_error("set", buf);
}

HDMotorController::memory_value_t HDMotorController::get(memory_bank_t memory_bank, variable_t variable_id){
  serial_port_.writef(10, "g %c0x%x\n", memory_bank, variable_id);
  char buf[20];
  serial_port_.read_until(buf, sizeof(buf), '\n', 10);

  parse_intval_or_error("get", buf);
}

void HDMotorController::copy(memory_bank_t source_memory_bank, variable_t variable_id){
  serial_port_.writef(10, "c %c0x%x\n", source_memory_bank, variable_id);
  char buf[20];
  serial_port_.read_until(buf, sizeof(buf), '\n', 10);
  parse_ok_or_error("copy", buf);
}


void HDMotorController::reset(){
  serial_port_.writef(10, "r\n");
}

void HDMotorController::trajectory(trajectory_command_t command){
  serial_port_.writef(10, "t %d\n", command);
  char buf[20];
  serial_port_.read_until(buf, sizeof(buf), '\n', 10);
  parse_ok_or_error("trajectory", buf);
}

void HDMotorController::set_register(register_t register_id, register_value_t value){
  serial_port_.writef(10, "i r%d %d\n", register_id, value);
  char buf[20];
  serial_port_.read_until(buf, sizeof(buf), '\n', 10);
  if(streq("ok", buf))
    return;
  parse_ok_or_error("set_register", buf);
}

HDMotorController::register_value_t HDMotorController::get_register(register_t register_id){
  serial_port_.writef(10, "i r%d\n", register_id);
  char buf[20];
  serial_port_.read_until(buf, sizeof(buf), '\n', 10);

  parse_intval_or_error("get_register", buf);
}


}
