#ifndef HD_MOTOR_CONTROLLER_H_
#define HD_MOTOR_CONTROLLER_H_

#include <stdlib.h>
#include <string>
#include <exception>
#include <stdexcept>
#include <stdint.h>
#include "serial_driver_base/serial_port.h"

namespace hd_driver{

/**
 * @author Mitchell Wills
 * @brief An object which represents a Harmonic Drive Motor Controller
 * After creation call open to open the connection to the motor controller.
 * Make sure to call close when you are done with the device.
 */
class HDMotorController{
 private:
  typedef int32_t memory_value_t;
  typedef int32_t register_value_t;
  typedef uint16_t register_t;
  typedef enum{
    memory_bank_flash = 'f',
    memory_bank_ram = 'r',
  } memory_bank_t;
  typedef enum{
    variable_amplifier_state = 0x24,

    //status registers
    variable_status_register = 0xa0,
    variable_trajectory_register = 0xc9,
    variable_fault_register = 0xa4,
    
    //programed position mode variables
    variable_position_profile_type = 0xc8,
    variable_position_command = 0xca,
    variable_position_maximum_velocity = 0xcb,
    variable_position_maximum_acceleration = 0xcc,
    variable_position_maximum_deceleration = 0xcd,
    variable_position_maximum_jerk = 0xce,
    variable_position_abort_decleration = 0xcf,

    //misc vars
    variable_input_voltage = 0x1d,
    variable_baud_rate = 0x90,


    //position loop run time variables
    variable_motor_position = 0x32,
    variable_load_position = 0x17,
    variable_following_error = 0x35,

    //position loop inputs from trajectory generator
    variable_commanded_position = 0x3d,
    variable_limited_position = 0x2d,
    variable_profile_velocity = 0x3B,
    variable_profile_acceleration = 0x3C,

  } variable_t;

//Status register bits
#define STATUS_SHORT_DETECTED (1<<0)
#define STATUS_AMPLIFIER_OVER_TEMP (1<<1)
#define STATUS_OVER_VOLTAGE (1<<2)
#define STATUS_UNDER_VOLTAGE (1<<3)
#define STATUS_MOTOR_TEMP_ACTIVE (1<<4)
#define STATUS_FEEDBACK_ERROR (1<<5)
#define STATUS_MOTOR_PHASING_ERROR (1<<6)
#define STATUS_CURRENT_OUTPUT_LIMITED (1<<7)
#define STATUS_VOLTAGE_OUTPUT_LIMITED (1<<8)
#define STATUS_POSITIVE_LIMIT_ACTIVE (1<<9)
#define STATUS_NEGATIVE_LIMIT_ACTIVE (1<<10)
#define STATUS_ENABLE_INPUT_NOT_ACTIVE (1<<11)
#define STATUS_SOFT_DISABLE (1<<12)
#define STATUS_ATEMPT_STOP (1<<13)
#define STATUS_BRAKE_ACTIVE (1<<14)
#define STATUS_PWM_OUT_DISABLED (1<<15)
#define STATUS_POSITIVE_SOFT_LIMIT (1<<16)
#define STATUS_NEGATIVE_SOFT_LIMIT (1<<17)
#define STATUS_TRACKING_ERROR (1<<18)
#define STATUS_TRACKING_WARNING (1<<19)
#define STATUS_AMPLIFIER_RESET (1<<20)
#define STATUS_POSITION_WRAPPED (1<<21)
#define STATUS_AMPLIFIER_FAULT (1<<22)
#define STATUS_VELOCITY_LIMIT_REACHED (1<<23)
#define STATUS_ACCELERATION_LIMIT_REACHED (1<<24)
#define STATUS_POSITION_OUTSIDE_TRACKING_WINDOW (1<<25)
#define STATUS_HOME_SWITCH_ACTIVE (1<<26)
#define STATUS_TRAJECTORY_RUNNING (1<<27)
#define STATUS_VELOCITY_WINDOW_MAX (1<<28)
#define STATUS_PHASE_NOT_INITIALIZED (1<<29)
#define STATUS_COMMAND_FAULT (1<<30)

//Trajectory Register bits
#define TRAJECTORY_CAM_TAVLE_UNDERFLOW (1<<9)
#define TRAJECTORY_HOMING_ERROR (1<<11)
#define TRAJECTORY_HOME_COMPLETE (1<<12)
#define TRAJECTORY_HOMING (1<<13)
#define TRAJECTORY_MOVE_ABORTED (1<<14)
#define TRAJECTORY_IN_MOTION (1<<15)

//Fault Register bits
#define FAULT_DATA_FLASH_CRC (1<<0)
#define FAULT_INTERNAL_AMPLIFIER (1<<1)
#define FAULT_SHORT_CIRCUT (1<<2)
#define FAULT_AMPLIFIER_OVER_TEMP (1<<3)
#define FAULT_MOTOR_OVER_TEMP (1<<4)
#define FAULT_OVER_VOLTAGE (1<<5)
#define FAULT_UNDER_VOLTAGE (1<<6)
#define FAULT_FEEDBACK_FAULT (1<<7)
#define FAULT_PHASING_ERROR (1<<8)
#define FAULT_FOLLOWING_ERROR (1<<9)
#define FAULT_OVER_CURRENT (1<<10)
#define FAULT_FPGA_FAILURE (1<<11)
#define FAULT_COMMAND_INPUT_LOST (1<<12)

  typedef enum{
    position_profile_absolute_trapazoidal = 0,
    position_profile_absolute_scurve = 1,
    position_profile_relative_trapazoidal = 256,
    position_profile_relative_scurve = 257,
    position_profile_valocity = 2
  } position_profile_type_t;
  typedef enum{
    trajectory_abort_move = 0,
    trajectory_update_move = 1,
    trajectory_home = 2,
  } trajectory_command_t;
  typedef enum{
    amplifier_disabled = 0,
    amplifier_current_programmed = 1,
    amplifier_current_analog = 2,
    amplifier_current_PWM = 3,
    amplifier_current_internal_func = 4,
    amplifier_current_uv = 5,
    amplifier_velocity_programmed = 11,
    amplifier_velocity_analog = 12,
    amplifier_velocity_PWM = 13,
    amplifier_velocity_internal_func = 14,
    amplifier_servo_trajectory_gen = 21,
    amplifier_servo_analog = 22,
    amplifier_servo_digital = 23,
    amplifier_servo_internal_func = 24,
    amplifier_servo_camming = 25,
    amplifier_servo_CANopen = 30,
  } amplifier_state_t;
 public:
  HDMotorController();
  ~HDMotorController();
  /**
   * @author Mitchell Wills
   * @brief Opens the connection to the motor controller
   * @param [in] port the port to connect to the device with (ex. /dev/ttyUSB0)
   */
  void open(std::string port);
  /**
   * @author Mitchell Wills
   * Close the connection to the motor controller
   */
  void close();
  /**
   * @author Mitchell Wills
   * @return true if the connection to the device is open
   */
  bool is_open(){return serial_port_.is_open();};

 public:
  /**
   * @brief set the state of the amplifier
   * @param state the new state of the amplifier
   */
  void set_state(amplifier_state_t state);
  void set_position(int32_t position);
  int get_position();
  uint32_t get_status();
  uint32_t get_trajectory_status();


 private:
  /**
   * @brief perform a set in either flash or RAM
   * @param memory_bank where to set the value (flash or RAM)
   * @param variable_id the actual variable to be set
   * @param value the new value for the variable
   */
  void set(memory_bank_t memory_bank, variable_t variable_id, memory_value_t value);
  /**
   * @brief get the value of a variable in memory
   * @param meory_bank where to get the value from (flash or RAM)
   * @param variable_id the id of the variable to get
   * @return the value of variable
   */
  memory_value_t get(memory_bank_t memory_bank, variable_t variable_id);
  /**
   * @brief copy a value from flash to RAM or RAM to flash
   * @param source_memory_bank the bank of memory to copy from (the value will be copied into the other)
   * @param variable_id the variable to copy
   */
  void copy(memory_bank_t source_memory_bank, variable_t variable_id);
  /**
   * @breif resets the amplifier
   */
  void reset();
  /**
   * @brief issue a trajectory command
   * @param command the command to issue
   */
  void trajectory(trajectory_command_t command);
  /**
   * @brief set a value in a register
   * @param register_id the register to set
   * @param value the value to put in the register
   */
  void set_register(register_t register_id, register_value_t value);
  /**
   * @brief get a value from a register
   * @param register_id the register to get from
   * @return the value that is in the register
   */
  register_value_t get_register(register_t register_id);

 private:
  serial_driver::DriverSerialPort serial_port_;
};

}

#endif //HD_MOTOR_CONTROLLER_H_
