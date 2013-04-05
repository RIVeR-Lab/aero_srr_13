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

 private:
  serial_driver::DriverSerialPort serial_port_;
};

}

#endif //HD_MOTOR_CONTROLLER_H_
