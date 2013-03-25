#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <string>

typedef uint8_t char_size_t;
typedef enum {
  serial_parity_odd,
  serial_parity_even,
  serial_parity_none
} serial_parity_t;

class DriverSerialPort{
 public:
  DriverSerialPort():fd_(-1){}

  int open(std::string& port, speed_t baudrate, char_size_t char_size, serial_parity_t parity);
  void close();
  bool is_open(){return fd_!=-1;}
  
  int _select_end(fd_set* read_fds, fd_set* write_fds, fd_set* error_fds, struct timeval* end_time);
  int select_read(struct timeval* end_time);
  int select_write(struct timeval* end_time);

  int _read(void* buf, size_t size);
  int read(void* buf, size_t size, unsigned int timeout);
  int read_line(char* buf, size_t size, char end_char, unsigned int timeout);

  int _write(const void* buf, size_t size);
  int write(const void* buf, size_t size, unsigned int timeout);
  int write(const char* c_str, unsigned int timeout);
  int write(const std::string str, unsigned int timeout);

 private:
  int fd_;

};

#endif//SERIAL_PORT_H_

