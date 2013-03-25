
#include "serial_driver_base/serial_port.h"
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <iostream>

int DriverSerialPort::open(std::string& port, speed_t baudrate, char_size_t char_size, serial_parity_t parity){
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if(fd_==-1)//TODO error
    return -1;

  struct termios options;
  tcgetattr(fd_, &options);

  cfsetospeed(&options, baudrate);
  cfsetispeed(&options, baudrate);

  //set to raw mode
  options.c_iflag = IGNBRK;
  options.c_lflag = 0;
  options.c_oflag = 0;
  options.c_cflag |= (CLOCAL | CREAD);

  //set character size
  options.c_cflag &= ~CSIZE;
  if(char_size==7)
    options.c_cflag |= CS7;
  else if(char_size==8)
    options.c_cflag |= CS8;
  else{
    close();
    //TODO error
    return -1;
  }

  if(parity==serial_parity_none){
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
  }
  else if(parity==serial_parity_odd){
    options.c_cflag |= PARENB;
    options.c_cflag |= PARODD;
    options.c_cflag &= ~CSTOPB;
  }
  else if(parity==serial_parity_even){
    options.c_cflag |= PARENB;
    options.c_cflag &= ~PARODD;
    options.c_cflag &= ~CSTOPB;
  }
  else{
    close();
    //TODO error
    return -1;
  }

  //apply settings
  tcsetattr(fd_, TCSANOW, &options);

  return 1;
}
void DriverSerialPort::close(){
  if(fd_!=-1)
    ::close(fd_);
  fd_ = -1;
}


static void get_time(struct timeval* t, unsigned int offset_ms){
  gettimeofday(t, NULL);
  int us = t->tv_usec + (offset_ms%1000)*1000;
  t->tv_usec = us%1000000;
  t->tv_sec = t->tv_sec + offset_ms/1000 + us/1000000;
}
static void get_time_till(struct timeval* target, struct timeval* result){
  struct timeval current;
  gettimeofday(&current, NULL);
  long long target_us = target->tv_usec + target->tv_sec*(long long)1000000;
  long long current_us = current.tv_usec + current.tv_sec*(long long)1000000;
  long long diff = target_us-current_us;
  if(diff>0){
    result->tv_usec = diff%1000000;
    result->tv_sec = diff/1000000;
  }
  else{
    result->tv_usec = 0;
    result->tv_sec = 0;
  }
}
int DriverSerialPort::_select_end(fd_set* read_fds, fd_set* write_fds, fd_set* error_fds, struct timeval* end_time){
  if(fd_==-1)
    return -1;
  struct timeval timeout;
  get_time_till(end_time, &timeout);
  return ::select(fd_ + 1, read_fds, write_fds, error_fds, &timeout);
}

int DriverSerialPort::_read(void* buf, size_t size){
  if(fd_==-1)
    return -1;
  return ::read(fd_, buf, size);
}

int DriverSerialPort::_write(const void* buf, size_t size){
  return ::write(fd_, buf, size);
}

int DriverSerialPort::select_read(struct timeval* end_time){
  fd_set fd_fds;
  FD_ZERO(&fd_fds);
  FD_SET(fd_, &fd_fds);
  return _select_end(&fd_fds, NULL, NULL, end_time);
}
int DriverSerialPort::select_write(struct timeval* end_time){
  fd_set fd_fds;
  FD_ZERO(&fd_fds);
  FD_SET(fd_, &fd_fds);
  return _select_end(NULL, &fd_fds, NULL, end_time);
}

int DriverSerialPort::read(void* buf, size_t size, unsigned int timeout){
  if(fd_==-1)
    return -1;
  struct timeval end_time;
  get_time(&end_time, timeout);
  uint8_t* cur_buf = (uint8_t*)buf;
  size_t total_read = 0;
  while(total_read<size){
    int num_read = _read(cur_buf, size-total_read);
    if(num_read<0 && errno!=EAGAIN)
      return num_read;
    else if(num_read>0){
      total_read += num_read;
      cur_buf += num_read;
    }

    if(total_read<size){//if not everything was read then call select
      if(select_read(&end_time)!=1)
	return -1;
    }
  }
  return total_read;
}

int DriverSerialPort::read_line(char* buf, size_t size, char end_char, unsigned int timeout){
  if(fd_==-1)
    return -1;
  struct timeval end_time;
  get_time(&end_time, timeout);
  char* cur_buf = (char*)buf;
  size_t total_read = 0;
  while(total_read<size-1){
    int num_read = _read(cur_buf, 1);
    if(num_read<0 && errno!=EAGAIN)
      return num_read;
    else if(num_read>0){
      if(*cur_buf == end_char){
	*cur_buf = '\0';
	return total_read;
      }
      total_read += num_read;
      cur_buf += num_read;
    }


    if(total_read<size-1){//if not everything was read then call select
      if(select_read(&end_time)!=1)
	return -1;
    }
  }
  //If we made it here then we reached the end of the buffer
  return -2;
}

int DriverSerialPort::write(const void* buf, size_t size, unsigned int timeout){
  if(fd_==-1)
    return -1;
  //TODO actually implement timeout
  return _write(buf, size);
}

int DriverSerialPort::write(const char* c_str, unsigned int timeout){
  return write(c_str, strlen(c_str), timeout);
}

int DriverSerialPort::write(const std::string str, unsigned int timeout){
  return write(str.c_str(), str.length(), timeout);
}
