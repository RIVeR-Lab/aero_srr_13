#ifndef __RoboteqDevice_H_
#define __RoboteqDevice_H_

#include "serial_driver_base/serial_port.h"

using namespace std;

class RoboteqDevice
{
private:
  serial_driver::DriverSerialPort serial_port;
protected:
	int Write(string str);
	int ReadAll(string &str);

	int IssueCommand(string commandType, string command, string args, int waitms, string &response, bool isplusminus = false);
	int IssueCommand(string commandType, string command, int waitms, string &response, bool isplusminus = false);

public:
	bool IsConnected();
	int Connect(string port);
	void Disconnect();

	int SetConfig(int configItem, int index, int value);
	int SetConfig(int configItem, int value);

	int SetCommand(int commandItem, int index, int value);
	int SetCommand(int commandItem, int value);
	int SetCommand(int commandItem);

	int GetConfig(int configItem, int index, int &result);
	int GetConfig(int configItem, int &result);

	int GetValue(int operatingItem, int index, int &result);
	int GetValue(int operatingItem, int &result);

	RoboteqDevice();
	~RoboteqDevice();
};

#endif
