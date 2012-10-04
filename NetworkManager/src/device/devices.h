/*
 * devices.h
 *
 *  Created on: Feb 14, 2012
 *      Author: mitchell
 */

#ifndef DEVICES_H_
#define DEVICES_H_

#include <nm-remote-settings.h>
#include <nm-client.h>

/**
 * Perform any necessary initialization that needs to occur prior to interacting with network devices
 * This will also enumerate and output all of the network devices on the system
 * @return true if successful
 */
gboolean initNetworkDevices(DBusGConnection* bus, NMClient* client, GMainLoop *eventLoop);


/**
 * try to get a network device to connect to a connection
 * @client an NMClient
 * @iface the iface of the network device
 * @connectionPath the connection path the device should connect to
 * @return true if successful
 */
gboolean connectNetworkDevice(NMClient* client, char* iface, char* connectionPath);
/**
 * attempt to get a network device to disconnect from the network it is connected t
 * @return true if successful
 */
gboolean disconnectNetworkDevice(NMClient* client, char* iface);

#endif /* DEVICES_H_ */
