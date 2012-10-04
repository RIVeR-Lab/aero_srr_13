/*
 * devices.c
 *
 *  Created on: Mar 5, 2012
 *      Author: mitchell
 */

#include "devices.h"
#include <nm-remote-settings.h>
#include <nm-client.h>
#include "../util/messages.h"
#include "../util/util.h"
#include <string.h>
#include "../connection/connections.h"

static const GPtrArray* devices;
gboolean initNetworkDevices(DBusGConnection* bus, NMClient* client, GMainLoop* eventLoop){
	debugMessage("init network devices");
	devices = nm_client_get_devices(client);
	updateDeviceList(devices);

	return TRUE;
}

NMDevice* getDevice(const char* iface){
	int i;
	for(i = 0; i<devices->len; ++i){
		NMDevice* device = (NMDevice*)g_ptr_array_index(devices, i);
		if(!strcmp(iface, nm_device_get_iface(device)))
			return device;
	}
	return NULL;
}


static void deviceConnect_cb(gpointer user_data, const char *object_path, GError *error){
	GMainLoop *loop = user_data;

	if(error)
		errorMessage("Error Connecting device Error: %d, %s", error->code, error->message);
	else
		infoMessage("Connected Device");

	g_main_loop_quit(loop);
}


gboolean isDeviceConnected(NMClient* client, char* iface){
	NMDevice* device = getDevice(iface);
	NMDeviceState state = nm_device_get_state(device);
	return state == NM_DEVICE_STATE_ACTIVATED;
}


gboolean connectNetworkDevice(NMClient* client, char* iface, char* connectionId){
	NMDevice* device = getDevice(iface);
	NMConnection* connection = getConnection(connectionId);
	if(device){
		if(connection){
			infoMessage("Connecting Device: %s to %s", iface, getConnectionId(connection));
			runInMainLoop(nm_client_activate_connection, client, NM_DBUS_SERVICE_SYSTEM_SETTINGS, nm_connection_get_path(connection), device, NULL, deviceConnect_cb, loop);
			return TRUE;
		}
		else{
			errorMessage("Could not find connection with id: %s", connectionId);
			return FALSE;
		}
	}
	else{
		errorMessage("Could not find device with iface: %s", iface);
		return FALSE;
	}
	return FALSE;
}




static void deviceDisconnect_cb(NMDevice *device, GError *error, gpointer user_data){
	GMainLoop *loop = user_data;

	if(error)
		errorMessage("Error disconnecting device: '%s' Error: %d, %s", nm_device_get_iface(device), error->code, error->message);
	else
		infoMessage("Disconnected Device: %s", nm_device_get_iface(device));

	g_main_loop_quit(loop);
}
gboolean disconnectNetworkDevice(NMClient* client, char* iface){
	NMDevice* device = getDevice(iface);
	if(device){
		infoMessage("Disconnecting Device: %s", iface);
		runInMainLoop(nm_device_disconnect, device, deviceDisconnect_cb, loop);
		return TRUE;
	}
	else{
		errorMessage("Could not find device with iface: %s", iface);
		return FALSE;
	}
}
