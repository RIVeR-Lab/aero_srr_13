#include "config.h"
#include "util/messages.h"
#include "device/devices.h"
#include "connection/connections.h"
#include "util/util.h"
#include <nm-client.h>
#include "connection/adhoc.h"
#include "connection/wired.h"
#include <pthread.h>
#include <nm-remote-settings.h>
#include <unistd.h>

void initialize(DBusGConnection *bus, NMClient *client, GMainLoop *eventLoop){
	debugMessage("Initializing");
	initNetworkDevices(bus, client, eventLoop);
	initNetworkConnections(bus, eventLoop);
}

void setupConnections(DBusGConnection *bus, NMClient *client){
	//create connections
	addAdhocConnection(bus, WIFI_ADHOC_NAME);
	addWiredConnection(bus, CAMERA_CONNECTION_NAME, CAMERA_CONNECTION_IP);
	sleep(1);
	connectNetworkDevice(client, WIFI_ADAPTER_IFACE, WIFI_ADHOC_NAME);
	connectNetworkDevice(client, CAMERA_ADAPTER_IFACE, CAMERA_CONNECTION_NAME);
}

void eventLoopThreadMain(GMainLoop *eventLoop){
	infoMessage("Starting Main Event Thread");
	g_main_loop_run(eventLoop);
	infoMessage("Main Event Thread Terminated");
}

void monitorNetworkInterfaces(DBusGConnection *bus, NMClient *client){
	debugMessage("Checking network interfaces");

	if(!isDeviceConnected(WIFI_ADAPTER_IFACE)){
		infoMessage("Wifi Not Connected, Attempting to connect");
		connectNetworkDevice(client, WIFI_ADAPTER_IFACE, WIFI_ADHOC_NAME);
	}

	//nm_client_get_active_connections
}

int main(int argc, char *argv[]) {
	g_type_init ();
	DBusGConnection* bus = dbus_g_bus_get (DBUS_BUS_SYSTEM, NULL);
	NMClient* client = nm_client_new();

	GMainLoop* eventLoop = g_main_loop_new(NULL, FALSE);
	pthread_t eventLoopThread;


	infoMessage("Starting Orxy Network Manager");
	infoMessage("Network Manager is %s", nm_client_get_manager_running(client)?"running":"not running");
	infoMessage("Network Manager state is '%s'", nmStateToStr(nm_client_get_state(client)));
	infoMessage("Networking is %s", nm_client_networking_get_enabled(client)?"enabled":"disabled");
	infoMessage("Networking is hardware %s", nm_client_wireless_hardware_get_enabled(client)?"enabled":"disabled");
	infoMessage("Wireless Networking is %s", nm_client_wireless_get_enabled(client)?"enabled":"disabled");
#ifdef DEBUG
	infoMessage("!!!DEBUG ENABLED!!!");
#endif

	initialize(bus, client, eventLoop);

	pthread_create(&eventLoopThread, NULL, (void * (*)(void *))eventLoopThreadMain, eventLoop);

	setupConnections(bus, client);

	while(1){
		sleep(MONITOR_SLEEP_TIME);
		monitorNetworkInterfaces(bus, client);
	}


	g_main_loop_quit(eventLoop);

	g_main_loop_unref (eventLoop);

	dbus_g_connection_unref (bus);
	pthread_exit(NULL);
	return 0;
}
