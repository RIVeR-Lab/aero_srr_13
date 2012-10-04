/*
 * connections.c
 *
 *  Created on: Feb 14, 2012
 *      Author: mitchell
 */

#include "connections.h"
#include "../util/messages.h"
#include "../util/util.h"
#include <string.h>
#include <stdlib.h>
#include <nm-remote-settings.h>
#include <nm-remote-settings-system.h>
#include <unistd.h>

static GSList *connections = NULL;
static NMSettingsInterface* settings;
static void updateConnections(){
	if(connections!=NULL)
		g_slist_free (connections);
	connections = nm_settings_interface_list_connections (settings);
	updateConnectionsList(connections);
}
static void newConnection_cb (NMRemoteSettings *nmremotesettings, NMConnection *connection, gpointer user_data){
	infoMessage("New Connection: %s", getConnectionId(connection));
	updateConnections();
}


GMainLoop *listConnectionsLoop = NULL;
static void listConnections_cb (NMRemoteSettings *settings, gpointer user_data){
	infoMessage("Listed Connections");
	debugMessage("Loop: ", listConnectionsLoop);
	g_main_loop_quit(listConnectionsLoop);
}
static gboolean listConnections (gpointer data){
	g_signal_connect (settings, NM_SETTINGS_INTERFACE_CONNECTIONS_READ,
	                  G_CALLBACK (listConnections_cb), listConnectionsLoop);
	return FALSE;
}
gboolean initNetworkConnections(DBusGConnection *bus, GMainLoop *eventLoop){
	if(settings==NULL){
		settings = NM_SETTINGS_INTERFACE(nm_remote_settings_system_new (bus));

		g_signal_connect (settings, NM_SETTINGS_INTERFACE_NEW_CONNECTION,
	                  G_CALLBACK (newConnection_cb), eventLoop);

		if(listConnectionsLoop==NULL){//only seems to be able to run once
			listConnectionsLoop = g_main_loop_new (NULL, FALSE);
			g_idle_add (listConnections, listConnectionsLoop);
			g_main_loop_run (listConnectionsLoop);
		}
	}

	return TRUE;
}
char* getConnectionPath(const char* id){
	int i;
	for(i = 0; i<g_slist_length(connections); ++i){
		NMConnection* connection = (NMConnection*)g_slist_nth_data(connections, i);
		if(!strcmp(id, getConnectionId(connection)))
			return strdup(nm_connection_get_path(connection));
	}
	return NULL;
}
NMConnection* getConnection(const char* id){
	int i;
	for(i = 0; i<g_slist_length(connections); ++i){
		NMConnection* connection = (NMConnection*)g_slist_nth_data(connections, i);
		if(!strcmp(id, getConnectionId(connection)))
			return connection;
	}
	return NULL;
}
const char* getConnectionId(NMConnection *connection){
	return nm_setting_connection_get_id(NM_SETTING_CONNECTION(nm_connection_get_setting_by_name(connection, NM_SETTING_CONNECTION_SETTING_NAME)));
}
struct addConnectionData{
	GMainLoop *loop;
	char* pathPointer;
};
static void addConnection_cb(NMSettingsInterface *settings, GError *error, gpointer user_data){
	GMainLoop *loop = user_data;

	if (error)
		errorMessage("Error adding connection: %s", error->message);
	else
		infoMessage("Added Connection");
	debugMessage("Loop %p", loop);
	g_main_loop_quit (loop);
}
gboolean addConnection(DBusGConnection *bus, NMConnection *connection){
	const char* connectionId = getConnectionId (NM_CONNECTION (connection));
	infoMessage("Adding Connection: %s", connectionId);
	NMConnection *oldConnection = getConnection(connectionId);
	if(oldConnection==NULL){
		runIfSuccessInMainLoop(nm_settings_interface_add_connection, settings, connection, addConnection_cb, loop);
		while(!getConnection(connectionId)){//TODO check if main loop ended for error
			sleep(1);
		}
		debugMessage("Detected Connection");
	}
	else{
		infoMessage("Connection: %s already existed", connectionId);
	}
	return TRUE;
}


void removeConnection_cb(NMSettingsConnectionInterface *connection, GError *error, gpointer user_data){
	GMainLoop *loop = user_data;

	if (error)
		errorMessage("Error removing connection: %s", error->message);
	else
		infoMessage("Removed: %s", getConnectionId (NM_CONNECTION (connection)));

	g_main_loop_quit (loop);
}
gboolean removeConnection(NMConnection *connection){
	infoMessage("Removing Connection: %s", getConnectionId (NM_CONNECTION (connection)));
	nm_settings_connection_interface_delete(NM_SETTINGS_CONNECTION_INTERFACE(connection), NULL, NULL);//TODO complains callback is null
	//runInMainLoop(nm_settings_connection_interface_delete, NM_SETTINGS_CONNECTION_INTERFACE(connection), removeConnection_cb, loop);//TODO callback seems to hang
	updateConnections();
	return TRUE;
}
