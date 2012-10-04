/*
 * connections.h
 *
 *  Created on: Feb 14, 2012
 *      Author: mitchell
 */

#ifndef CONNECTIONS_H_
#define CONNECTIONS_H_

#include <nm-settings-interface.h>
#include <nm-setting-connection.h>
#include <nm-settings-connection-interface.h>

/**
 * Perform any necessary initialization that needs to occur prior to interacting with network connections
 * This will also enumerate and output all of the network connections on the system
 * @return true if successful
 */
gboolean initNetworkConnections(DBusGConnection *bus, GMainLoop *eventLoop);

/**
 * add a connection to the network manager
 * will remove a connection with the same name
 * @return true on success
 */
gboolean addConnection(DBusGConnection *bus, NMConnection* connection);


/**
 * remove a connection from the network manager
 * @return true on success
 */
gboolean removeConnection(NMConnection *connection);

/**
 * @return the connection path of the given connection id or NULL if the connection is not found, the string is a duplicate so it should be freed
 */
char* getConnectionPath(const char* id);

/**
 * @return the connection with the given connection id or NULL if the connection is not found
 */
NMConnection* getConnection(const char* id);


/**
 * @return the id of the given NMConnection
 */
const char* getConnectionId(NMConnection *connection);


#endif /* CONNECTIONS_H_ */
