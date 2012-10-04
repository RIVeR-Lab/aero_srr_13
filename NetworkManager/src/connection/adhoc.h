/*
 * adhoc.h
 *
 *  Created on: Feb 14, 2012
 *      Author: mitchell
 */

#ifndef ADHOC_H_
#define ADHOC_H_

#include <dbus/dbus-glib.h>

/**
 * Add an adhoc network connection
 * @return true on success
 */
gboolean addAdhocConnection(DBusGConnection* bus, char* name);

#endif /* ADHOC_H_ */
