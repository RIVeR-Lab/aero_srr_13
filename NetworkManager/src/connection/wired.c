/*
 * adhoc.c
 *
 *  Created on: Feb 14, 2012
 *      Author: mitchell
 */

#include "wired.h"
#include "connections.h"
#include <nm-setting-connection.h>
#include <nm-setting-wired.h>
#include <nm-setting-ip4-config.h>
#include <nm-utils.h>

gboolean addWiredConnection(DBusGConnection* bus, char* name, guint32 a1, guint32 a2, guint32 a3, guint32 a4){
	NMConnection *connection = nm_connection_new ();
	NMSettingConnection *connectionSettings = (NMSettingConnection *) nm_setting_connection_new ();
	char* uuid = nm_utils_uuid_generate();
	g_object_set (G_OBJECT (connectionSettings),
				  NM_SETTING_CONNECTION_UUID, uuid,
				  NM_SETTING_CONNECTION_ID, name,
				  NM_SETTING_CONNECTION_TYPE, NM_SETTING_WIRED_SETTING_NAME,
				  NULL);
	g_free (uuid);
	nm_connection_add_setting (connection, NM_SETTING (connectionSettings));

	NMSettingWired *wiredSetting = (NMSettingWired *) nm_setting_wired_new ();
	nm_connection_add_setting (connection, NM_SETTING (wiredSetting));

	NMSettingIP4Config *ipv4Setting = (NMSettingIP4Config *) nm_setting_ip4_config_new ();
	NMIP4Address* address = nm_ip4_address_new();
	guint32 staticIp = ((a4&0xFF)<<24) | ((a3&0xFF)<<16) | ((a2&0xFF)<<8) | (a1&0xFF);
	nm_ip4_address_set_address(address, staticIp);
	nm_ip4_address_set_prefix(address, 24);
	nm_setting_ip4_config_add_address(ipv4Setting, address);
	g_object_set (G_OBJECT (ipv4Setting),
				  NM_SETTING_IP4_CONFIG_METHOD, NM_SETTING_IP4_CONFIG_METHOD_MANUAL,
				  NULL);
	nm_connection_add_setting (connection, NM_SETTING (ipv4Setting));

	gboolean success = addConnection(bus, connection);

	g_object_unref (connection);

	return success;
}
