/*
 * adhoc.c
 *
 *  Created on: Feb 14, 2012
 *      Author: mitchell
 */

#include "adhoc.h"
#include "connections.h"
#include <nm-setting-connection.h>
#include <nm-setting-wireless.h>
#include <nm-setting-ip4-config.h>
#include <nm-utils.h>

gboolean addAdhocConnection(DBusGConnection* bus, char* name){
	NMConnection *connection = nm_connection_new ();
	NMSettingConnection *connectionSettings = (NMSettingConnection *) nm_setting_connection_new ();
	char* uuid = nm_utils_uuid_generate();
	g_object_set (G_OBJECT (connectionSettings),
				  NM_SETTING_CONNECTION_UUID, uuid,
				  NM_SETTING_CONNECTION_ID, name,
				  NM_SETTING_CONNECTION_TYPE, NM_SETTING_WIRELESS_SETTING_NAME,
				  NULL);
	g_free (uuid);
	nm_connection_add_setting (connection, NM_SETTING (connectionSettings));

	NMSettingWireless *wirelessSetting = (NMSettingWireless *) nm_setting_wireless_new ();
	GByteArray *ssid = g_byte_array_sized_new (sizeof (name));
	g_byte_array_append (ssid, (const guint8*)name, sizeof (name));
	g_object_set (G_OBJECT (wirelessSetting),
			NM_SETTING_WIRELESS_SSID, ssid,
			NM_SETTING_WIRELESS_MODE, "adhoc",
				  NULL);
	nm_connection_add_setting (connection, NM_SETTING (wirelessSetting));

	NMSettingIP4Config *ipv4Setting = (NMSettingIP4Config *) nm_setting_ip4_config_new ();
	g_object_set (G_OBJECT (ipv4Setting),
				  NM_SETTING_IP4_CONFIG_METHOD, NM_SETTING_IP4_CONFIG_METHOD_SHARED,
				  NULL);
	nm_connection_add_setting (connection, NM_SETTING (ipv4Setting));

	gboolean success = addConnection(bus, connection);

	g_object_unref (connection);

	return success;
}
