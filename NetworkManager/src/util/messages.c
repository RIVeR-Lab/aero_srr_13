/*
 * messages.c
 *
 *  Created on: Feb 10, 2012
 *      Author: mitchell
 */

#include "messages.h"
#include "util.h"
#include "../config.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <nm-client.h>
#include <nm-device.h>
#include "../connection/connections.h"

void debugMessage(char* fmt, ...){
#ifdef DEBUG
	char* tmp = mallocConcat("DEBUG: ", fmt, "\n");
	va_list args;
	va_start(args,fmt);
	vfprintf(stdout, tmp, args);
	fflush(stdout);
	va_end(args);
	free(tmp);
	fflush(stdout);
#endif
}

void infoMessage(char* fmt, ...){
	char* tmp = mallocConcat("INFO: ", fmt, "\n");
	va_list args;
	va_start(args,fmt);
	vfprintf(stdout, tmp, args);
	fflush(stdout);
	va_end(args);
	free(tmp);
	fflush(stdout);
}

void errorMessage(char* fmt, ...){
	char* tmp = mallocConcat("ERROR: ", fmt, "\n");
	va_list args;
	va_start(args,fmt);
	vfprintf(stderr, tmp, args);
	fflush(stderr);
	va_end(args);
	free(tmp);
	fflush(stderr);
}

void updateDeviceList(const GPtrArray* devices){
	int i;
	printf("::Network Device List:\n");
	for(i = 0; i<devices->len; ++i){
		NMDevice* device = (NMDevice*)g_ptr_array_index(devices, i);
		printf("::\t%s on '%s' in state '%s'\n", nm_device_get_product(device), nm_device_get_iface(device), nmDeviceStateToStr(nm_device_get_state(device)));
	}
	fflush(stdout);
}

void updateConnectionsList(GSList *connections){
#ifdef DEBUG
	int i;
	printf("::Network Connections List:\n");
	for(i = 0; i<g_slist_length(connections); ++i){
		NMConnection* connection = (NMConnection*)g_slist_nth_data(connections, i);
		printf("::\t%s at '%s'\n", getConnectionId(connection), nm_connection_get_path(connection));
	}
	fflush(stdout);
#endif
}
