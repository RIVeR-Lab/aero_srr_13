/*
 * util.c
 *
 *  Created on: Feb 10, 2012
 *      Author: mitchell
 */

#include "util.h"
#include <string.h>

BusLoop* newBusLoop(DBusGConnection* bus, GMainLoop* loop){
	BusLoop* busLoop = malloc(sizeof(BusLoop));
	busLoop->bus = bus;
	busLoop->loop = loop;
	return busLoop;
}



char* mallocConcat(char* prefix, char* middle, char* postfix){
	char* output = malloc(sizeof(char)*(strlen(prefix)+strlen(middle)+strlen(postfix)+1));
	strcpy(output, prefix);
	strcpy(output+strlen(prefix), middle);
	strcpy(output+strlen(prefix)+strlen(middle), postfix);
	*(output+strlen(prefix)+strlen(middle)+strlen(postfix)) = '\0';
	return output;
}


char* nmStateToStr(NMState state){
	switch(state){
	case NM_STATE_ASLEEP:
		return "Asleep";
	case NM_STATE_DISCONNECTED:
		return "Disconnected";
	case NM_STATE_CONNECTING:
		return "Connecting";
	case NM_STATE_CONNECTED:
		return "Connected";

	default:
	case NM_STATE_UNKNOWN:
		return "Unknown";
	}
}

char* nmDeviceStateToStr(NMDeviceState state){
	switch(state){
	case NM_DEVICE_STATE_UNMANAGED:
		return "Unmanaged";

	case NM_DEVICE_STATE_UNAVAILABLE:
		return "Unavailable";
	case NM_DEVICE_STATE_PREPARE:
		return "Prepare";
	case NM_DEVICE_STATE_CONFIG:
		return "State Config";
	case NM_DEVICE_STATE_NEED_AUTH:
		return "Need Auth";
	case NM_DEVICE_STATE_IP_CONFIG:
		return "IP Config";
	case NM_DEVICE_STATE_ACTIVATED:
		return "Activated";
	case NM_DEVICE_STATE_FAILED:
		return "Failed";

	default:
	case NM_DEVICE_STATE_UNKNOWN:
		return "Unknown";
	}
}
