/*
 * util.h
 *
 *  Created on: Feb 10, 2012
 *      Author: mitchell
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <dbus/dbus-glib.h>
#include <nm-connection.h>
#include <NetworkManager.h>

/**
 * will encapsulate the running of a function in a main loop
 * the loop will exist with the name loop
 * it is the responsibility of the called function to call g_main_loop_quit on the loop
 */
#define runInMainLoop(function, ...)\
	{GMainLoop* loop = g_main_loop_new(NULL, FALSE);\
	function (__VA_ARGS__);\
	g_main_loop_run(loop);\
	g_main_loop_unref (loop);}

/**
 * will encapsulate the running of a function in a main loop
 * the loop will exist with the name loop
 * it is the responsibility of the called function to call g_main_loop_quit on the loop
 * the loop will only be run if the function returns true
 */
#define runIfSuccessInMainLoop(function, ...)\
	{GMainLoop* loop = g_main_loop_new(NULL, FALSE);\
	if(function (__VA_ARGS__))\
	g_main_loop_run(loop);\
	g_main_loop_unref (loop);}

/**
 * will encapsulate the running of a function in a main loop
 * the loop will exist with the name loop
 * it is the responsibility of the called function to call g_main_loop_quit on the loop
 * the result of the function call will be assigned to a variable named result
 */
/*#define callInMainLoop(type, function, ...)\
	GMainLoop* loop = g_main_loop_new(NULL, FALSE);\
	type result = function (__VA_ARGS__);\
	g_main_loop_run(loop);\
	g_main_loop_unref (loop);\*/

/**
 * Structure that can store the dbus connection and glib main loop to be passed into a callback function as user data
 */
struct BusLoop{
	DBusGConnection* bus;
	GMainLoop* loop;
};
typedef struct BusLoop BusLoop;

/**
 * Allocate and fill a new BusLoop structure
 */
BusLoop* newBusLoop(DBusGConnection* bus, GMainLoop* loop);

/**
 * malloc a new string that will hold the two strings passed to it and concatenate both in the new memory
 * @param prefix the first string
 * @param middle the second string
 * @param postfix the third string
 *
 */
char* mallocConcat(char* prefix, char* middle, char* postfix);

char* nmStateToStr(NMState state);

char* nmDeviceStateToStr(NMDeviceState state);

#endif /* UTIL_H_ */
