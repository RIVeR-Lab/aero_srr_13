#include "hd_driver/hd_motor_controller.h"
#include <stdio.h>
#include <unistd.h>

int main(int argc, char **argv){
	if(argc!=3){
		printf("Usage:\n %s port position\n", argv[0]);
		return 1;
	}
	char *endp;
	long position = strtol(argv[2], &endp, 10);
	if (argv[2] == endp || *endp != '\0'){
		fprintf(stderr, "Invalid position\n");
		return 1;
	}
	hd_driver::HDMotorController controller;
	controller.open(argv[1]);

	printf("Motor starting at: %d\n", controller.get_position());
	printf("Moving motor to: %ld\n", position);
	controller.set_position(position);
	while(controller.get_status()&STATUS_TRAJECTORY_RUNNING){
	  usleep(100000);
	  printf("Motor at: %d\n", controller.get_position());
	}
	if(controller.get_trajectory_status()&TRAJECTORY_MOVE_ABORTED)
	  printf("Move aborted\n");
	printf("status = 0x%X\n", controller.get_status());
	printf("trajectory status = 0x%X\n", controller.get_trajectory_status());
	printf("Motor finished at: %d\n", controller.get_position());
	controller.close();
	return 0;
}
