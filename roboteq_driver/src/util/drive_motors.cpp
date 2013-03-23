#include "roboteq_driver/roboteq_motor_controller.h"
#include <stdio.h>
#include <unistd.h>

int main(int argc, char **argv){
	if(argc!=5){
		printf("Usage:\n %s port leftSpeed rightSpeed time\n", argv[0]);
		return 1;
	}
	char *endp;
	double runTime = strtod(argv[4], &endp);
	if (argv[4] == endp || *endp != '\0'){
		fprintf(stderr, "Invalid time parameter\n");
		return 1;
	}
	double leftSpeed = strtod(argv[2], &endp);
	if (argv[2] == endp || *endp != '\0'){
		fprintf(stderr, "Invalid left speed parameter\n");
		return 1;
	}
	double rightSpeed = strtod(argv[3], &endp);
	if (argv[3] == endp || *endp != '\0'){
		fprintf(stderr, "Invalid right speed parameter\n");
		return 1;
	}
	printf("Running motors at (%f, %f) for %fs\n", leftSpeed, rightSpeed, runTime);
	roboteq_driver::RoboteqMotorController controller(2.0, 2.0, 2.0, 2.0, 250, 250);
	controller.open(argv[1]);
	for(double time = 0; time<runTime; time+=0.1){
		controller.setSpeed(leftSpeed, rightSpeed);
		usleep(100000);
	}
	controller.setSpeed(0, 0);
	controller.close();
	return 0;
}
