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
	roboteq_driver::RoboteqMotorController controller(1000.0, 1000.0, 5000, 5000);
	controller.open(argv[1]);
	for(double time = 0; time<runTime; time+=0.1){
	  controller.setRPM(1, leftSpeed);
	  controller.setRPM(2, rightSpeed);
	  double left, right;
	  double leftf, rightf;
	  controller.getPosition(1, left);
	  controller.getPosition(2, right);
	  printf("Pos: %f, %f\n", left, right);
	  controller.getCurrent(1, leftf);
	  controller.getCurrent(2, rightf);
	  printf("Current: %f, %f\n", leftf, rightf);
	  controller.getTemp(1, leftf);
	  controller.getTemp(2, rightf);
	  printf("Temp: %f, %f\n", leftf, rightf);
	  usleep(100000);
	}
	controller.setPower(1, 0);
	controller.setPower(2, 0);
	controller.close();
	return 0;
}
