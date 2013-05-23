include $(shell rospack find mk)/cmake.mk
upload: all
	make -f $(shell rospack find aero_arduino)/build/Makefile -C $(shell rospack find aero_arduino)/build aero_arduino-upload
