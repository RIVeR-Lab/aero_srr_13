#Build filtering libraries
rosbuild_add_library(${PROJECT_NAME}Filters
		      nkfilter.cpp
		      linearinteropolation.cpp
		      nonlinearanalyticconditionalgaussianmobile.cpp)