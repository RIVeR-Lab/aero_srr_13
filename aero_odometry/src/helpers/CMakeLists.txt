#Build filtering libraries
rosbuild_add_library(${PROJECT_NAME}Filters
		      nkfilter.cpp
		      linearinterpoloation.cpp
		      nonlinearanalyticconditionalgaussianmobile.cpp)
