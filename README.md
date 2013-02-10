aero_srr
========

This repo maintains the a collection of ROS packages for NASA Sample Return Robot Challenge Team AERO.
All packages are in the root directory of the repository.

Development
-----------
Checkout the repository to the src directory of the catkin worksapce (`${catkin_workspace}/src/aero_srr`). To build the packages return to the catkin workspace directory and then run `catkin_make`. To do a clean build run `rm -r build devel`.

The development branch is the 'in-flux' version of the master branch. Code which has been at least tested for basic functionality, but not for full validation, should be merged into this branch.

When creating/modify data for the development branch, create a new 'topic' branch off of development that will contain all of your commits as you work on your modifcations. Once your work is finished and gone through basic testing, merge it back into the development branch.


Packages
--------
- **aero\_drive\_controller**: A package containing a node which provides services for driving the aero platform (such as driving arcs)
- **aero\_path\_planning**: A node which plans robot movement using tentacles
- **aero\_srr**: A meta package for all of the aero\_srr packages
- **aero\_srr\_bringup**: A collection of lauch files for starting up the SRR packages
- **aero\_srr\_gui**: An example gui demonstrating using QT4 in ROS
- **aero\_srr\_msgs**: A package containing all of the messages used by the AERO SRR packages
