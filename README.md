this package is merged from fuerte ROS.
We modified the CMakeList.txt file and package.xml file.

please note now we need use "uic main_window.ui -o ui_main_window.h" to gernerate this .h file and copy it to "include" folder.

catkin_make
source ./devel/setup.bash
rosrun dri_jaguar_manipulator_arm_ctrl1 dri_jaguar_manipulator_arm_ctrl1_node
