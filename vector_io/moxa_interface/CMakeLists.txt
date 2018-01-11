cmake_minimum_required(VERSION 2.8.3)
project(moxa_interface)

find_package(catkin REQUIRED COMPONENTS rospy)

## Uncomment if the package has a setup.py
catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
                        bin/moxa_modbus_driver
                        bin/moxa_rest_driver
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
                      
