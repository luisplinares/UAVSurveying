cmake_minimum_required(VERSION 2.8.3)
project(apx15)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
             message_generation
             roscpp 
             std_msgs
             roslint 
             serial 
             sensor_msgs
             )

## Generate messages in the 'msg' folder
add_message_files(
	FILES
    RawUbx.msg
    GSOF.msg
    GNSS.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
  )


add_subdirectory(
  "/home/srcc/Documents/VectorNav/vnproglib-1.1.5.0/cpp" libvncxx
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  "/home/srcc/Documents/VectorNav/vnproglib-1.1.5.0/cpp/include" #vectornav
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
add_executable(apx15_gsof_driver src/main.cpp src/comms.cpp src/apx15_gsof_parser.cpp src/Utils.cpp src/minIni.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(apx15_gsof_driver ${catkin_LIBRARIES} )

target_link_libraries(apx15_gsof_driver PRIVATE libvncxx)

if (UNIX OR APPLE)
	target_link_libraries(apx15_gsof_driver LINK_PUBLIC pthread)
else()
	#target_link_libraries(apx15_gsof_driver LINK_PUBLIC Advapi32 SetupAPI)
endif()


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS apx15_gsof_driver
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#---------------------------------------------------------------------------------------
# end of CMakeList.txt
#---------------------------------------------------------------------------------------
