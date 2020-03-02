cmake_minimum_required(VERSION 2.8.3)
project(hector)

set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "-fopenmp")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosflight_msgs
  hector_msgs
  dynamic_reconfigure
  sensor_msgs
  tf
)
find_package(Eigen3 REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Follower.cfg
  cfg/Controller.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS roscpp rospy rosflight_msgs hector_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

## Declare a C++ executable
#add_executable(hector_controller
#            src/controller_base.cpp
#            src/controller_example.cpp)
#add_dependencies(hector_controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
#target_link_libraries(hector_controller ${catkin_LIBRARIES})

#add_executable(hector_controller99
#            src/controller_base99.cpp
#            src/controller_example99.cpp)
#add_dependencies(hector_controller99 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
#target_link_libraries(hector_controller99 ${catkin_LIBRARIES})

add_executable(hector_pseudo_controller
           # src/controller_base.cpp
           # src/controller_example.cpp)
             src/pseudo_controller_base.cpp
             src/pseudo_controller_example.cpp)
add_dependencies(hector_pseudo_controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(hector_pseudo_controller ${catkin_LIBRARIES})

## Declare a C++ executable
#add_executable(hector_estimator
#            src/estimator_base.cpp
#            src/estimator_example.cpp)
#add_dependencies(hector_estimator ${catkin_EXPORTED_TARGETS})
#target_link_libraries(hector_estimator ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(hector_estimator_statesub
            src/estimator_base_statesub.cpp
            )
add_dependencies(hector_estimator_statesub ${catkin_EXPORTED_TARGETS})
target_link_libraries(hector_estimator_statesub ${catkin_LIBRARIES})

## Declare a C++ executable
#add_executable(hector_estimator_updata_statesub
#            src/estimator_base_updata_statesub.cpp
#            )
#add_dependencies(hector_estimator_updata_statesub ${catkin_EXPORTED_TARGETS})
#target_link_libraries(hector_estimator_updata_statesub ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(hector_estimator_99pub
            src/estimator_base_99pub.cpp
            )
add_dependencies(hector_estimator_99pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(hector_estimator_99pub ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(hector_path_follower
            src/path_follower_example.cpp
            src/path_follower_base.cpp)
add_dependencies(hector_path_follower ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(hector_path_follower ${catkin_LIBRARIES})


## Declare a C++ executable
add_executable(hector_path_manager
            src/path_manager_base.cpp
            src/path_manager_example.cpp)
add_dependencies(hector_path_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(hector_path_manager ${catkin_LIBRARIES})


## Declare a C++ executable
add_executable(hector_path_planner
            src/path_planner.cpp)
add_dependencies(hector_path_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(hector_path_planner ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(hector_path_point_transfer
            src/path_point_transfer.cpp)      
add_dependencies(hector_path_point_transfer ${catkin_EXPORTED_TARGETS})
target_link_libraries(hector_path_point_transfer ${catkin_LIBRARIES})

# ## Declare a C++ executable
# add_executable(udp_send_image
# 	src/udp_send_image.cpp
# 	)
# add_dependencies(udp_send_image
# 	${catkin_EXPORTED_TARGETS}
# 	)
# target_link_libraries(udp_send_image
# 	${catkin_LIBRARIES}
# 	${OpenCV_LIBRARIES}
# 	)

# ## Declare a C++ executable
# add_executable(udp_recv_image
# 	src/udp_recv_image.cpp
# 	)
# add_dependencies(udp_recv_image
# 	${catkin_EXPORTED_TARGETS}
# 	)
# target_link_libraries(udp_recv_image
# 	${catkin_LIBRARIES}
# 	${OpenCV_LIBRARIES}
# 	)

# ## Declare a C++ executable
# add_executable(udp_send_state
#             src/udp_send_state.cpp)
# add_dependencies(udp_send_state ${catkin_EXPORTED_TARGETS})
# target_link_libraries(udp_send_state ${catkin_LIBRARIES})

