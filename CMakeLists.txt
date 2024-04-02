cmake_minimum_required(VERSION 3.0.2)
project(mav_sensors_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS 
                roscpp 
                lpp 
                mav_sensors_drivers
                geometry_msgs
                sensor_msgs
                std_msgs
                tf2_eigen
                nodelet
                std_srvs
            )
find_package(Eigen3 REQUIRED)

add_message_files()
generate_messages(DEPENDENCIES std_msgs)

catkin_package( INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
                LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
                CATKIN_DEPENDS roscpp lpp mav_sensors_drivers geometry_msgs sensor_msgs std_msgs roslib tf2_eigen nodelet std_srvs
              )
include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

add_compile_definitions(MODE_ROSLOG)
add_library(${PROJECT_NAME}
        src/base_sensor.cpp
        src/baro.cpp
        src/imu.cpp
        src/radar.cpp
        )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)

add_library(${PROJECT_NAME}_nodelet
        src/nodelets/baro_nodelet.cpp
        src/nodelets/imu_nodelet.cpp
        src/nodelets/radar_nodelet.cpp
        )
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME})

###########
# INSTALL #
###########

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})