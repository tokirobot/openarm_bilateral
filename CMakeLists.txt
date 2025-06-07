# Copyright 2025 Reazon Holdings, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(openarm_bilateral)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# TRUE : if you wanna log
set(LOGGING FALSE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# added
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

message(STATUS "AAAA")

include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
        src
        )

add_library(bilateral_openarm STATIC
        src/canfd/canbus.cpp
        src/dmmotor/damiao_port.cpp
        src/dmmotor/motor_control.cpp
        src/dmmotor/motor.cpp
        src/controller/control.cpp
        src/controller/dynamics.cpp
        )


ament_target_dependencies(bilateral_openarm
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        urdf
        kdl_parser
        orocos_kdl
        ament_index_cpp
        )

if(LOGGING)
        message(STATUS "Building with logging support.")
        find_package(OpenCV REQUIRED)
        find_package(realsense2 REQUIRED)

        add_executable(bilateral_openarm_main_logging src/bilateral_main_logging.cpp)
        target_link_libraries(bilateral_openarm_main_logging
                bilateral_openarm
                ${OpenCV_LIBS}
                realsense2::realsense2
                )

        install(TARGETS
                bilateral_openarm_main_logging
                DESTINATION lib/${PROJECT_NAME}
                )
else()
        message(STATUS "Building standard main without logging.")
        add_executable(bilateral_openarm_main src/bilateral_main.cpp)
        target_link_libraries(bilateral_openarm_main bilateral_openarm)

        install(TARGETS
                bilateral_openarm_main
                DESTINATION lib/${PROJECT_NAME}
                )
endif()

#add_executable(bilateral_openarm_main src/bilateral_main.cpp)
#target_link_libraries(bilateral_openarm_main bilateral_openarm)

add_executable(sample_dynamics samples/check_dynamics.cpp)
target_link_libraries(sample_dynamics bilateral_openarm)

add_executable(sample_grav_comp samples/check_grav_comp.cpp)
target_link_libraries(sample_grav_comp bilateral_openarm)

add_executable(constant_speed_test samples/constant_speed_test.cpp)
target_link_libraries(constant_speed_test bilateral_openarm)

add_executable(canfd_test samples/canfd_test.cpp)
target_link_libraries(canfd_test bilateral_openarm)

install(TARGETS
        #bilateral_openarm_main
        sample_dynamics
        sample_grav_comp
        constant_speed_test
        canfd_test
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
        )

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
        )

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        set(ament_cmake_copyright_FOUND TRUE)
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()
