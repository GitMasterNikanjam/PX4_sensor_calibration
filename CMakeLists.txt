cmake_minimum_required(VERSION 3.16)

project(PX4_sensor_calibration LANGUAGES CXX)

add_library(px4_sensor_calibration
	Accelerometer.cpp
	Barometer.cpp
	Gyroscope.cpp
	Magnetometer.cpp
	Utilities.cpp
)
add_library(PX4::sensor_calibration ALIAS px4_sensor_calibration)
target_compile_features(px4_sensor_calibration PUBLIC cxx_std_11)
target_include_directories(px4_sensor_calibration PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(px4_sensor_calibration PUBLIC PX4::matrix)
