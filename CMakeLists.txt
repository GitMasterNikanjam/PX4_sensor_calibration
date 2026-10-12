cmake_minimum_required(VERSION 3.16)

project(PX4_sensor_calibration LANGUAGES CXX)

add_library(px4_sensor_calibration
	src/Accelerometer.cpp
	src/Barometer.cpp
	src/Gyroscope.cpp
	src/Magnetometer.cpp
	src/Utilities.cpp
)
add_library(PX4::sensor_calibration ALIAS px4_sensor_calibration)
target_compile_features(px4_sensor_calibration PUBLIC cxx_std_11)
target_include_directories(px4_sensor_calibration PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(px4_sensor_calibration PUBLIC PX4::matrix)

option(PX4_SENSOR_CALIBRATION_BUILD_EXAMPLES "Build sensor calibration examples" OFF)

if(PX4_SENSOR_CALIBRATION_BUILD_EXAMPLES)
	add_executable(sensor_calibration_example examples/sensor_calibration_example.cpp)
	target_compile_features(sensor_calibration_example PRIVATE cxx_std_11)
	target_link_libraries(sensor_calibration_example PRIVATE PX4::sensor_calibration)
endif()
