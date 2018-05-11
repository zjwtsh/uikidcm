cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(VisionModeling)

# include for lua headers
include_directories(/usr/include/lua5.1)
include_directories(/usr/local/include/bfl)

# ImageProc main library definition
add_library(VisionModeling SHARED
	preprocessedObservation.cpp
	ballModelingObject.cpp
	lua_accumulate_ball.cpp
	luaVisionModeling.cpp
	nonlinearMeasurementPdf.cpp
	nonlinearSystemPdf.cpp)

# set optimization level
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# set library prefix to "" otherwise it will create libImageProc.so
set_target_properties(VisionModeling PROPERTIES PREFIX "")

# link to math library
target_link_libraries(VisionModeling m orocos-bfl)

install_targets(/ VisionModeling)

