cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(OPCamera)

# include for lua headers
include_directories(/usr/include/lua5.1)
include_directories(/usr/local/include/bfl)

# ImageProc main library definition
add_library(OPCam SHARED
	timeScalar.cc
	OPCam.cpp
	v4l2.cpp)

# set optimization level
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# set library prefix to "" otherwise it will create libImageProc.soset_target_properties(VisionModeling PROPERTIES PREFIX "")
set_target_properties(OPCam PROPERTIES PREFIX "")
# link to math library
target_link_libraries(OPCam m rt)

install(TARGETS OPCam DESTINATION ./)

