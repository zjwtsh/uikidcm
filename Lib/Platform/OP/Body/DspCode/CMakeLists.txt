cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(DspPacket)

# include for lua headers
include_directories(/usr/include/lua5.1)
include_directories(/usr/local/include/bfl)

# ImageProc main library definition
add_library(DspPacket SHARED
	luaDspCode.cpp
	DspCode.c
	ctrl_rs232.c)

# set optimization level
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# set library prefix to "" otherwise it will create libImageProc.soset_target_properties(VisionModeling PROPERTIES PREFIX "")
set_target_properties(DspPacket PROPERTIES PREFIX "")
# link to math library
target_link_libraries(DspPacket rt)

install(TARGETS DspPacket DESTINATION ./)
#install_targets(/ DspPacket)

