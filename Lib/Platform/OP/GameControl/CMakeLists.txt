cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(OPGameControlReceiver)

# include for lua headers
include_directories(/usr/include/lua5.1)
include_directories(/usr/local/include/bfl)

# ImageProc main library definition
add_library(OPGameControlReceiver SHARED
		luaOPGameControlReceiver.cc
		timeScalar.cc)

# set optimization level
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# set library prefix to "" otherwise it will create libImageProc.soset_target_properties(VisionModeling PROPERTIES PREFIX "")
set_target_properties(OPGameControlReceiver PROPERTIES PREFIX "")
# link to math library
target_link_libraries(OPGameControlReceiver m)

install(TARGETS OPGameControlReceiver DESTINATION ./)

