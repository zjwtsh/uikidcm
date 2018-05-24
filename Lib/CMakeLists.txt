cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(MOS2018DEC)

ADD_SUBDIRECTORY(Modules)
#ADD_SUBDIRECTORY(Platform/OP)

SET( CMAKE_INSTALL_PREFIX ../Player/Lib/)

MESSAGE( "Setting installation directory to ${CMAKE_INSTALL_PREFIX}" )

