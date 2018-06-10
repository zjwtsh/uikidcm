# this is required
SET(CMAKE_SYSTEM_NAME Linux)

SET(CROSS_COMPILING_ROOT /home/zhangjiwen/Downloads/toolschain/4.5.1/)
# specify the cross compiler
SET(CMAKE_C_COMPILER   ${CROSS_COMPILING_ROOT}/bin/arm-linux-gcc)
SET(CMAKE_CXX_COMPILER ${CROSS_COMPILING_ROOT}/bin/arm-linux-g++)

# where is the target environment 
SET(CMAKE_FIND_ROOT_PATH  ${CROSS_COMPILING_ROOT}/arm-none-linux-gnueabi/sys-root)

# search for programs in the build host directories (not necessary)
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# configure Boost and Qt
# SET(QT_QMAKE_EXECUTABLE /opt/qt-embedded/qmake)
SET(BOOST_ROOT ${CROSS_COMPILING_ROOT}/arm-none-linux-gnueabi/sys-root/usr/include/)

