cmake_minimum_required(VERSION 2.6.2)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

link_directories(.)

#rosbuild_gensrv()


rosbuild_add_executable(transfer_images src/transfer_images.cpp)
rosbuild_add_executable(transfer_images_ffld src/transfer_images_ffld.cpp)
rosbuild_add_executable(transfer_images_torecord src/transfer_images_torecord.cpp) 

TARGET_LINK_LIBRARIES(transfer_images_ffld -lFFLD)

FIND_PACKAGE(JPEG REQUIRED)
IF(JPEG_FOUND)
  INCLUDE_DIRECTORIES(${JPEG_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(transfer_images_ffld ${JPEG_LIBRARIES})
ENDIF()




# There are no CMake Eigen package, so find it ourselves
FILE(GLOB EIGEN_ARCHIVE "eigen*")
FIND_PATH(EIGEN_INCLUDE_DIR Eigen ${EIGEN_ARCHIVE} .)
IF(NOT EIGEN_INCLUDE_DIR)
  MESSAGE(FATAL_ERROR "Could not find Eigen.")
ENDIF()
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

# There are no CMake FFTW package, so find it ourselves
FIND_PATH(FFTW3_INCLUDE_DIR fftw3.h)
IF(FFLD_HOGPYRAMID_DOUBLE)
  FIND_LIBRARY(FFTW3_LIBRARIES fftw3)
ELSE()
  FIND_LIBRARY(FFTW3_LIBRARIES fftw3f)
ENDIF()
IF(NOT FFTW3_INCLUDE_DIR OR NOT FFTW3_LIBRARIES)
  MESSAGE(FATAL_ERROR "Could not find fftw3.")
ENDIF()
INCLUDE_DIRECTORIES(${FFTW3_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(transfer_images_ffld ${FFTW3_LIBRARIES})


# Not required, but stronlgy recommended on multi-core systems
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_C_FLAGS}")
ENDIF()




#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()


