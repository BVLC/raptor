#                          Implementation of the paper
#                "Exact Acceleration of Linear Object Detectors"
#              12th European Conference on Computer Vision, 2012.
#
#      Copyright (c) 2012 Idiap Research Institute, <http://www.idiap.ch/>
#              Written by Charles Dubout <charles.dubout@idiap.ch>

cmake_minimum_required(VERSION 2.8.3)
project(ffld)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED cmake_modules)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(JPEG REQUIRED)
find_package(LibXml2 REQUIRED)

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


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES FFLD
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS ${JPEG_LIBRARIES}
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${JPEG_INCLUDE_DIR}
  ${LIBXML2_INCLUDE_DIR}
)


# The build options
OPTION(FFLD_HOGPYRAMID_FELZENSZWALB_FEATURES "Use Felzenszwalb's original features (slower and not as accurate, provided for compatibility only)." OFF)
OPTION(FFLD_HOGPYRAMID_DOUBLE "Use doubles instead of floats (slower, uses twice more memory, and the increase in precision is not necessarily useful)." OFF)

# Select a default build configuration if none was chosen
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()

SET(CMAKE_CXX_FLAGS_DEBUG "-pg")

SET(SOURCES src/Rectangle.cpp src/JPEGImage.cpp src/Object.cpp src/Scene.cpp src/HOGPyramid.cpp
src/Patchwork.cpp src/Model.cpp src/Mixture.cpp src/DPMDetection.cpp)

ADD_LIBRARY(FFLD ${SOURCES})

ADD_EXECUTABLE(ffld src/ffld.cpp)
ADD_EXECUTABLE(detect src/detect.cpp)
ADD_EXECUTABLE(detect_opencv include/ffld/opencvToJPEGImage.h src/detect_opencv.cpp)

TARGET_LINK_LIBRARIES(FFLD ${Eigen_LIBRARIES} ${JPEG_LIBRARIES} ${LIBXML2_LIBRARIES} ${FFTW3_LIBRARIES})

TARGET_LINK_LIBRARIES(ffld FFLD ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${JPEG_LIBRARIES} ${LIBXML2_LIBRARIES} ${FFTW3_LIBRARIES})
TARGET_LINK_LIBRARIES(detect FFLD ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${JPEG_LIBRARIES} ${LIBXML2_LIBRARIES} ${FFTW3_LIBRARIES})
TARGET_LINK_LIBRARIES(detect_opencv FFLD ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES} ${LIBXML2_LIBRARIES} ${FFTW3_LIBRARIES})

# Define the options
IF(FFLD_HOGPYRAMID_FELZENSZWALB_FEATURES)
  ADD_DEFINITIONS(-DFFLD_HOGPYRAMID_FELZENSZWALB_FEATURES)
ENDIF()

IF(FFLD_HOGPYRAMID_DOUBLE)
  ADD_DEFINITIONS(-DFFLD_HOGPYRAMID_DOUBLE)
ENDIF()


# Not required, but stronlgy recommended on multi-core systems
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_C_FLAGS}")
ENDIF()


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
install(TARGETS ffld detect detect_opencv FFLD 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)





