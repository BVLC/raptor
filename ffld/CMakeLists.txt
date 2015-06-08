#                          Implementation of the paper
#                "Exact Acceleration of Linear Object Detectors"
#              12th European Conference on Computer Vision, 2012.
#
#      Copyright (c) 2012 Idiap Research Institute, <http://www.idiap.ch/>
#              Written by Charles Dubout <charles.dubout@idiap.ch>

cmake_minimum_required(VERSION 2.8.3)
project(ffld_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
#  INCLUDE_DIRS include
#  LIBRARIES ffld_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)



## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ffld_ros_node ffld_ros_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ffld_ros_node
#   ${catkin_LIBRARIES}
# )



# The build options
OPTION(FFLD_HOGPYRAMID_FELZENSZWALB_FEATURES "Use Felzenszwalb's original features (slower and not as accurate, provided for compatibility only)." OFF)
OPTION(FFLD_HOGPYRAMID_DOUBLE "Use doubles instead of floats (slower, uses twice more memory, and the increase in precision is not necessarily useful)." OFF)

# Select a default build configuration if none was chosen
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()

SET(CMAKE_CXX_FLAGS_DEBUG "-pg")

MESSAGE(ffld/Rectangle.h)

# Also list the headers so that they are displayed along the .cpp files in the IDE
SET(HEADERS ffld/Rectangle.h ffld/JPEGImage.h ffld/Object.h ffld/Scene.h ffld/HOGPyramid.h ffld/Patchwork.h ffld/Model.h ffld/Mixture.h ffld/Intersector.h)

SET(SOURCES ffld/Rectangle.cpp ffld/JPEGImage.cpp ffld/Object.cpp ffld/Scene.cpp ffld/HOGPyramid.cpp
ffld/Patchwork.cpp ffld/Model.cpp ffld/Mixture.cpp ffld/DPMDetection.cpp)

ADD_LIBRARY(FFLD ${SOURCES})

ADD_EXECUTABLE(ffld ffld/ffld.cpp)
ADD_EXECUTABLE(detect ffld/detect.cpp)
TARGET_LINK_LIBRARIES(ffld FFLD)
TARGET_LINK_LIBRARIES(detect FFLD)


# Define the options
IF(FFLD_HOGPYRAMID_FELZENSZWALB_FEATURES)
  ADD_DEFINITIONS(-DFFLD_HOGPYRAMID_FELZENSZWALB_FEATURES)
ENDIF()

IF(FFLD_HOGPYRAMID_DOUBLE)
  ADD_DEFINITIONS(-DFFLD_HOGPYRAMID_DOUBLE)
ENDIF()

# There are no CMake Eigen package, so find it ourselves
FILE(GLOB EIGEN_ARCHIVE "ffld/eigen*")
FIND_PATH(EIGEN_INCLUDE_DIR ffld/Eigen ${EIGEN_ARCHIVE} .)
IF(NOT EIGEN_INCLUDE_DIR)
  MESSAGE(FATAL_ERROR "Could not find Eigen.")
ENDIF()
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
INCLUDE_DIRECTORIES(ffld)

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
TARGET_LINK_LIBRARIES(ffld ${FFTW3_LIBRARIES})
TARGET_LINK_LIBRARIES(detect ${FFTW3_LIBRARIES})


FIND_PACKAGE(JPEG REQUIRED)
IF(JPEG_FOUND)
  INCLUDE_DIRECTORIES(${JPEG_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(ffld ${JPEG_LIBRARIES})
  TARGET_LINK_LIBRARIES(detect ${JPEG_LIBRARIES})
ENDIF()

FIND_PACKAGE(OpenCV)
IF(OpenCV_FOUND)
  ADD_EXECUTABLE(detect_opencv ${HEADERS} ${SOURCES} ffld/opencvToJPEGImage.h ffld/detect_opencv.cpp)
  INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(detect_opencv ${OpenCV_LIBS})
  TARGET_LINK_LIBRARIES(detect_opencv ${JPEG_LIBRARIES})
  TARGET_LINK_LIBRARIES(detect_opencv ${FFTW3_LIBRARIES})
ENDIF()

FIND_PACKAGE(LibXml2 REQUIRED)
IF(LIBXML2_FOUND)
  INCLUDE_DIRECTORIES(${LIBXML2_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(ffld ${LIBXML2_LIBRARIES})
  TARGET_LINK_LIBRARIES(detect ${LIBXML2_LIBRARIES})
  IF(OpenCV_FOUND)
    TARGET_LINK_LIBRARIES(detect_opencv ${LIBXML2_LIBRARIES})
  ENDIF()
  ADD_DEFINITIONS(${LIBXML2_DEFINITIONS})
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

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation
install(TARGETS ffld detect detect_opencv FFLD 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(FILES ${HEADERS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
