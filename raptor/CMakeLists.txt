cmake_minimum_required(VERSION 2.8.3)
project(raptor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  ffld
  image_transport
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  cmake_modules
  pcl_conversions
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(JPEG REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(PCL REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${JPEG_INCLUDE_DIR}
  ${LIBXML2_INCLUDE_DIR}
  ${PCL_INCLUDE_DIR} 
)

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



#  Not required, but stronlgy recommended on multi-core systems
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_C_FLAGS}")
ENDIF()


SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET")

catkin_package()

add_executable(transfer_images          src/transfer_images.cpp)
add_executable(transfer_images_ffld     src/transfer_images_ffld.cpp)
add_executable(transfer_images_torecord src/transfer_images_torecord.cpp) 

target_link_libraries(transfer_images          ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${JPEG_LIBRARIES} ${LIBXML2_LIBRARIES} ${FFTW3_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(transfer_images_ffld     ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${JPEG_LIBRARIES} ${LIBXML2_LIBRARIES} ${FFTW3_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(transfer_images_torecord ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${JPEG_LIBRARIES} ${LIBXML2_LIBRARIES} ${FFTW3_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})


# Mark executables and/or libraries for installation
install(TARGETS transfer_images transfer_images_torecord transfer_images_ffld 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

