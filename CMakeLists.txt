cmake_minimum_required(VERSION 2.8.3)
project(openface_ros)

set(OPENFACE_ROOT_DIR ~/3rdparty/OpenFace)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  message_runtime
  image_transport
  cv_bridge
  std_msgs
  geometry_msgs
  sensor_msgs
  roscpp
  tf2
  tf2_ros
)

find_package(Boost 1.5.9 REQUIRED COMPONENTS filesystem system)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

find_library(OPENFACE_LD_LIB LandmarkDetector)
find_library(OPENFACE_FA_LIB FaceAnalyser)
find_library(OPENFACE_GA_LIB GazeAnalyser)
find_library(OPENFACE_UT_LIB Utilities)
find_library(DLIB_LIBRARY dlib ${OPENFACE_ROOT_DIR}/build/lib/3rdParty/dlib)

set(OPENFACE_LIBS ${OPENFACE_LD_LIB} ${OPENFACE_FA_LIB} ${OPENFACE_GA_LIB} ${OPENFACE_UT_LIB})

find_path(OPENFACE_INCLUDE_DIR LandmarkDetectorModel.h PATH_SUFFIXES OpenFace)

set(DLIB_INCLUDE_DIR ${OPENFACE_ROOT_DIR}/lib/3rdParty/dlib/include)

add_message_files(FILES ActionUnit.msg Face.msg Faces.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

add_definitions(-std=c++11)
add_definitions(-g -pg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENFACE_INCLUDE_DIR}
  ${BLAS_INCLUDE_DIRS}
  ${DLIB_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}/boost
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_executable(openface_ros src/openface_ros.cpp)
add_dependencies(openface_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(openface_ros openface_ros_generate_messages_cpp)

target_link_libraries(openface_ros
  ${catkin_LIBRARIES}
  ${OPENFACE_LIBS}
  lapack
  openblas
  ${DLIB_LIBRARY}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  tbb
  tbbmalloc
  tbbmalloc_proxy
  ${BLAS_LIBRARIES}
)

install(TARGETS openface_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
