cmake_minimum_required(VERSION 2.8.3)
project(velo_scanmatch)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_CXX_COMPILER "/home/yoga/lib/clang+llvm-6.0.0-x86_64-linux-gnu-ubuntu-16.04/bin/clang++")

#set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
#set(CMAKE_C_COMPILER "/usr/bin/clang")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -std=gnu++11")
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  laser_assembler
  tf_conversions
  pcl_ros
  pcl_conversions
  message_generation
  eigen_conversions
  protobuf_catkin
  ecl_threads
  tf)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
  ${PROJECT_SOURCE_DIR}/src )
find_package(LuaGoogle REQUIRED)
find_package(Ceres REQUIRED HINTS "/home/agoy/cs_ws/devel_isolated/ceres-solver/lib/cmake/Ceres")

#find_package(PkgConfig REQUIRED)

include_directories(
  include
  include/dataFramework
        ${LUA_INCLUDE_DIR}
    /usr/include/lua5.2
	${CERES_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS} 
#	${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}
	${PCL_INCLUDE_DIRS})

add_service_files(
	FILES
	SaveTraj.srv
	)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS include
)


add_executable(scanRegistration src/scanRegistration.cpp)
target_link_libraries(scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(laserOdometry src/laserOdometry.cpp src/proto_stream.cc src/time_conversion.cc src/time.cc src/msg_conversion.cc src/transform.cc)
#target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(laserOdometry  ${catkin_LIBRARIES} trajproto protobuf ${PCL_LIBRARIES} ${OpenCV_LIBS} glog)

add_executable(laserMapping src/laserMapping.cpp src/proto_stream.cc src/time_conversion.cc src/time.cc src/msg_conversion.cc src/transform.cc)
#target_link_libraries(laserMapping ${catkin_LIBRARIES} trajproto protobuf ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(transformMaintenance src/transformMaintenance.cpp)
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(snapshotter src/snapshotter.cpp)
target_link_libraries(snapshotter ${catkin_LIBRARIES})

add_library(trajproto src/trajectory.pb.cc src/transform.pb.cc src/sensor.pb.cc)
set_target_properties(trajproto PROPERTIES COMPILE_FLAGS "-I/home/loc22/Documents/workspace/protobuf-3.5.1/install/include -pthread" )
#set_target_properties(transformproto PROPERTIES LINK_FLAGS "-rdynamic /home/loc22/Documents/workspace/protobuf-3.5.1/install/lib/libprotobuf.so -pthread -lpthread")

link_directories("/home/loc22/Documents/workspace/protobuf-3.5.1/install/lib/")
add_executable(snapaccumer src/snapaccumer.cpp src/proto_stream.cc src/transform_interpolation_buffer.cc src/proto_stream.cc src/rigid_transform.cc src/time.cc src/transform.cc src/lua_parameter_dictionary.cc)
#add_executable(snapaccumer src/transform.pb.cc src/snapaccumer.cpp )
set_target_properties(snapaccumer PROPERTIES COMPILE_FLAGS "-I/home/loc22/Documents/workspace/protobuf-3.5.1/install/include -pthread" )
#set_target_properties(snapaccumer PROPERTIES LINK_FLAGS "-rdynamic /home/loc22/Documents/workspace/protobuf-3.5.1/install/lib/libprotobuf.so -pthread -lpthread")
target_link_libraries(snapaccumer ${catkin_LIBRARIES} trajproto protobuf ${CERES_LIBRARIES} ${LUA_LIBRARIES} /usr/lib/x86_64-linux-gnu/liblua5.2.a)

target_link_libraries(laserMapping ${catkin_LIBRARIES} trajproto protobuf ${PCL_LIBRARIES} ${OpenCV_LIBS} glog)

#add_executable(snapsaver src/exporter.cpp src/dataFramework/data_model.cpp)
#target_link_libraries(snapsaver ${catkin_LIBRARIES} )

add_executable(traj_process src/trajProcess.cc src/proto_stream.cc src/transform_interpolation_buffer.cc src/proto_stream.cc src/rigid_transform.cc src/time.cc src/transform.cc src/lua_parameter_dictionary.cc src/time_conversion.cc src/gps_odom.cc)
target_link_libraries(traj_process trajproto glog gflags ${catkin_LIBRARIES} ${LUA_LIBRARIES} ${PCL_LIBRARIES} /usr/lib/x86_64-linux-gnu/liblua5.2.a)

add_executable(extract_odom src/extractOdomToProto.cpp src/proto_stream.cc src/transform_interpolation_buffer.cc src/proto_stream.cc src/rigid_transform.cc src/time.cc src/transform.cc src/lua_parameter_dictionary.cc src/time_conversion.cc src/odom_calc.cc src/gps_odom_process.cc src/svy21.cc src/odom_process.cc)
target_link_libraries(extract_odom trajproto glog gflags protobuf ${catkin_LIBRARIES} ${LUA_LIBRARIES} /usr/lib/x86_64-linux-gnu/liblua5.2.a)

# add_executable(test_write_pcd src/test_write_pcd.cc)
# target_link_libraries(test_write_pcd ${catkin_LIBRARIES})

add_dependencies(laserMapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(laserOdometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(calib_process src/calibProcess.cc src/proto_stream.cc src/transform_interpolation_buffer.cc src/proto_stream.cc src/rigid_transform.cc src/time.cc src/transform.cc src/lua_parameter_dictionary.cc src/time_conversion.cc src/gps_odom.cc)
target_link_libraries(calib_process trajproto glog gflags ${catkin_LIBRARIES} ${LUA_LIBRARIES} ${PCL_LIBRARIES} /usr/lib/x86_64-linux-gnu/liblua5.2.a)

add_executable(calib_two_traj src/calibTwoTraj.cpp src/proto_stream.cc src/transform_interpolation_buffer.cc src/proto_stream.cc src/rigid_transform.cc src/time.cc src/transform.cc src/lua_parameter_dictionary.cc src/time_conversion.cc src/gps_odom.cc)
target_link_libraries(calib_two_traj trajproto glog gflags ${catkin_LIBRARIES} ${LUA_LIBRARIES} ${PCL_LIBRARIES} /usr/lib/x86_64-linux-gnu/liblua5.2.a)
