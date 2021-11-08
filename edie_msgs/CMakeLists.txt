cmake_minimum_required(VERSION 2.8.3)
project(edie_msgs)

find_package(catkin REQUIRED COMPONENTS
	roscpp
  std_msgs
	message_generation
)

add_message_files(
	FILES
	EdieView.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
     include
     ${catkin_INCLUDE_DIRS}
)

set(PROJECT_LIB_FILES
    src/edie_struct.cpp
)

add_library(${PROJECT_NAME}
    ${PROJECT_LIB_FILES}
)

add_dependencies(${PROJECT_NAME}
   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
