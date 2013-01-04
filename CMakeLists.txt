cmake_minimum_required(VERSION 2.8)
project(object_recognition_linemod)
find_package(catkin REQUIRED object_recognition_core object_recognition_renderer)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
catkin_package(DEPENDS object_recognition_core object_recognition_renderer)
else()
catkin_stack()
catkin_project(object_recognition_linemod DEPENDS object_recognition_core object_recognition_renderer)
find_package(object_recognition_core)
find_package(object_recognition_renderer)
list(APPEND catkin_INCLUDE_DIRS ${object_recognition_core_INCLUDE_DIRS} ${object_recognition_renderer_INCLUDE_DIRS})
list(APPEND catkin_LIBRARIES ${object_recognition_core_LIBRARIES} ${object_recognition_renderer_LIBRARIES})
endif()

#install targets for all things python
catkin_python_setup()

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

# Compile the module
add_subdirectory(python)
add_subdirectory(sample)
add_subdirectory(src)

# add the tests
option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)    
  enable_testing()
  add_subdirectory(test)
endif()
