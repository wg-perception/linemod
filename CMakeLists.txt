cmake_minimum_required(VERSION 2.8)
project(object_recognition_linemod)
find_package(catkin REQUIRED)
if (${catkin_VERSION} VERSION_GREATER "0.5.28")
catkin_package()
else()
catkin_stack()
catkin_project(object_recognition_linemod)
endif()

#install targets for all things python
catkin_python_setup()

find_package(ecto REQUIRED)

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

