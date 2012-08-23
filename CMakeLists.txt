cmake_minimum_required(VERSION 2.8)
project(object_recognition_linemod)
find_package(catkin REQUIRED)
catkin_stack()
catkin_project(object_recognition_linemod
               DEPENDS OpenCV object_recognition_core
)

#install targets for all things python
catkin_python_setup()

find_package(ecto REQUIRED)

# Compile the module
set_ecto_install_package_name(${PROJECT_NAME}/ecto_cells)

add_subdirectory(python)
add_subdirectory(src)

# add the tests
option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)    
  enable_testing()
  add_subdirectory(test)
endif()

