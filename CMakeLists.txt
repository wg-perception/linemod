cmake_minimum_required(VERSION 2.8)
project(object_recognition_linemod)

find_package(catkin)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
find_package(catkin REQUIRED object_recognition_core object_recognition_renderer)
catkin_package(CATKIN_DEPENDS ecto object_recognition_core object_recognition_renderer
               DEPENDS OpenCV
)
else()
find_package(ROS REQUIRED ecto object_recognition_core object_recognition_renderer)
catkin_stack()
catkin_project(object_recognition_linemod DEPENDS object_recognition_core object_recognition_renderer)
set(catkin_LIBRARIES ${ROS_LIBRARIES})
set(catkin_INCLUDE_DIRS ${ROS_INCLUDE_DIRS})
set(CATKIN_DEVEL_PREFIX ${CMAKE_BINARY_DIR})
set(CATKIN_PACKAGE_BIN_DESTINATION share/${PROJECT_NAME}/bin)
set(CATKIN_PACKAGE_INCLUDE_DESTINATION include/${PROJECT_NAME})
set(CATKIN_PACKAGE_LIB_DESTINATION lib)
set(CATKIN_PACKAGE_PYTHON_BUILD_DESTINATION gen/py/${PROJECT_NAME}/)
set(CATKIN_PACKAGE_PYTHON_DESTINATION ${INSTALLED_PYTHONPATH}/${PROJECT_NAME}/)
set(CATKIN_PACKAGE_SHARE_DESTINATION share/${PROJECT_NAME}/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

#install targets for all things python
catkin_python_setup()

find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

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

# build the docs
ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/source doc)

# install the conf files
install(DIRECTORY conf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
