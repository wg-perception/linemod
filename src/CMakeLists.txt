# Compile the ecto 
if(LINEMOD_VIZ_IMG)
  add_definitions("-DLINEMOD_VIZ_IMG=1")
endif()

set(SRC_FILES db_linemod.cpp
              linemod_detect.cpp
              linemod_train.cpp
              ModelFiller.cpp
              module.cpp
              linemod_icp.cpp
)
set(INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
                 ${OpenCV_INCLUDE_DIRS}
)
set(LIBRARIES ${catkin_LIBRARIES}
              ${OpenCV_LIBRARIES}
)

if(LINEMOD_VIZ_PCD)
  set(SRC_FILES ${SRC_FILES} linemod_pointcloud.cpp)

  find_package(roscpp)
  set(INCLUDE_DIRS ${INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS})
  set(LIBRARIES ${LIBRARIES} ${roscpp_LIBRARIES})

  add_definitions("-DLINEMOD_VIZ_PCD=1")
endif()

include_directories(${INCLUDE_DIRS})
ectomodule(ecto_linemod DESTINATION ${PROJECT_NAME}/ecto_cells
                        INSTALL
                        ${SRC_FILES}
)

link_ecto(ecto_linemod ${LIBRARIES})
