# Compile the ecto 
ectomodule(ecto_sample_linemod DESTINATION ${PROJECT_NAME}/ecto_cells
           linemod.cpp
           module.cpp
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

link_ecto(ecto_sample_linemod ${catkin_LIBRARIES}
                              ${OpenCV_LIBRARIES}
)
