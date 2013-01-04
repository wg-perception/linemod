find_package(OpenCV REQUIRED)

# Compile the ecto 
ectomodule(ecto_linemod DESTINATION ${PROJECT_NAME}/ecto_cells
                        INSTALL
           db_linemod.cpp
           linemod_detect.cpp
           linemod_train.cpp
           ModelFiller.cpp
           module.cpp
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

link_ecto(ecto_linemod ${catkin_LIBRARIES}
                       ${OpenCV_LIBRARIES}
)
