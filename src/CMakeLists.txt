find_package(object_recognition_core REQUIRED)
find_package(OpenCV REQUIRED)
find_pcl_package()

# Compile the ecto 
ectomodule(ecto_linemod
           db_linemod.cpp
           linemod_detect.cpp
           linemod_train.cpp
           ModelFiller.cpp
           module.cpp
)

include_directories(SYSTEM
                    ${OpenCV_INCLUDE_DIRS}
                    ${object_recognition_core_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
)

link_ecto(ecto_linemod
          ${object_recognition_core_LIBRARIES}
          ${OpenCV_LIBRARIES}
          ${PCL_LIBRARIES}
)

install_ecto_module(ecto_linemod)
