find_package(object_recognition_core REQUIRED)
find_package(OpenCV REQUIRED)

# Compile the ecto 
ectomodule(ecto_sample_linemod DESTINATION ${PROJECT_NAME}/ecto_cells
           linemod.cpp
           module.cpp
)

include_directories(SYSTEM
                    ${OpenCV_INCLUDE_DIRS}
                    ${object_recognition_core_INCLUDE_DIRS}
)

link_ecto(ecto_sample_linemod
          ${object_recognition_core_LIBRARIES}
          ${OpenCV_LIBRARIES}
)
