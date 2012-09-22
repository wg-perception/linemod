if (ROS_GROOVY_FOUND)
configure_file(__init__.py.plain.in
               ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/ecto_cells/__init__.py
               @ONLY
)

install(FILES ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/ecto_cells/__init__.py
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/ecto_cells
)
else()
configure_file(__init__.py.plain.in
               ${ECTO_PYTHON_BUILD_PATH}/${PROJECT_NAME}/ecto_cells/__init__.py
               @ONLY
)

install(FILES ${ECTO_PYTHON_BUILD_PATH}/${PROJECT_NAME}/ecto_cells/__init__.py
        DESTINATION ${ECTO_PYTHON_INSTALL_PATH}/${PROJECT_NAME}/ecto_cells
)
endif()
