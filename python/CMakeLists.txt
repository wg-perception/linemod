configure_file(__init__.py.plain.in
               ${ECTO_PYTHON_BUILD_PATH}/${PROJECT_NAME}/ecto_cells/__init__.py
               @ONLY
)

install(FILES ${ECTO_PYTHON_BUILD_PATH}/${PROJECT_NAME}/ecto_cells/__init__.py
        DESTINATION ${ECTO_PYTHON_INSTALL_PATH}/${PROJECT_NAME}/ecto_cells
)
