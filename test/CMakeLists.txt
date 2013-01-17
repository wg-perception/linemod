# LINE-MOD tests
find_package(object_recognition_core REQUIRED)
enable_testing()
object_recognition_core_config_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/config_detection.linemod)
object_recognition_core_config_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/config_training.linemod)
