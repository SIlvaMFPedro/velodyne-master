add_library(velodyne_rawdata rawdata.cpp calibration.cpp)
target_link_libraries(velodyne_rawdata
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES})
install(TARGETS velodyne_rawdata
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})