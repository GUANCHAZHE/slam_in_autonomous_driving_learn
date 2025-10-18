add_executable(my_point_cloud_load_and_vis my_point_cloud_load_and_vis.cc)
target_link_libraries(my_point_cloud_load_and_vis
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        gflags)