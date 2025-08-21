add_executable(my_run_imu_integration
        my_run_imu_integration.cc)

target_link_libraries(my_run_imu_integration
        glog gflags ${PROJECT_NAME}.common
        )
        # glog     log文件系统
        # gflags   用于命令行参数解析
        # ${PROJECT_NAME}.common 其他通用的库


# target_link_libraries(${PROJECT_NAME}.ch3_my
#         glog gflags ${PROJECT_NAME}.common
#         )