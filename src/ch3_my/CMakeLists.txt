add_executable(my_run_imu_integration
        my_run_imu_integration.cc)
target_link_libraries(my_run_imu_integration
        glog gflags ${PROJECT_NAME}.common
        )
        # glog     log文件系统
        # gflags   用于命令行参数解析
        # ${PROJECT_NAME}.common 其他通用的库

add_executable(my_run_eskf_gins my_run_eskf_gins)
target_link_libraries(my_run_eskf_gins
                glog gflags ${PROJECT_NAME}.common ${PROJECT_NAME}.ch3_my                
)

add_library(${PROJECT_NAME}.ch3_my

                my_static_imu_init.cc
                utm_convert.cc 
                ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/utm.cc
                ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/tranmerc.cc
)

# target_link_libraries(${PROJECT_NAME}.ch3_my
#         glog gflags ${PROJECT_NAME}.common
#         )