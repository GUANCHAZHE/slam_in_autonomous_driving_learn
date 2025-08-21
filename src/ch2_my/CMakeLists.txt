add_executable(my_motion my_motion.cc)
target_link_libraries(my_motion
    ${PROJECT_NAME}.common
    ${PROJECT_NAME}.tools
    )