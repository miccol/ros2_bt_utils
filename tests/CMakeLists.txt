# ###################################################
# TESTS                                            #
# ###################################################
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

set(ROS2_BT_UTILS_TESTS
    main.cpp
    gtest_condition_topic_subscriber.cpp
    gtest_condition_ros_service_client.cpp
    gtest_action_ros_action_client.cpp
    gtest_action_ros_service_client.cpp
)

set(TEST_DEPENDENCIES
    std_msgs
    example_interfaces
)

if(ament_cmake_FOUND AND BUILD_TESTING)
    ament_add_gtest_executable(${LIBRARY_NAME}_test ${ROS2_BT_UTILS_TESTS})
    ament_target_dependencies(${LIBRARY_NAME}_test ${TEST_DEPENDENCIES} ${DEPENDENCIES})

    target_link_libraries(${LIBRARY_NAME}_test
        ${LIBRARY_NAME}
        ${ament_LIBRARIES}
    )
    target_include_directories(${LIBRARY_NAME}_test PRIVATE gtest/include)
    include_directories($<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty>)
endif()

# install(
# TARGETS
# ${LIBRARY_NAME}_test
# ARCHIVE DESTINATION lib
# LIBRARY DESTINATION lib
# RUNTIME DESTINATION lib/${PROJECT_NAME}
# )
