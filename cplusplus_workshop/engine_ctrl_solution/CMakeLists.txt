cmake_minimum_required(VERSION 3.5)
project(engine_ctrl_solution)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(GTest REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(include
    ${GTEST_INCLUDE_DIR})
ament_export_include_directories(include)

add_executable(engine_ctrl_solution
  src/engine_ctrl_solution.cpp
  src/engine_simulator.cpp
  src/engine_timer.cpp
  src/main.cpp
)


ament_target_dependencies(engine_ctrl_solution rclcpp std_msgs)

target_link_libraries(engine_ctrl_solution
    )

install(TARGETS
  engine_ctrl_solution
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


# test
FILE(GLOB TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/unittests/*.cpp)

add_executable(gtest_${PROJECT_NAME}
    ${TEST_SOURCES}
    src/engine_timer.cpp
)

target_link_libraries(gtest_${PROJECT_NAME}
    gtest gtest_main pthread
    ${JSONCPP_LIBRARIES}
)
add_test(NAME gtest_${PROJECT_NAME}
    COMMAND gtest_${PROJECT_NAME}
)
