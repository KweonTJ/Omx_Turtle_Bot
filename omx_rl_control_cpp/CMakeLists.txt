cmake_minimum_required(VERSION 3.8)
project(omx_rl_control_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# The Eigen actor runs directly in the 50 Hz control loop. Use the standard
# optimized CMake profile unless a developer explicitly selects another one.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

option(OMX_RL_ENABLE_TORCH "Enable the optional TorchScript policy backend" ON)
set(OMX_RL_TORCH_FOUND FALSE)
if(OMX_RL_ENABLE_TORCH)
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND AND NOT Torch_DIR)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c
        "import torch; print(torch.utils.cmake_prefix_path)"
      RESULT_VARIABLE OMX_RL_TORCH_DISCOVERY_RESULT
      OUTPUT_VARIABLE OMX_RL_TORCH_PREFIX
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
    if(OMX_RL_TORCH_DISCOVERY_RESULT EQUAL 0 AND OMX_RL_TORCH_PREFIX)
      list(APPEND CMAKE_PREFIX_PATH "${OMX_RL_TORCH_PREFIX}")
    endif()
  endif()
  find_package(Torch QUIET)
  if(Torch_FOUND)
    set(OMX_RL_TORCH_FOUND TRUE)
    separate_arguments(OMX_RL_TORCH_CXX_FLAGS NATIVE_COMMAND "${TORCH_CXX_FLAGS}")
    message(STATUS "omx_rl_control_cpp: TorchScript backend enabled")
  else()
    message(WARNING
      "omx_rl_control_cpp: LibTorch not found; building Eigen CPU fallback only")
  endif()
endif()

add_library(${PROJECT_NAME}_core
  src/policy_actor.cpp
  src/runtime_math.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)
if(OMX_RL_TORCH_FOUND)
  target_compile_definitions(${PROJECT_NAME}_core PRIVATE OMX_RL_HAS_TORCH=1)
  target_compile_options(${PROJECT_NAME}_core PRIVATE ${OMX_RL_TORCH_CXX_FLAGS})
  target_link_libraries(${PROJECT_NAME}_core ${TORCH_LIBRARIES})
endif()

add_executable(rl_control_cpp_node src/rl_control_cpp_node.cpp)
target_link_libraries(rl_control_cpp_node ${PROJECT_NAME}_core Eigen3::Eigen)
if(OMX_RL_TORCH_FOUND)
  target_compile_options(rl_control_cpp_node PRIVATE ${OMX_RL_TORCH_CXX_FLAGS})
  target_link_libraries(rl_control_cpp_node ${TORCH_LIBRARIES})
  set_property(TARGET rl_control_cpp_node PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()
ament_target_dependencies(rl_control_cpp_node
  ament_index_cpp
  rclcpp
  rclcpp_action
  action_msgs
  control_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
)

add_executable(policy_probe src/policy_probe.cpp)
target_link_libraries(policy_probe ${PROJECT_NAME}_core Eigen3::Eigen)
if(OMX_RL_TORCH_FOUND)
  target_compile_options(policy_probe PRIVATE ${OMX_RL_TORCH_CXX_FLAGS})
  target_link_libraries(policy_probe ${TORCH_LIBRARIES})
  set_property(TARGET policy_probe PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

install(TARGETS
  ${PROJECT_NAME}_core
  rl_control_cpp_node
  policy_probe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/export_sb3_actor.py tools/compare_cpp_policy.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_runtime_math test/test_runtime_math.cpp)
  if(TARGET test_runtime_math)
    target_link_libraries(test_runtime_math ${PROJECT_NAME}_core Eigen3::Eigen)
    target_include_directories(test_runtime_math PRIVATE include)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(Eigen3)
ament_package()
