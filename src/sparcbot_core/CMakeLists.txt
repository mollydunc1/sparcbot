cmake_minimum_required(VERSION 3.16)
project(sparcbot_core LANGUAGES CXX)

# sparcbot_core is plain C++ with no ROS dependency. It builds two ways:
#   * inside the ROS 2 workspace:   colcon build --symlink-install
#   * standalone, as before:        mkdir build && cd build && cmake .. && make -j4
find_package(ament_cmake QUIET)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_library(GPIOD_LIBRARY gpiod REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
    set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
    set(YAML_CPP_TARGET yaml-cpp)
endif()

# The single parameter file shared with the ROS 2 nodes.
# Tests/tools read it at runtime; override with the SPARCBOT_CONFIG env variable.
set(SPARCBOT_DEFAULT_CONFIG
    "${CMAKE_CURRENT_SOURCE_DIR}/../sparcbot_bringup/config/sparcbot.yaml"
    CACHE FILEPATH "Default SPARCBot parameter file")
get_filename_component(SPARCBOT_DEFAULT_CONFIG "${SPARCBOT_DEFAULT_CONFIG}" ABSOLUTE)

# --------------------------------------------------------------------------
# Reusable SPARCBot library
# --------------------------------------------------------------------------

set(SPARCBOT_SOURCES
    src/config/robot_config.cpp
    src/hardware/motor.cpp
    src/estimation/wheel_state_estimator.cpp
    src/estimation/odometry.cpp
    src/kinematics/differential_drive_kinematics.cpp
)


add_library(sparcbot_core ${SPARCBOT_SOURCES})
add_library(sparcbot_core::sparcbot_core ALIAS sparcbot_core)

set_target_properties(sparcbot_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(sparcbot_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(sparcbot_core
    PUBLIC
        ${GPIOD_LIBRARY}
        Threads::Threads
    PRIVATE
        ${YAML_CPP_TARGET}
)

target_compile_definitions(sparcbot_core PRIVATE
    SPARCBOT_DEFAULT_CONFIG="${SPARCBOT_DEFAULT_CONFIG}"
)

target_compile_options(sparcbot_core PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

# --------------------------------------------------------------------------
# Bench tests and tools: plain executables that drive the hardware directly.
# Do not run them while the ROS driver node is running (both need the GPIO).
# --------------------------------------------------------------------------

set(SPARCBOT_EXECUTABLES "")

function(add_sparcbot_executable target source)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${source}")
        add_executable(${target} ${source})
        target_link_libraries(${target} PRIVATE sparcbot_core)
        set(SPARCBOT_EXECUTABLES ${SPARCBOT_EXECUTABLES} ${target} PARENT_SCOPE)
    endif()
endfunction()

# Tests
add_sparcbot_executable(as5600_test tests/as5600_test.cpp)
add_sparcbot_executable(differential_drive_kinematics_test tests/differential_drive_kinematics_test.cpp)
add_sparcbot_executable(dual_motor_test tests/dual_motor_test.cpp)
add_sparcbot_executable(wheel_state_test tests/wheel_state_test.cpp)

# Characterization / diagnostics tools
add_sparcbot_executable(keyboard_teleop tools/keyboard_teleop.cpp)
add_sparcbot_executable(dual_motor_characterization tools/dual_motor_characterization.cpp)
add_sparcbot_executable(robot_startup_test tools/robot_startup_test.cpp)

# --------------------------------------------------------------------------
# ROS 2 / ament install + export (only when built with colcon)
# --------------------------------------------------------------------------

if(ament_cmake_FOUND)
    install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

    install(TARGETS sparcbot_core
        EXPORT export_sparcbot_core
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/${PROJECT_NAME}
    )

    # `ros2 run sparcbot_core <tool>` works for every bench test / tool.
    install(TARGETS ${SPARCBOT_EXECUTABLES} DESTINATION lib/${PROJECT_NAME})

    ament_export_targets(export_sparcbot_core HAS_LIBRARY_TARGET)
    ament_export_dependencies(yaml-cpp Threads)
    ament_package()
endif()
