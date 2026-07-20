cmake_minimum_required(VERSION 3.16)
project(parking_trajectory_sim LANGUAGES CXX)

include(FetchContent)

find_package(Eigen3 3.4 CONFIG QUIET)
if(NOT Eigen3_FOUND)
    set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.1
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(eigen)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(parking_trajectory_sim
    src/main.cpp
    src/control/discrete_lqr.cpp
    src/actuators/steering_actuator.cpp
    src/trajectory/trajectory_sampler.cpp
    src/trajectory/reference_searcher.cpp
    src/trajectory/path_generator.cpp
    src/trajectory/velocity_profile.cpp
    src/trajectory/sample_trajectories.cpp
    src/controllers/feedforward_controller.cpp
    src/controllers/yaw_feedback_controller.cpp
    src/controllers/lateral_feedback_controller.cpp
    src/controllers/pure_pursuit_controller.cpp
    src/controllers/stanley_controller.cpp
    src/controllers/lqr_controller.cpp
    src/controllers/linear_mpc_controller.cpp
    src/models/kinematic_bicycle.cpp
    src/models/linear_error_model.cpp
    src/simulation/parking_simulator.cpp
    src/scenario/parking_scenario.cpp
    src/scenario/sample_parking_scenario.cpp
    src/controllers/longitudinal/velocity_stop_controller.cpp
    src/coordination/longitudinal_lateral_coordinator.cpp
    src/io/csv_writer.cpp
)

target_include_directories(parking_trajectory_sim PRIVATE include)
target_link_libraries(parking_trajectory_sim PRIVATE Eigen3::Eigen)

if(MINGW)
    target_compile_options(parking_trajectory_sim PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)

if(BUILD_TESTING)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(
            NAME documentation_figures_test
            COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_SOURCE_DIR}/python/doc_figures/test_doc_figures.py)
        add_test(
            NAME traceability_test
            COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_SOURCE_DIR}/python/check_traceability.py)
    endif()

    add_executable(kinematic_bicycle_test
        tests/kinematic_bicycle_test.cpp
        src/models/kinematic_bicycle.cpp
    )
    target_include_directories(kinematic_bicycle_test PRIVATE include)

    if(MINGW)
        target_compile_options(kinematic_bicycle_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME kinematic_bicycle_test COMMAND kinematic_bicycle_test)

    add_executable(parking_simulator_test
        tests/parking_simulator_test.cpp
        src/simulation/parking_simulator.cpp
        src/scenario/parking_scenario.cpp
        src/controllers/longitudinal/velocity_stop_controller.cpp
        src/coordination/longitudinal_lateral_coordinator.cpp
        src/actuators/steering_actuator.cpp
        src/trajectory/trajectory_sampler.cpp
        src/trajectory/reference_searcher.cpp
        src/controllers/feedforward_controller.cpp
        src/models/kinematic_bicycle.cpp
    )
    target_include_directories(parking_simulator_test PRIVATE include)

    if(MINGW)
        target_compile_options(parking_simulator_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME parking_simulator_test COMMAND parking_simulator_test)

    add_executable(yaw_feedback_controller_test
        tests/yaw_feedback_controller_test.cpp
        src/controllers/yaw_feedback_controller.cpp
    )
    target_include_directories(yaw_feedback_controller_test PRIVATE include)

    if(MINGW)
        target_compile_options(yaw_feedback_controller_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(
        NAME yaw_feedback_controller_test
        COMMAND yaw_feedback_controller_test)

    add_executable(lateral_feedback_controller_test
        tests/lateral_feedback_controller_test.cpp
        src/controllers/lateral_feedback_controller.cpp
        src/simulation/parking_simulator.cpp
        src/scenario/parking_scenario.cpp
        src/controllers/longitudinal/velocity_stop_controller.cpp
        src/coordination/longitudinal_lateral_coordinator.cpp
        src/actuators/steering_actuator.cpp
        src/trajectory/trajectory_sampler.cpp
        src/trajectory/reference_searcher.cpp
        src/models/kinematic_bicycle.cpp
    )
    target_include_directories(lateral_feedback_controller_test PRIVATE include)

    if(MINGW)
        target_compile_options(lateral_feedback_controller_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(
        NAME lateral_feedback_controller_test
        COMMAND lateral_feedback_controller_test)

    add_executable(reference_searcher_test
        tests/reference_searcher_test.cpp
        src/trajectory/reference_searcher.cpp
    )
    target_include_directories(reference_searcher_test PRIVATE include)

    if(MINGW)
        target_compile_options(reference_searcher_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME reference_searcher_test COMMAND reference_searcher_test)

    add_executable(path_tracking_controllers_test
        tests/path_tracking_controllers_test.cpp
        src/controllers/pure_pursuit_controller.cpp
        src/controllers/stanley_controller.cpp
        src/trajectory/reference_searcher.cpp
        src/simulation/parking_simulator.cpp
        src/scenario/parking_scenario.cpp
        src/controllers/longitudinal/velocity_stop_controller.cpp
        src/coordination/longitudinal_lateral_coordinator.cpp
        src/actuators/steering_actuator.cpp
        src/trajectory/trajectory_sampler.cpp
        src/models/kinematic_bicycle.cpp
    )
    target_include_directories(path_tracking_controllers_test PRIVATE include)

    if(MINGW)
        target_compile_options(path_tracking_controllers_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(
        NAME path_tracking_controllers_test
        COMMAND path_tracking_controllers_test)

    add_executable(steering_actuator_test
        tests/steering_actuator_test.cpp
        src/actuators/steering_actuator.cpp
    )
    target_include_directories(steering_actuator_test PRIVATE include)

    if(MINGW)
        target_compile_options(steering_actuator_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME steering_actuator_test COMMAND steering_actuator_test)

    add_executable(lqr_controller_test
        tests/lqr_controller_test.cpp
        src/control/discrete_lqr.cpp
        src/controllers/lqr_controller.cpp
        src/models/linear_error_model.cpp
        src/simulation/parking_simulator.cpp
        src/scenario/parking_scenario.cpp
        src/controllers/longitudinal/velocity_stop_controller.cpp
        src/coordination/longitudinal_lateral_coordinator.cpp
        src/actuators/steering_actuator.cpp
        src/trajectory/trajectory_sampler.cpp
        src/trajectory/reference_searcher.cpp
        src/models/kinematic_bicycle.cpp
    )
    target_include_directories(lqr_controller_test PRIVATE include)
    target_link_libraries(lqr_controller_test PRIVATE Eigen3::Eigen)

    if(MINGW)
        target_compile_options(lqr_controller_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME lqr_controller_test COMMAND lqr_controller_test)

    add_executable(trajectory_generation_test
        tests/trajectory_generation_test.cpp
        src/trajectory/path_generator.cpp
        src/trajectory/velocity_profile.cpp
    )
    target_include_directories(trajectory_generation_test PRIVATE include)

    if(MINGW)
        target_compile_options(trajectory_generation_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME trajectory_generation_test COMMAND trajectory_generation_test)

    add_executable(parking_state_machine_test
        tests/parking_state_machine_test.cpp
        src/scenario/parking_scenario.cpp
        src/scenario/sample_parking_scenario.cpp
        src/trajectory/sample_trajectories.cpp
        src/trajectory/path_generator.cpp
        src/trajectory/velocity_profile.cpp
    )
    target_include_directories(parking_state_machine_test PRIVATE include)
    if(MINGW)
        target_compile_options(parking_state_machine_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME parking_state_machine_test COMMAND parking_state_machine_test)

    add_executable(longitudinal_control_test
        tests/longitudinal_control_test.cpp
        src/controllers/longitudinal/velocity_stop_controller.cpp
        src/coordination/longitudinal_lateral_coordinator.cpp
        src/models/kinematic_bicycle.cpp
    )
    target_include_directories(longitudinal_control_test PRIVATE include)
    if(MINGW)
        target_compile_options(longitudinal_control_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME longitudinal_control_test COMMAND longitudinal_control_test)

    add_executable(linear_mpc_controller_test
        tests/linear_mpc_controller_test.cpp
        src/controllers/linear_mpc_controller.cpp
        src/models/linear_error_model.cpp
    )
    target_include_directories(linear_mpc_controller_test PRIVATE include)
    target_link_libraries(linear_mpc_controller_test PRIVATE Eigen3::Eigen)
    if(MINGW)
        target_compile_options(linear_mpc_controller_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME linear_mpc_controller_test COMMAND linear_mpc_controller_test)
endif()
