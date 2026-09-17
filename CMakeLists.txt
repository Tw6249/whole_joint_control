cmake_minimum_required(VERSION 3.16)
project(unitree_h1_direct_interface LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(H1IF_BUILD_UNITREE "Build the Unitree SDK2 runtime target" OFF)
option(H1IF_BUILD_TORCH_POLICY "Build TorchScript online policy support" OFF)
set(H1IF_LIBTORCH_DIR
    ""
    CACHE PATH "Path to the LibTorch installation")

find_package(Threads REQUIRED)

add_library(h1if_core INTERFACE)
target_include_directories(h1if_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(h1_mock_closed_loop
    src/mock_closed_loop.cpp
)
target_link_libraries(h1_mock_closed_loop PRIVATE h1if_core Threads::Threads)

add_executable(h1_controller_stepper
    src/controller_stepper.cpp
)
target_link_libraries(h1_controller_stepper PRIVATE h1if_core Threads::Threads)

add_executable(h1_safety_tests
    tests/test_safety.cpp
)
target_link_libraries(h1_safety_tests PRIVATE h1if_core)

enable_testing()
add_test(NAME h1_safety_tests COMMAND h1_safety_tests)
add_test(NAME h1_mock_closed_loop COMMAND h1_mock_closed_loop ${CMAKE_CURRENT_SOURCE_DIR}/config/simulation/h1_full_body_mujoco_fit.yaml 1.0)

if(H1IF_BUILD_TORCH_POLICY)
    if(EXISTS "${H1IF_LIBTORCH_DIR}/share/cmake/Torch/TorchConfig.cmake")
        list(APPEND CMAKE_PREFIX_PATH "${H1IF_LIBTORCH_DIR}")
    endif()
    find_package(Torch REQUIRED)

    function(h1if_enable_torch_policy target_name)
        target_compile_definitions(${target_name} PRIVATE H1IF_ENABLE_TORCH_POLICY=1)
        target_link_libraries(${target_name} PRIVATE ${TORCH_LIBRARIES})
        target_compile_options(${target_name} PRIVATE ${TORCH_CXX_FLAGS})
        set_target_properties(${target_name} PROPERTIES
            BUILD_RPATH "${H1IF_LIBTORCH_DIR}/lib"
            INSTALL_RPATH "${H1IF_LIBTORCH_DIR}/lib"
        )
    endfunction()

    h1if_enable_torch_policy(h1_controller_stepper)

endif()

if(H1IF_BUILD_UNITREE)
    find_package(unitree_sdk2 REQUIRED)

    add_executable(h1_direct
        src/main_h1_direct.cpp
    )
    target_link_libraries(h1_direct PRIVATE h1if_core unitree_sdk2 Threads::Threads)
    if(H1IF_BUILD_TORCH_POLICY)
        h1if_enable_torch_policy(h1_direct)
    endif()

    add_executable(h1_knee_state
        src/subscribe_knee_state.cpp
    )
    target_link_libraries(h1_knee_state PRIVATE h1if_core unitree_sdk2 Threads::Threads)

    add_executable(h1_torque_logger
        src/log_joint_torques.cpp
    )
    target_link_libraries(h1_torque_logger PRIVATE h1if_core unitree_sdk2 Threads::Threads)

    add_executable(h1_loco_cli
        src/h1_loco_cli.cpp
    )
    target_link_libraries(h1_loco_cli PRIVATE unitree_sdk2 Threads::Threads)

    add_executable(h1_knee_pid
        src/main_h1_knee_pid.cpp
    )
    target_link_libraries(h1_knee_pid PRIVATE h1if_core unitree_sdk2 Threads::Threads)
endif()
