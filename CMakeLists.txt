cmake_minimum_required(VERSION 3.16)

project(
    aerakia_flight_algorithms
    VERSION 0.1.0
    DESCRIPTION "Hardware-independent flight estimation algorithms and validation tools"
    LANGUAGES C
)

option(AERAKIA_BUILD_TESTS "Build host-side tests" ON)
option(AERAKIA_BUILD_VALIDATION "Build the PC validation runner" ON)

add_library(aerakia_algorithms STATIC
    src/eskf.c
    src/eskf_adapter.c
    src/eskf_math.c
    src/eskf_models.c
    src/mag_gate.c
    src/mahony.c
)
add_library(Aerakia::Algorithms ALIAS aerakia_algorithms)
target_include_directories(aerakia_algorithms
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_features(aerakia_algorithms PUBLIC c_std_99)

if(MSVC)
    target_compile_options(aerakia_algorithms PRIVATE /W4)
else()
    target_compile_options(aerakia_algorithms PRIVATE -Wall -Wextra)
    target_link_libraries(aerakia_algorithms PUBLIC m)
endif()

if(AERAKIA_BUILD_TESTS)
    enable_testing()

    add_executable(test_eskf tests/test_eskf.c)
    target_link_libraries(test_eskf PRIVATE Aerakia::Algorithms)
    add_test(NAME eskf_core COMMAND test_eskf)

    add_executable(test_eskf_models tests/test_eskf_models.c)
    target_include_directories(test_eskf_models PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(test_eskf_models PRIVATE Aerakia::Algorithms)
    add_test(NAME eskf_models COMMAND test_eskf_models)

    add_executable(test_mag_gate tests/test_mag_gate.c)
    target_link_libraries(test_mag_gate PRIVATE Aerakia::Algorithms)
    add_test(NAME magnetic_gate COMMAND test_mag_gate)

    add_executable(test_public_api tests/test_public_api.c)
    target_link_libraries(test_public_api PRIVATE Aerakia::Algorithms)
    add_test(NAME public_api COMMAND test_public_api)
endif()

if(AERAKIA_BUILD_VALIDATION)
    add_executable(aerakia_validation_runner validation/validation_runner.c)
    target_link_libraries(aerakia_validation_runner PRIVATE Aerakia::Algorithms)
endif()
