cmake_minimum_required(VERSION 3.16)

project(
    aerakia_flight_algorithms
    VERSION 0.1.0
    DESCRIPTION "Hardware-independent flight estimation algorithms and validation tools"
    LANGUAGES C
)

option(AERAKIA_BUILD_TESTS "Build host-side tests" ON)
option(AERAKIA_BUILD_VALIDATION "Build the PC validation runner" ON)
option(AERAKIA_WARNINGS_AS_ERRORS "Treat warnings as build failures" ON)
option(AERAKIA_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

if(MSVC)
    add_compile_options(/W4)
    if(AERAKIA_WARNINGS_AS_ERRORS)
        add_compile_options(/WX)
    endif()
else()
    add_compile_options(-Wall -Wextra -pedantic)
    if(AERAKIA_WARNINGS_AS_ERRORS)
        add_compile_options(-Werror)
    endif()
    if(AERAKIA_ENABLE_SANITIZERS)
        add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    endif()
endif()

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

if(NOT MSVC)
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

    add_executable(aerakia_input_integrity_campaign validation/input_integrity_campaign.c)
    target_link_libraries(aerakia_input_integrity_campaign PRIVATE Aerakia::Algorithms)
    add_test(NAME input_integrity_campaign COMMAND aerakia_input_integrity_campaign)

    add_executable(aerakia_estimator_supervisor_contract
        validation/estimator_supervisor_contract.c
    )
    target_link_libraries(aerakia_estimator_supervisor_contract PRIVATE Aerakia::Algorithms)
    add_test(NAME estimator_supervisor_contract COMMAND aerakia_estimator_supervisor_contract)

    add_executable(aerakia_fcone_adapter_contract validation/fcone_adapter_contract.c)
    target_link_libraries(aerakia_fcone_adapter_contract PRIVATE Aerakia::Algorithms)
    add_test(NAME fcone_adapter_contract COMMAND aerakia_fcone_adapter_contract)
endif()

if(AERAKIA_BUILD_VALIDATION)
    add_executable(aerakia_validation_runner validation/validation_runner.c)
    target_link_libraries(aerakia_validation_runner PRIVATE Aerakia::Algorithms)
endif()
