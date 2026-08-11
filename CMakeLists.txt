cmake_minimum_required(VERSION 3.16)

project(
    aerakia_flight_algorithms
    VERSION 0.3.0
    DESCRIPTION "Hardware-independent flight estimation algorithms and validation tools"
    LANGUAGES C
)

if(NOT MSVC)
    # Enforce the advertised language contract instead of inheriting a newer compiler default.
    set(CMAKE_C_STANDARD 99)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    set(CMAKE_C_EXTENSIONS OFF)
endif()

option(AERAKIA_BUILD_TESTS "Build host-side tests" ON)
option(AERAKIA_BUILD_VALIDATION "Build the PC validation runner" ON)
option(AERAKIA_WARNINGS_AS_ERRORS "Treat warnings as build failures" ON)
option(AERAKIA_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
set(AERAKIA_ESKF_CORE_PRECISION "double" CACHE STRING
    "ESKF core scalar type: double (reviewed default) or float (evaluation only)")
set_property(CACHE AERAKIA_ESKF_CORE_PRECISION PROPERTY STRINGS double float)
if(NOT AERAKIA_ESKF_CORE_PRECISION STREQUAL "double"
   AND NOT AERAKIA_ESKF_CORE_PRECISION STREQUAL "float")
    message(FATAL_ERROR
        "AERAKIA_ESKF_CORE_PRECISION must be either 'double' or 'float'")
endif()

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
    src/barometer_supervisor.c
    src/eskf.c
    src/eskf_adapter.c
    src/eskf_math.c
    src/eskf_models.c
    src/mag_gate.c
    src/mahony.c
)
add_library(Aerakia::Algorithms ALIAS aerakia_algorithms)
set_target_properties(aerakia_algorithms PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)
target_include_directories(aerakia_algorithms
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_features(aerakia_algorithms PUBLIC c_std_99)
if(AERAKIA_ESKF_CORE_PRECISION STREQUAL "float")
    # PUBLIC is intentional: applications allocate ESKF_Handle and must use
    # the same scalar layout as this library.
    target_compile_definitions(aerakia_algorithms PUBLIC AERAKIA_ESKF_CORE_USE_FLOAT=1)
endif()

if(NOT MSVC)
    target_link_libraries(aerakia_algorithms PUBLIC m)
endif()

if(AERAKIA_BUILD_TESTS)
    enable_testing()

    add_executable(test_eskf tests/test_eskf.c)
    target_link_libraries(test_eskf PRIVATE Aerakia::Algorithms)
    add_test(NAME eskf_core COMMAND test_eskf)

    if(AERAKIA_ESKF_CORE_PRECISION STREQUAL "double")
        # This target is a double-precision numerical oracle (finite differences
        # and a high-order Q integration), not an implementation-agnostic API
        # test. The float candidate is checked against a double replay instead.
        add_executable(test_eskf_models tests/test_eskf_models.c)
        target_include_directories(test_eskf_models PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_link_libraries(test_eskf_models PRIVATE Aerakia::Algorithms)
        add_test(NAME eskf_models COMMAND test_eskf_models)
    endif()

    add_executable(test_mag_gate tests/test_mag_gate.c)
    target_link_libraries(test_mag_gate PRIVATE Aerakia::Algorithms)
    add_test(NAME magnetic_gate COMMAND test_mag_gate)

    add_executable(test_barometer_supervisor tests/test_barometer_supervisor.c)
    target_link_libraries(test_barometer_supervisor PRIVATE Aerakia::Algorithms)
    add_test(NAME barometer_supervisor COMMAND test_barometer_supervisor)

    add_executable(test_public_api tests/test_public_api.c)
    target_link_libraries(test_public_api PRIVATE Aerakia::Algorithms)
    add_test(NAME public_api COMMAND test_public_api)

    add_executable(test_version tests/test_version.c)
    target_link_libraries(test_version PRIVATE Aerakia::Algorithms)
    target_compile_definitions(test_version PRIVATE
        AERAKIA_CMAKE_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
        AERAKIA_CMAKE_VERSION_MINOR=${PROJECT_VERSION_MINOR}
        AERAKIA_CMAKE_VERSION_PATCH=${PROJECT_VERSION_PATCH}
        AERAKIA_CMAKE_VERSION_STRING="${PROJECT_VERSION}"
    )
    add_test(NAME public_version COMMAND test_version)

    add_executable(test_validation_covariance tests/test_validation_covariance.c)
    target_link_libraries(test_validation_covariance PRIVATE Aerakia::Algorithms)
    add_test(NAME validation_covariance COMMAND test_validation_covariance)

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

    add_executable(aerakia_delayed_gnss_reprop_oracle
        validation/delayed_gnss_reprop_oracle.c
    )
    target_link_libraries(aerakia_delayed_gnss_reprop_oracle PRIVATE Aerakia::Algorithms)
    add_test(
        NAME delayed_gnss_reprop_oracle
        COMMAND aerakia_delayed_gnss_reprop_oracle --rate-hz 400 --delay-ms 150
    )
    add_test(
        NAME delayed_gnss_reprop_sequential_oracle
        COMMAND aerakia_delayed_gnss_reprop_oracle
            --rate-hz 400 --delay-ms 150 --scenario sequential
    )
    add_test(
        NAME delayed_gnss_reprop_overlapping_oracle
        COMMAND aerakia_delayed_gnss_reprop_oracle
            --rate-hz 400 --delay-ms 150 --scenario overlap
    )

    add_executable(aerakia_fixed_lag_replay_contract
        validation/fixed_lag_replay_contract.c
    )
    target_include_directories(aerakia_fixed_lag_replay_contract
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(aerakia_fixed_lag_replay_contract PRIVATE Aerakia::Algorithms)
    add_test(NAME fixed_lag_replay_contract COMMAND aerakia_fixed_lag_replay_contract)

    add_executable(aerakia_fixed_lag_covariance_composition
        validation/fixed_lag_covariance_composition_oracle.c
    )
    target_include_directories(aerakia_fixed_lag_covariance_composition
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(aerakia_fixed_lag_covariance_composition PRIVATE Aerakia::Algorithms)
    add_test(
        NAME fixed_lag_covariance_composition
        COMMAND aerakia_fixed_lag_covariance_composition
    )

    add_executable(aerakia_fixed_lag_measurement_cross_covariance
        validation/fixed_lag_measurement_cross_covariance_oracle.c
    )
    target_include_directories(aerakia_fixed_lag_measurement_cross_covariance
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(aerakia_fixed_lag_measurement_cross_covariance PRIVATE Aerakia::Algorithms)
    add_test(
        NAME fixed_lag_measurement_cross_covariance
        COMMAND aerakia_fixed_lag_measurement_cross_covariance
    )
endif()

if(AERAKIA_BUILD_VALIDATION)
    add_executable(aerakia_validation_runner validation/validation_runner.c)
    target_include_directories(aerakia_validation_runner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(aerakia_validation_runner PRIVATE Aerakia::Algorithms)
endif()
