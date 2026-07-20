# SPDX-License-Identifier: Apache-2.0
#
# Standalone, read-only build of the official PX4 ecl_EKF core needed by M0.
# Configuring PX4's root project pulls unrelated flight-stack submodules.  This
# project instead compiles the frozen source files that PX4's own ecl_EKF
# target selects for the declared GNSS/barometer/heading-magnetometer profile.

cmake_minimum_required(VERSION 3.16)
project(aerakia_px4_bias_ab_host LANGUAGES CXX)

if(NOT DEFINED AERAKIA_PX4_SOURCE OR NOT IS_DIRECTORY "${AERAKIA_PX4_SOURCE}")
    message(FATAL_ERROR "AERAKIA_PX4_SOURCE must name an official PX4 checkout")
endif()
if(NOT DEFINED AERAKIA_PX4_BIAS_AB_RUNNER_SOURCE
   OR NOT EXISTS "${AERAKIA_PX4_BIAS_AB_RUNNER_SOURCE}")
    message(FATAL_ERROR "AERAKIA_PX4_BIAS_AB_RUNNER_SOURCE is required")
endif()

set(PX4 "${AERAKIA_PX4_SOURCE}")
set(EKF "${PX4}/src/modules/ekf2/EKF")

# PX4's root build generates this board header. The standalone host runner has
# no board peripherals; its complete EKF feature selection is declared below.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/px4_boardconfig.h"
    "#pragma once\n#define CONFIG_PLATFORM_POSIX 1\n#define CONFIG_BOARD_ROOT_PATH \".\"\n")
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# The EKF publishes internal aid-source diagnostics through three generated
# uORB structs. Generate only those headers with PX4's unmodified generator;
# the full PX4 message graph is not required for this standalone library.
set(UORB_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/uORB/topics")
set(UORB_MESSAGES
    "${PX4}/msg/EstimatorAidSource1d.msg"
    "${PX4}/msg/EstimatorAidSource2d.msg"
    "${PX4}/msg/EstimatorAidSource3d.msg"
)
set(UORB_HEADERS
    "${UORB_OUTPUT_DIR}/estimator_aid_source1d.h"
    "${UORB_OUTPUT_DIR}/estimator_aid_source2d.h"
    "${UORB_OUTPUT_DIR}/estimator_aid_source3d.h"
)
add_custom_command(
    OUTPUT ${UORB_HEADERS}
    COMMAND ${Python3_EXECUTABLE} "${PX4}/Tools/msg/px_generate_uorb_topic_files.py"
        --headers -f ${UORB_MESSAGES}
        -i "${PX4}/msg" "${PX4}/msg/versioned"
        -o "${UORB_OUTPUT_DIR}"
        -e "${PX4}/Tools/msg/templates/uorb"
    DEPENDS ${UORB_MESSAGES}
        "${PX4}/Tools/msg/px_generate_uorb_topic_files.py"
        "${PX4}/Tools/msg/px_generate_uorb_topic_helper.py"
        "${PX4}/Tools/msg/templates/uorb/msg.h.em"
    WORKING_DIRECTORY "${PX4}/msg"
    VERBATIM
)
add_custom_target(aerakia_px4_uorb_headers DEPENDS ${UORB_HEADERS})

add_library(aerakia_px4_ecl_ekf STATIC
    "${EKF}/control.cpp"
    "${EKF}/covariance.cpp"
    "${EKF}/ekf.cpp"
    "${EKF}/ekf_helper.cpp"
    "${EKF}/estimator_interface.cpp"
    "${EKF}/height_control.cpp"
    "${EKF}/velocity_fusion.cpp"
    "${EKF}/position_fusion.cpp"
    "${EKF}/yaw_fusion.cpp"
    "${EKF}/imu_down_sampler/imu_down_sampler.cpp"
    "${EKF}/aid_sources/fake_height_control.cpp"
    "${EKF}/aid_sources/fake_pos_control.cpp"
    "${EKF}/aid_sources/ZeroGyroUpdate.cpp"
    "${EKF}/aid_sources/ZeroVelocityUpdate.cpp"
    "${EKF}/aid_sources/gnss/gnss_checks.cpp"
    "${EKF}/aid_sources/gnss/gnss_height_control.cpp"
    "${EKF}/aid_sources/gnss/gps_control.cpp"
    "${EKF}/yaw_estimator/EKFGSF_yaw.cpp"
    "${EKF}/aid_sources/barometer/baro_height_control.cpp"
    "${EKF}/aid_sources/magnetometer/mag_control.cpp"
    "${EKF}/aid_sources/magnetometer/mag_fusion.cpp"
    "${EKF}/bias_estimator/bias_estimator.cpp"
    "${EKF}/output_predictor/output_predictor.cpp"
    "${PX4}/src/lib/geo/geo.cpp"
    "${PX4}/src/lib/lat_lon_alt/lat_lon_alt.cpp"
    "${PX4}/src/lib/world_magnetic_model/geo_mag_declination.cpp"
    "${PX4}/src/lib/mathlib/math/test/test.cpp"
)
add_dependencies(aerakia_px4_ecl_ekf aerakia_px4_uorb_headers)

target_compile_features(aerakia_px4_ecl_ekf PUBLIC cxx_std_17)
# These feature switches alter official ecl_EKF class layouts.  They must be
# visible to the external runner too, otherwise its `Ekf` allocation has a
# different ABI than the linked library.
target_compile_definitions(aerakia_px4_ecl_ekf PUBLIC
    __PX4_POSIX
    CONFIG_EKF2_GNSS
    CONFIG_EKF2_BAROMETER
    CONFIG_EKF2_MAGNETOMETER
)
target_include_directories(aerakia_px4_ecl_ekf PUBLIC
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${PX4}/platforms/common/include"
    "${PX4}/platforms/common"
    "${PX4}/platforms/posix/include"
    "${PX4}/src/include"
    "${PX4}/src"
    "${PX4}/src/lib"
    "${PX4}/src/lib/matrix"
    "${PX4}/src/modules"
    "${PX4}/src/modules/ekf2"
    "${EKF}"
    "${EKF}/python"
)
# PX4's root build injects this header for every translation unit.  It owns
# the official __EXPORT visibility definition; preserve that mechanism in the
# standalone build instead of declaring a local substitute.
target_compile_options(aerakia_px4_ecl_ekf PRIVATE
    -fno-associative-math
    -include visibility.h
)

add_executable(aerakia_px4_bias_ab_runner
    "${AERAKIA_PX4_BIAS_AB_RUNNER_SOURCE}"
)
target_compile_features(aerakia_px4_bias_ab_runner PRIVATE cxx_std_17)
target_compile_options(aerakia_px4_bias_ab_runner PRIVATE -include visibility.h)
target_link_libraries(aerakia_px4_bias_ab_runner PRIVATE aerakia_px4_ecl_ekf)
