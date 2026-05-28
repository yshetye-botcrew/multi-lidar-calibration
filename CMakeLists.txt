cmake_minimum_required(VERSION 3.14)
project(multi-lidar-calibration LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

# FastDDS installation path
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/fastdds")
# message_manager (for DDS point cloud subscription)
list(APPEND CMAKE_PREFIX_PATH "/opt/botcrew" "$ENV{CMAKE_PREFIX_PATH}")

find_package(message_manager REQUIRED)
find_package(fastcdr CONFIG REQUIRED)
find_package(fastdds CONFIG REQUIRED)
find_package(PCL REQUIRED)

# nlohmann_json (header-only)
find_package(nlohmann_json QUIET)
if(NOT TARGET nlohmann_json::nlohmann_json)
    find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp
        PATHS /usr/include $ENV{HOME}/.local/include
    )
    if(NOT NLOHMANN_JSON_INCLUDE_DIR)
        message(FATAL_ERROR "nlohmann_json not found. Install nlohmann-json3-dev.")
    endif()
    add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)
    set_target_properties(nlohmann_json::nlohmann_json PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${NLOHMANN_JSON_INCLUDE_DIR}"
    )
endif()

# ---------------------------------------------------------------------------
# Main calibration binary
# ---------------------------------------------------------------------------

add_executable(multi_lidar_calibrate src/auto_calibrate.cpp)
target_include_directories(multi_lidar_calibrate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(multi_lidar_calibrate PRIVATE
    message_manager::message_manager
    ${PCL_LIBRARIES}
    nlohmann_json::nlohmann_json
)
target_compile_definitions(multi_lidar_calibrate PRIVATE ${PCL_DEFINITIONS})

install(TARGETS multi_lidar_calibrate RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# ---------------------------------------------------------------------------
# Unit tests (PCL + Eigen only — no DDS dependency)
# ---------------------------------------------------------------------------

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(calibration_tests tests/test_calibration.cpp)
target_include_directories(calibration_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(calibration_tests
    GTest::gtest_main
    ${PCL_LIBRARIES}
)
target_compile_definitions(calibration_tests PRIVATE ${PCL_DEFINITIONS})
add_test(NAME calibration_tests COMMAND calibration_tests)
