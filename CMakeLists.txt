cmake_minimum_required(VERSION 3.16)
project(RocketUpFilters VERSION 0.1.0 DESCRIPTION "State estimation for rocket flight computers" LANGUAGES CXX)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(RUFILTERS_TOP_LEVEL ON)
else()
    set(RUFILTERS_TOP_LEVEL OFF)
endif()
option(RUFILTERS_BUILD_TESTS "Build tests" ${RUFILTERS_TOP_LEVEL})
option(RUFILTERS_BUILD_EXAMPLES "Build desktop examples" ${RUFILTERS_TOP_LEVEL})

# Header-only; `src/` is also the Arduino library source folder.
add_library(RocketUpFilters INTERFACE)
add_library(RocketUpFilters::RocketUpFilters ALIAS RocketUpFilters)
target_include_directories(RocketUpFilters INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>)
target_compile_features(RocketUpFilters INTERFACE cxx_std_11)

if(RUFILTERS_BUILD_TESTS)
    enable_testing()
    add_executable(rufilters_tests tests/test_filters.cpp)
    target_link_libraries(rufilters_tests PRIVATE RocketUpFilters::RocketUpFilters)
    set_target_properties(rufilters_tests PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON)
    if(MSVC)
        target_compile_options(rufilters_tests PRIVATE /W4)
    else()
        target_compile_options(rufilters_tests PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
    endif()
    add_test(NAME rufilters_tests COMMAND rufilters_tests)
endif()

if(RUFILTERS_BUILD_EXAMPLES)
    add_executable(replay_rocketup examples/desktop/replay_rocketup.cpp)
    target_link_libraries(replay_rocketup PRIVATE RocketUpFilters::RocketUpFilters)
endif()

install(DIRECTORY src/ DESTINATION include)
