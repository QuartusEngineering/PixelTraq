cmake_minimum_required(VERSION 3.14)
project(PixelTraq)

add_library(PixelTraq INTERFACE)


include(FetchContent)


# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_PLOTS "Enable matplotplusplus plotting" ON)
option(ENABLE_TESTS "Enable tests" ON)

include_directories(${PROJECT_SOURCE_DIR}/include)




add_library(nlohmann_json INTERFACE)
target_include_directories(nlohmann_json INTERFACE ${CMAKE_SOURCE_DIR}/include/external/nlohmann)

add_library(stb_image INTERFACE)
target_include_directories(stb_image INTERFACE ${CMAKE_SOURCE_DIR}/include/external/stb_image)

if (ENABLE_PLOTS)
    FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG "v1.2.2")

    FetchContent_GetProperties(matplotplusplus)
    if(NOT matplotplusplus_POPULATED)
        FetchContent_Populate(matplotplusplus)
        add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()
else()
    message(STATUS "Optional dependency matplotplusplus not selected.")
endif()

add_subdirectory(src)
add_subdirectory(scripts)

if (ENABLE_TESTS)
    add_subdirectory(tests)
endif()

file(GLOB_RECURSE SOURCE_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp" "${CMAKE_SOURCE_DIR}/scripts/*.cpp" "${CMAKE_SOURCE_DIR}/include/*.h")

# compiler optimizations
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

