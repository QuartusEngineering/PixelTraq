add_library(utils STATIC "utils.cpp" "common_math.cpp")

# Add include directories for this library
target_include_directories(utils PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(utils PUBLIC nlohmann_json stb_image)