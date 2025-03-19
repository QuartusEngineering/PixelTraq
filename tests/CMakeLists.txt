include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.11.0
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Explicitly add Google Test targets
add_library(GTest::GTest ALIAS gtest)
add_library(GTest::Main ALIAS gtest_main)

# Create an executable for the test
set(TEST_DATA_DIR "${CMAKE_SOURCE_DIR}/tests/test_inputs")
add_executable(tests 
	test_main.cpp 
	src/test_camera_classes.cpp 	
	src/pinhole_test.cpp
	src/brown_conrady_test.cpp
	src/kannala_test.cpp
	src/camera_test.cpp
	src/remapper_test.cpp
	src/commonmath_test.cpp
)

target_compile_definitions(tests PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
target_include_directories(tests PRIVATE ${PROJECT_SOURCE_DIR}/tests/include)
target_link_libraries(tests PRIVATE GTest::GTest GTest::Main nlohmann_json camera stb_image remapper)

# Enable C++17 or later
target_compile_features(tests PRIVATE cxx_std_17)

# Add tests to CTest
include(GoogleTest)
gtest_discover_tests(tests)