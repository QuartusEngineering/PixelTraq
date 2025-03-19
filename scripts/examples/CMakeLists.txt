set(DATA_DIR "${CMAKE_SOURCE_DIR}/scripts/examples/inputs")

add_executable(load_save_example "load_save_example.cpp")
target_link_libraries(load_save_example PRIVATE camera)
target_compile_definitions(load_save_example PRIVATE DATA_DIR="${DATA_DIR}")

add_executable(projection_example "projection_example.cpp")
target_link_libraries(projection_example PRIVATE camera)

add_executable(undistortion_example "undistortion_example.cpp")
target_link_libraries(undistortion_example PRIVATE remapper camera)
target_compile_definitions(undistortion_example PRIVATE DATA_DIR="${DATA_DIR}")

add_executable(reconstruction_example "reconstruction_example.cpp")
target_link_libraries(reconstruction_example PRIVATE camera)

add_executable(stereo_rectification_example "stereo_rectification_example.cpp")
target_link_libraries(stereo_rectification_example PRIVATE remapper)
target_compile_definitions(stereo_rectification_example PRIVATE DATA_DIR="${DATA_DIR}")

if (ENABLE_PLOTS)
    target_link_libraries(projection_example PRIVATE matplot)
    target_link_libraries(reconstruction_example PRIVATE matplot)
    add_definitions(-DENABLE_PLOTS)
endif()