add_library(camera STATIC
    camera.cpp
    pinhole.cpp
    general_ftheta.cpp
    general_ftan_theta.cpp
    kannala.cpp
    brown_conrady.cpp
)

target_include_directories(camera PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(camera PUBLIC utils nlohmann_json)