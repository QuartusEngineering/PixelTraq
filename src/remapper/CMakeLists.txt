add_library(remapper STATIC remapper.cpp)

target_include_directories(remapper PUBLIC ${CMAKE_SOURCE_DIR}/include/remapper)

target_link_libraries(remapper PUBLIC camera utils)