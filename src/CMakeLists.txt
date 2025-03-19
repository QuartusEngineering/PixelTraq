
# Add the camera and remapper libraries
add_subdirectory(camera)
add_subdirectory(remapper)
add_subdirectory(utilities)

target_link_libraries(PixelTraq INTERFACE camera remapper utils)


if(APPLE)
    set(CMAKE_C_COMPILER "/usr/local/Cellar/llvm/5.0.1/bin/clang")
    set(CMAKE_CXX_COMPILER "/usr/local/Cellar/llvm/5.0.1/bin/clang++")
    set(OPENMP_LIBRARIES "/usr/local/Cellar/llvm/5.0.1/lib")
    set(OPENMP_INCLUDES "/usr/local/Cellar/llvm/5.0.1/include")
endif()
# Find OpenMP
if(APPLE AND USE_OpenMP)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang")
        set(OpenMP_C "${CMAKE_C_COMPILER}")
        set(OpenMP_C_FLAGS "-fopenmp=libomp -Wno-unused-command-line-argument")
        set(OpenMP_C_LIB_NAMES "libomp" "libgomp" "libiomp5")
        set(OpenMP_libomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libgomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libiomp5_LIBRARY ${OpenMP_C_LIB_NAMES})
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
      set(OpenMP_CXX_FLAGS "-fopenmp=libomp -Wno-unused-command-line-argument")
      set(OpenMP_CXX_LIB_NAMES "libomp" "libgomp" "libiomp5")
      set(OpenMP_libomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
      set(OpenMP_libgomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
      set(OpenMP_libiomp5_LIBRARY ${OpenMP_CXX_LIB_NAMES})
    endif()
endif()

OPTION (USE_OpenMP "Use OpenMP to enable <omp.h>" ON)

# Add this to the top of your CMakeLists.txt to enable OpenMP
find_package(OpenMP REQUIRED)

