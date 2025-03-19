# PixelTraq
![License](https://img.shields.io/badge/license-Apache2.0-blue.svg)

The PixelTraq library is a C++ library developed to help users integrate cameras calibrated using the PixelTraq Camera Calibration  system into their projects. The library contains ready made camera model implementations and code for common workflows such as image undistortion, 3d reconstruction, and stereo rectification.

## External Links
[PixelTraq Service](https://www.quartus.com/products/pixeltraq/)\
[PixelTraq Resources](https://www.quartus.com/products/pixeltraq/resources/)\
[Documentation](https://github.com/pages/QuartusEngineering/PixelTraq)

## Integration

There are a few different ways to include the library in your project. The following sections outline how to integrate the library into your project.

### Stand Alone to Run Examples

If you are only interested in running the examples or tools, you can clone the library directly using:
```
git clone https://github.com/QuartusEngineering/PixelTraq.git
```

The examples can be built as long as you have cmake installed using the following steps:

```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2"
cmake --build . --parallel 2 --config Release
```
Replace --parallel 2 with --parallel \<number of cores in your machine>.

### Add to a Project Using CMake

#### Add as a Subdirectory

To add PixelTraq as a subdirectory, clone the project into your main project folder or add it as a submodule.
```
git clone https://github.com/QuartusEngineering/PixelTraq.git
```

Next, add the following line to your project's CMakeLists.txt file
```
add_subdirectory(PixelTraq)
```

You will also need to target the library to link to your execuatable 
```
add_executable(target main.cpp)
target_link_libraries(target PUBLIC PixelTraq)
```

Any source files that use the PixelTraq library need to include the entry point header file
```
#include <PixelTraq/pixeltraq.h>
```

#### Add using FetchContent

Enter the following code block into your cmake file to fetch the pixeltraq github

```cmake
# Include FetchContent before using it
include(FetchContent)

# Fetch the external project (pixelTraq)
FetchContent_Declare(
    PixelTraq
    GIT_REPOSITORY https://github.com/QuartusEngineering/PixelTraq.git
    GIT_TAG main  # Use the specific branch or commit hash if needed
)

FetchContent_MakeAvailable(PixelTraq)

# Define the executable
add_executable(<yourProject> "<yourProject.cpp>" "<yourProject.h>")

#add the include directory
target_include_directories(<yourProject> PRIVATE ${pixeltraq_SOURCE_DIR}/include)

#link the pixelTraq library
target_link_libraries(<yourProject> PRIVATE PixelTraq)
```

## Examples

We have created a number of examples for you to follow in order for you to familiarize yourself with this library.

### ./scripts/examples/load_save_example.cpp
This example shows how to consume a pixelTraq json file and save out the json file.

### ./scripts/examples/projection_example.cpp
This example shows how to manually set up your own Camera model then project points from world frame to camera frame. Then from camera frame to the image plane

### ./scripts/examples/reconstruction_example.cpp
This examples showns how to reconstruct 3d points from measurements in 2d stereo images

### ./scripts/examples/stereo_rectification_example.cpp
This method shows how to rectify images captured from the left and right camera of a stereo module.

### ./scripts/examples/undistortion_example.cpp
This example shows how to initialize a remapper which can be used for quickly undistorting an image

## Tools

### ./scripts/tools/undistort_image.cpp
This tool can be built as an executable for undistorting an image using a camera model file and an optional target camera model file.

## License
This library is licensed under the Apache License Version 2.0 - see the LICENSE file for details.
