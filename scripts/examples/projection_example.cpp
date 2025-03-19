#include <iostream>
#include <stdexcept>
#include "pixeltraq.h"

#ifdef ENABLE_PLOTS
#include <matplot/matplot.h>
#endif

int main() {

#ifdef ENABLE_PLOTS
    using namespace matplot;
#endif

    try {
        // create camera model
        std::vector<double> focal_length = { 200, 200 };
        std::vector<double> principal_point = { 320, 240 };
        std::vector<int> image_size = { 640, 480 };
        std::vector<double> radial_dist_sym = { 0.1 };
        std::vector<double> radial_distortion_asym = { };
        std::vector<double> radial_distortion_four = { };
        std::vector<double> tangential_distortion_asym = { };
        std::vector<double> tangential_distortion_four = { };
        Point3 rotation = { 0, 0, 0 };
        Point3 translation = { 0.1, 0, 3};
        auto model = std::make_shared<Kannala>(
            focal_length,
            principal_point,
            image_size,
            radial_dist_sym,
            radial_distortion_asym,
            radial_distortion_four,
            tangential_distortion_asym,
            tangential_distortion_four,
            rotation,
            translation
        );

        // Create a checkerboard
        std::vector<std::array<double, 3>> checkerboardVertices;
        const int rows = 10;
        const int cols = 10;

        // Size of each square
        const double squareSize = 0.1;

        // Generate the coordinates
        for (int i = 0; i <= rows; ++i) {
            for (int j = 0; j <= cols; ++j) {
                double x = -0.5 + j * squareSize;
                double y = -0.5 + i * squareSize;
                double z = 0.0;
                checkerboardVertices.push_back({ x, y, z });
            }
        }

        // This transformation moves the points from the world frame to the camera frame, which must be done before projection
        auto pointsInCamera = model->worldToCameraPnts(checkerboardVertices);

        // project points into the images
        // This section simulates a camera projecting the 3d points into the image
        auto imagePoints = model->project(pointsInCamera);

#ifdef ENABLE_PLOTS
        // Display the projected points
        std::vector<double> x, y;
        for (int i = 0; i < imagePoints.size(); i++)
        {
            x.push_back(imagePoints[i][0]);
            y.push_back(imagePoints[i][1]);
        }
        plot(x, y, "*");
        title("Grid Points Projected Into Image");
        show();
#endif

        // backproject
        // This section shows how we can invert the camera model to get the rays for each image point 
        auto rays = model->backproject(imagePoints);

#ifdef ENABLE_PLOTS
        // Display the projected points
        std::vector<double> xr, yr, zr, origin;
        for (int i = 0; i < rays.size(); i++)
        {
            origin.push_back(0.0);
            xr.push_back(rays[i][0]);
            yr.push_back(rays[i][1]);
            zr.push_back(rays[i][2]);
        }
        figure();
        quiver3(origin, origin, origin, xr, yr, zr);
        title("Grid Rays Projected Out of Camera");
        show();
#endif
    }
    catch (const std::exception& e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}