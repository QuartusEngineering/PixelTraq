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
        
        // create left camera model
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
        auto leftCamera = std::make_shared<Kannala>(
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

        // create right camera model from left
        auto rightCamera = std::make_shared<Kannala>(*leftCamera); // this is how you copy the object
        rightCamera->setTranslation({ -0.1,0, 3 });

        // create world points
        // These points represent the points of the 3d object that willl used to form the images and then reconstructed
        std::vector<std::array<double,3>> cubeVertices = {
            {-1.0, -1.0, -1.0},
            {-1.0, -1.0,  1.0},
            {-1.0,  1.0, -1.0},
            {-1.0,  1.0,  1.0},
            {1.0, -1.0, -1.0},
            {1.0, -1.0,  1.0},
            {1.0,  1.0, -1.0},
            {1.0,  1.0,  1.0}
        };

        // transform the points into both camera frames
        // This transformation moves the points from the world frame to the camera frame, which must be done before projection
        auto pointsInLeftCamera = leftCamera->worldToCameraPnts(cubeVertices);
        auto pointsInRightCamera = rightCamera->worldToCameraPnts(cubeVertices);

        // project points into the images
        // This section simulates a stereo camera projecting the 3d points into two images
        auto imagePointsLeft = leftCamera->project(pointsInLeftCamera);
        auto imagePointsRight = rightCamera->project(pointsInRightCamera);

#ifdef ENABLE_PLOTS
        // Display the projected points
        std::vector<double> xL, yL, xR, yR;
        for (int i = 0; i < imagePointsLeft.size(); i++)
        {
            xL.push_back(imagePointsLeft[i][0]);
            yL.push_back(imagePointsLeft[i][1]);
            xR.push_back(imagePointsRight[i][0]);
            yR.push_back(imagePointsRight[i][1]);
        }
        plot(xL, yL, "*", xR, yR, "*");
        title("Projections Overlaid");
        matplot::legend({"Left","Right"});
        show();
#endif

        // backproject
        // This section shows how we can invert the camera model to get the rays for each image point 
        auto raysLeft = leftCamera->backproject(imagePointsLeft);
        auto raysRight = rightCamera->backproject(imagePointsRight);

        // 3d reconstructions
        // Finally, we reconstruct 3d points via ray intersection
        auto verticesRecovered = CommonMath::intersectRays(leftCamera,rightCamera,raysLeft,raysRight);

        // compute reconstruction error
        std::vector<Point3> reconstructionErrors;
        reconstructionErrors.reserve(cubeVertices.size());
        for (int i = 0; i < cubeVertices.size(); i++)
        {
            reconstructionErrors.push_back(cubeVertices[i] - verticesRecovered[i]);
        }

#ifdef ENABLE_PLOTS
        // Display the projected points
        std::vector<double> x, y, z, xo, yo, zo;
        for (int i = 0; i < verticesRecovered.size(); i++)
        {
            x.push_back(verticesRecovered[i][0]);
            y.push_back(verticesRecovered[i][1]);
            z.push_back(verticesRecovered[i][2]);
            xo.push_back(cubeVertices[i][0]);
            yo.push_back(cubeVertices[i][1]);
            zo.push_back(cubeVertices[i][2]);
        }
        figure();
        plot3(x, y,z, "*", xo, yo, zo, "o");
        title("Reconstruction Overlaid");
        matplot::legend({ "Original","Reconstructed" });
        show();
#endif()
    }
    catch (const std::exception& e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}