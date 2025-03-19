#include "remapper/remapper.h"
#include "camera/pinhole.h"
#include <vector>

/**
 * @brief Constructs a Remapper with a source camera and automatically selects the result of getPinhole as the target camera
 *
 * @param cam_source Shared pointer to the source Camera object.
 */
Remapper::Remapper(const std::shared_ptr<Camera>& cam_source) : cam_source(cam_source)
{
    Remapper::configure(cam_source, std::static_pointer_cast<Camera>(cam_source->getPinhole()));
}

/**
 * @brief Constructs a Remapper with a source and a target camera.
 *
 * @param cam_source Shared pointer to the source Camera object.
 * @param cam_target Shared pointer to the target Camera object.
 */
Remapper::Remapper(const std::shared_ptr<Camera>& cam_source, const std::shared_ptr<Camera>& cam_target)
    : cam_source(cam_source), cam_target(cam_target) {

    Remapper::configure(cam_source, cam_target);
}

/**
 * @brief Constructs a Remapper with a source camera, a target camera, and a rotation matrix.
 *
 * @param cam_source Shared pointer to the source Camera object.
 * @param cam_target Shared pointer to the target Camera object.
 * @param rotation_matrix The rotation matrix used to map source to target.
 */
Remapper::Remapper(const std::shared_ptr<Camera>& cam_source, const std::shared_ptr<Camera>& cam_target, const Matrix3x3& rotation_matrix)
    : cam_source(cam_source), cam_target(cam_target) {

    Remapper::configure(cam_source, cam_target, rotation_matrix);
}

/**
 * @brief Applies distortion to an image following the mapping from target to source.
 *
 * @param image The input image to be distorted.
 * @return The distorted image.
 */
std::vector<std::vector<std::vector<double>>>  Remapper::distort(const std::vector<std::vector<std::vector<double>>>& image) {
    return CommonMath::interp2(image, Xd_invert, Yd_invert);
}

/**
 * @brief Removes distortion from an image following the mapping from source to target.
 *
 * @param image The input image to be undistorted.
 * @return The undistorted image.
 */
std::vector<std::vector<std::vector<double>>>  Remapper::undistort(const std::vector<std::vector<std::vector<double>>>& image) {
    return CommonMath::interp2(image, Xd, Yd);
}

void Remapper::configure(const std::shared_ptr<Camera>& cam_source, const std::shared_ptr<Camera>& cam_target, const Matrix3x3& rotation_matrix)
{
    
    std::vector<int> source_size = cam_source->getImageSize();

    source_width = source_size[0];
    source_height = source_size[1];

    X.resize(source_height, std::vector<double>(source_width, 0));
    Y.resize(source_height, std::vector<double>(source_width, 0));

    for (int y = 0; y < source_height; ++y) {
        for (int x = 0; x < source_width; ++x) {
            X[y][x] = x + 1;
            Y[y][x] = y + 1;
        }
    }

    std::vector<int> target_size = cam_target->getImageSize();
    target_width = target_size[0];
    target_height = target_size[1];

    std::vector<std::array<double, 2>> target_pixels;
#pragma omp parallel for
    for (int y = 0; y < target_height; ++y) {
        for (int x = 0; x < target_width; ++x) {
            target_pixels.push_back({ static_cast<double>(x), static_cast<double>(y) });
        }
    }

    std::vector<std::array<double, 2>> source_pixels;
#pragma omp parallel for
    for (int y = 0; y < source_height; ++y) {
        for (int x = 0; x < source_width; ++x) {
            source_pixels.push_back({ static_cast<double>(x), static_cast<double>(y) });
        }
    }

    std::vector<std::array<double, 3>> grid_rays = cam_target->backproject(target_pixels);
    std::vector<std::array<double, 2>> distorted_pixels = cam_source->project(CommonMath::rotatePoints(grid_rays, CommonMath::rotationInverse(rotation_matrix)));

    // these sections are repeated, should turn into function
    Xd.resize(target_height, std::vector<double>(target_width, 0));
    Yd.resize(target_height, std::vector<double>(target_width, 0));

#pragma omp parallel for
    for (size_t i = 0; i < distorted_pixels.size(); ++i) {
        size_t row = i / target_width;
        size_t col = i % target_width;
        Xd[row][col] = distorted_pixels[i][0];
        Yd[row][col] = distorted_pixels[i][1];
    }

    std::vector<std::array<double, 3>> grid_rays_invert = cam_source->backproject(source_pixels);
    std::vector<std::array<double, 2>> distorted_pixels_invert = cam_target->project(CommonMath::rotatePoints(grid_rays_invert, rotation_matrix));

    Xd_invert.resize(source_height, std::vector<double>(source_width, 0));
    Yd_invert.resize(source_height, std::vector<double>(source_width, 0));

#pragma omp parallel for
    for (size_t i = 0; i < distorted_pixels_invert.size(); ++i) {
        size_t row = i / source_width;
        size_t col = i % source_width;
        Xd_invert[row][col] = distorted_pixels_invert[i][0];
        Yd_invert[row][col] = distorted_pixels_invert[i][1];
    }
}


