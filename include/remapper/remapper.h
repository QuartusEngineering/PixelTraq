#ifndef REMAPPER_H
#define REMAPPER_H

#include <vector>
#include <memory>
#include "camera/camera.h"

class Remapper {
public:
    Remapper(const std::shared_ptr<Camera>& cam_source);
    Remapper(const std::shared_ptr<Camera>& cam_source, const std::shared_ptr<Camera>& cam_target);
    Remapper(const std::shared_ptr<Camera>& cam_source, const std::shared_ptr<Camera>& cam_target, const Matrix3x3& rotation_matrix);

    std::vector<std::vector<std::vector<double>>>  undistort(const std::vector<std::vector<std::vector<double>>>& image);
    std::vector<std::vector<std::vector<double>>>  distort(const std::vector<std::vector<std::vector<double>>>& image);

private:
    void configure(const std::shared_ptr<Camera>& cam_source, const std::shared_ptr<Camera>& cam_target, const Matrix3x3& rotation_matrix = { { {1.0,0,0},{0,1.0,0},{0,0,1.0} } });
    
    std::shared_ptr<Camera> cam_source;
    std::shared_ptr<Camera> cam_target;

    int source_width;
    int source_height;
    int target_width;
    int target_height;

    std::vector<std::vector<double>> X, Y;
    std::vector<std::vector<double>> Xd, Yd, Xd_invert, Yd_invert;
};

#endif // REMAPPER_H
