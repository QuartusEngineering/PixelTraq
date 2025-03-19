#ifndef CAMERA_H
#define CAMERA_H

#include <string>
#include <iostream>
#include <memory>
#include "external/nlohmann/json.hpp"
#include "utilities/utils.h"
#include "utilities/common_math.h"

class Pinhole; // Forward declaration

class Camera : public std::enable_shared_from_this<Camera> {
public:

    Camera();
    Camera(std::vector<int> image_size, Point3 rotation, Point3 translation);
    virtual ~Camera() = default;

    std::string getParameterDisplayString() const;
    void display() const;

    // getter for accessing image size
    std::vector<int> getImageSize() const;
    void setImageSize(std::vector<int> image_size);

    // Pure virtual functions
    virtual Point2 project(const Point3& point_3d) const = 0;
    virtual Point3 backproject(const Point2& point_2d) const = 0;
    virtual const std::string getModelName() const = 0;
    virtual std::shared_ptr<Pinhole> getPinhole() const = 0;

    // Multiple Array projects
    std::vector<Point2> project(const std::vector<Point3>& points_3d) const;
    std::vector<std::vector<Point2>> project(const std::vector<std::vector<Point3>>& points_3d1) const;
    std::vector<std::vector<std::array<double, 3>>> backproject(const std::vector<std::vector<double>>& image) const;
    std::vector<Point3> backproject(const std::vector<Point2>& points_2d) const;

    // Getters for extrinsic parameters
    const Point3 getTranslation() const { return translation; }
    const Point3 getRotation() const { return rotation; }
    void setTranslation(Point3 translation);
    void setRotation(Point3 rotation);
    const Point3 getInvTranslation() const { return inv_translation; }
    const Matrix3x3 getRotationMatrix() const { return rotation_matrix; }
    const Matrix3x3 getInvRotationMatrix() const { return inv_rotation_matrix; }

    // Extrinsic Functions
    const std::vector<Point3> worldToCameraPnts(const std::vector<Point3>& world_points) const;
    const std::vector<Point3> cameraToWorldPnts(const std::vector<Point3>& world_points) const;
    const Point3 worldToCameraPnts(const Point3& world_point) const;
    const Point3 cameraToWorldPnts(const Point3& world_point) const;

    static std::shared_ptr<Camera> load(const std::string& fileName);
    void save(const std::string& fileName) const;

protected:
    Point3 translation { 0,0,0 };       // Translation [tx, ty, tz]
    Point3 rotation { 0,0,0 };          // Rotation [rx, ry, rz]
    Matrix3x3 rotation_matrix;
    Matrix3x3 inv_rotation_matrix;
    Point3 inv_translation;
    std::vector<int>  image_size {0,0};          // Image size [size x, size y]
    std::string model_name;
    std::vector<std::function<std::vector<double>()>> properties;
    std::vector<std::string> labels;

    Point3 createInverseTranslation(const Point3& translation);

    virtual const std::vector<std::vector<double>> getParameters() const = 0;
    virtual void setParameters(std::vector<std::vector<double>> parameters) = 0;
    virtual const std::vector<std::string> getParameterNames() const = 0;
    virtual const std::vector<std::string> getParameterLabels() const = 0;

private:
    static std::string getFileExtension(const std::string& fileName);
    static std::shared_ptr<Camera> loadCameraFromJson(const std::string& fileName);
    static void saveCameraToJson(const std::shared_ptr<const Camera>& camera, const std::string& fileName);

    // Load parameters from JSON (common to all cameras)
    void loadExtrinsicsFromJson(const nlohmann::json& j);
    void loadIntrinsicsFromJson(const nlohmann::json& j);
};

#endif // CAMERA_H
