#ifndef PINHOLE_H
#define PINHOLE_H

#include <vector>
#include "camera.h"

class Pinhole : public Camera {
public:

    // this prevents blocking the overloaded methods
    using Camera::project;
    using Camera::backproject;

    Pinhole() = default;
    // Constructor that initializes with focal_length and principal_point vectors
    Pinhole(const std::vector<double>& focal_length,
        const std::vector<double>& principal_point,
        const double& skew,
        const std::vector<int>& image_size,
        const Point3& rotation = { 0.0, 0.0, 0.0 },
        const Point3& translation = { 0.0, 0.0, 0.0 });
    Pinhole(const Pinhole& model);

    // Implement projection and backprojection for Pinhole camera model
    std::array<double, 2> project(const Point3& point_3d) const override;

    // backproject a single 2D point to a 3D direction vector
    Point3 backproject(const std::array<double, 2>& point_2d) const;

    // override getter methods
    virtual const std::string getModelName() const override;
    virtual std::shared_ptr<Pinhole> getPinhole() const override;

    // model specific getter methods
    std::vector<double> getPrincipalPoint() const;
    std::vector<double> getFocalLength() const;
    double getSkew() const;

    // model specific setter methods
    void setPrincipalPoint(std::vector<double> principal_point);
    void setFocalLength(std::vector<double> focal_length);
    void setSkew(double skew);

    // load method
    static std::shared_ptr<Pinhole> load(const std::string& fileName);

protected:
    std::vector<double> principal_point = { 0,0 };
    std::vector<double> focal_length = { 1,1 };
    double skew = 0;

    std::string model_name = "Pinhole";

    std::vector<std::string> parameter_names = {
        "Focal Length",
        "Principal Point",
        "Skew"
    };

    //Update
    std::vector<std::string> parameter_file_labels = {
        "focal_distance",
        "principal_point",
        "skew"
    };

    virtual const std::vector<std::vector<double>> getParameters() const override;
    virtual void setParameters(std::vector<std::vector<double>> parameters) override;
    virtual const std::vector<std::string> getParameterNames() const override;
    virtual const std::vector<std::string> getParameterLabels() const override;
};

#endif // PINHOLE_H
