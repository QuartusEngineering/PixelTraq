#ifndef BROWN_CONRADY_H
#define BROWN_CONRADY_H

#include "camera.h"
#include "pinhole.h"
#include "general_ftan_theta.h"
#include <vector>
#include <functional>

class BrownConrady : public Camera {
public:
    
    // this prevents blocking the overloaded methods
    using Camera::project;
    using Camera::backproject;

    BrownConrady();

    BrownConrady(
        const std::vector<double>& focal_length,
        const std::vector<double>& principal_point,
        const std::vector<int>& image_size,
        const std::vector<double>& radial_distortion = {},
        const std::vector<double>& tangential_distortion = {},
        const std::vector<double>& tangential_distortion_polycoeff = {},
        const Point3& rotation = { 0.0, 0.0, 0.0 },
        const Point3& translation = { 0.0, 0.0, 0.0 }
    );
    BrownConrady(const BrownConrady& model);

    //Functions for projecting and backprojecting points
    std::array<double, 2> project(const Point3& point_3d) const override;
    Point3 backproject(const std::array<double, 2>& point_2d) const override;

    // override getter methods
    virtual const std::string getModelName() const override;
    virtual std::shared_ptr<Pinhole> getPinhole() const override;
    
    // model specific getter methods
    std::vector<double> getPrincipalPoint() const;
    std::vector<double> getFocalLength() const;
    std::vector<double> getRadialDistCoeffs() const;
    std::vector<double> getTangentialDistCoeffs() const;
    std::vector<double> getTangentialPolynominalDistCoeffs() const;
    void getBackprojectSettings(double& threshold, int& iterations) const;

    // model specific setter methods
    void setPrincipalPoint(std::vector<double> principal_point);
    void setFocalLength(std::vector<double> focal_length);
    void setRadialDistCoeffs(std::vector<double> radial_dist_coeffs);
    void setTangentialDistCoeffs(std::vector<double> tangential_dist_coeffs);
    void setTangentialPolynominalDistCoeffs(std::vector<double> tangential_polynomial_dist_coeffs);
    void setBackprojectSettings(double threshold, int iterations);
    
    // load method
    static std::shared_ptr<BrownConrady> load(const std::string& fileName);

protected:
    std::unique_ptr<GenFTanTheta> internalModel = std::make_unique<GenFTanTheta>();

    std::string model_name = "Brown Conrady";

    std::vector<std::string> parameter_names = {
        "Focal Length",
        "Principal Point",
        "Radial Distortion Coefficients",
        "Tangential Distortion Coefficients",
        "Tangential Distortion Polynomial Coefficients"
    };

    std::vector<std::string> parameter_file_labels = {
        "EFL",
        "principal_point",
        "radial_distortion_coeff",
        "tangential_distortion_coeff",
        "tangential_distortion_poly_coeff"
    };

    virtual const std::vector<std::vector<double>> getParameters() const override;
    virtual void setParameters(std::vector<std::vector<double>> parameters) override;
    virtual const std::vector<std::string> getParameterNames() const override;
    virtual const std::vector<std::string> getParameterLabels() const override;
};

#endif // BROWN_CONRADY_H
