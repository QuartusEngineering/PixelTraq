#ifndef KANNALA_H
#define KANNALA_H

#include "camera.h"
#include "pinhole.h"
#include "general_ftheta.h"
#include <vector>

class Kannala : public Camera {


public:

    // this prevents blocking the overloaded methods
    using Camera::project;
    using Camera::backproject;

    Kannala();

    Kannala(
        const std::vector<double>& focal_length,
        const std::vector<double>& principal_point,
        const std::vector<int>& image_size,
        const std::vector<double>& radial_distortion_sym = {},
        const std::vector<double>& radial_distortion_asym = {},
        const std::vector<double>& radial_distortion_four = {},
        const std::vector<double>& tangential_distortion_asym = {},
        const std::vector<double>& tangential_distortion_four = {},
        const Point3& rotation = { 0.0, 0.0, 0.0 },
        const Point3& translation = { 0.0, 0.0, 0.0 }
    );

    Kannala(const Kannala& model);

    //Functions for projecting and backprojecting points
    virtual Point2 project(const Point3& point_3d) const override;
    virtual Point3 backproject(const Point2& point_2d) const override;

    // general getter/setter methods
    virtual const std::string getModelName() const override;
    virtual std::shared_ptr<Pinhole> getPinhole() const override;

    // model specific getter methods
    std::vector<double> getFocalLength() const;
    std::vector<double> getPrincipalPoint() const;
    std::vector<double> getRadialDistSymCoeffs() const;
    std::vector<double> getRadialDistAsymCoeffs() const;
    std::vector<double> getRadialDistFourCoeffs() const;
    std::vector<double> getTangentialDistAsymCoeffs() const;
    std::vector<double> getTangentialDistFourCoeffs() const;
    void getBackprojectSettings(double& threshold, int& iterations) const;

    // model specific setter methods
    void setFocalLength(std::vector<double> focal_length);
    void setPrincipalPoint(std::vector<double> principal_point);
    void setRadialDistSymCoeffs(std::vector<double> radial_dist_sym_coeffs);
    void setRadialDistAsymCoeffs(std::vector<double> radial_dist_asym_coeffs);
    void setRadialDistFourCoeffs(std::vector<double> radial_dist_four_coeffs);
    void setTangentialDistAsymCoeffs(std::vector<double> tangential_dist_asym_coeffs);
    void setTangentialDistFourCoeffs(std::vector<double> tangential_dist_four_coeffs);
    void setBackprojectSettings(double threshold, int iterations);

    // load method
    static std::shared_ptr<Kannala> load(const std::string& fileName);

protected:
    std::string model_name = "Kannala";

    std::vector<std::string> parameter_names = {
        "Focal Length",
        "Principal Point",
        "Radial Distortion Symmetric Coefficients",
        "Radial Distortion Asymmetric Coefficients",
        "Radial Distortion Fourier Coefficients",
        "Tangential Distortion Asymmetric Coefficients",
        "Tangential Distortion Fourier Coefficients"
    };

    std::vector<std::string> parameter_file_labels = {
        "mu_mv",
        "principal_point",
        "radial_distortion_coeff",
        "radial_asym_poly",
        "radial_asym_fourier",
        "tangential_asym_poly",
        "tangential_asym_fourier"
    };

    std::unique_ptr<GenFTheta> internalModel = std::make_unique<GenFTheta>();

    virtual const std::vector<std::vector<double>> getParameters() const override;
    virtual void setParameters(std::vector<std::vector<double>> parameters) override;
    virtual const std::vector<std::string> getParameterNames() const override;
    virtual const std::vector<std::string> getParameterLabels() const override;
};

#endif // KANNALA_H
