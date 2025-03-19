#ifndef GEN_FTHETA_H
#define GEN_FTHETA_H

#include "camera.h"

class GenFTheta : public Camera {
public:

    // this prevents blocking the overloaded methods
    using Camera::project;
    using Camera::backproject;

    GenFTheta() = default;

    GenFTheta(
        const std::vector<double>& focal_length,
        const std::vector<double>& principal_point,
        const double& skew,
        const std::vector<int>& image_size,
        const std::vector<double>& radial_distortion_sym = {},
        const std::vector<double>& radial_distortion_asym = {},
        const std::vector<double>& radial_distortion_four = {},
        const std::vector<double>& tangential_distortion_asym = {},
        const std::vector<double>& tangential_distortion_four = {},
        const Point3& rotation = { 0.0, 0.0, 0.0 },
        const Point3& translation = { 0.0, 0.0, 0.0 }
    );
    GenFTheta(const GenFTheta& model);

    //Functions for projecting and backprojecting points
    virtual std::array<double, 2> project(const Point3& point_3d) const override;
    virtual Point3 backproject(const std::array<double, 2>& point_2d) const override;

    // override getter methods
    virtual const std::string getModelName() const override;
    virtual std::shared_ptr<Pinhole> getPinhole() const override;

    // model specific getter methods
    std::vector<double> getFocalLength() const;
    std::vector<double> getPrincipalPoint() const;
    double getSkew() const;
    std::vector<double> getRadialDistSymCoeffs() const;
    std::vector<double> getRadialDistAsymCoeffs() const;
    std::vector<double> getRadialDistFourCoeffs() const;
    std::vector<double> getTangentialDistAsymCoeffs() const;
    std::vector<double> getTangentialDistFourCoeffs() const;
    void getBackprojectSettings(double& threshold, int& iterations) const;

    // model specific setter methods
    void setFocalLength(std::vector<double> focal_length);
    void setPrincipalPoint(std::vector<double> principal_point);
    void setSkew(double skew);
    void setRadialDistSymCoeffs(std::vector<double> radial_dist_sym_coeffs);
    void setRadialDistAsymCoeffs(std::vector<double> radial_dist_asym_coeffs);
    void setRadialDistFourCoeffs(std::vector<double> radial_dist_four_coeffs);
    void setTangentialDistAsymCoeffs(std::vector<double> tangential_dist_asym_coeffs);
    void setTangentialDistFourCoeffs(std::vector<double> tangential_dist_four_coeffs);
    void setBackprojectSettings(double threshold, int iterations);
    
    // load method
    static std::shared_ptr<GenFTheta> load(const std::string& fileName);

private:
    std::vector<double> focal_length { 1,1 };
    std::vector<double> principal_point{ 0,0 };
    double skew = 0;
    std::vector<double> radial_distortion_sym = { 1 };
    std::vector<double> radial_distortion_asym = { 0 };
    std::vector<double> radial_distortion_four = { 0 , 0 };
    std::vector<double> tangential_distortion_asym = { 0 };
    std::vector<double> tangential_distortion_four = { 0 , 0 };

    // backproject settings
    double threshold = 1e-6;
    int iterations = 20;

    std::string model_name = "General FTheta";

    std::vector<std::string> parameter_names = {
        "Focal Length",
        "Principal Point",
        "Skew",
        "Radial Distortion Symmetric Coefficients",
        "Radial Distortion Asymmetric Coefficients",
        "Radial Distortion Fourier Coefficients",
        "Tangential Distortion Asymmetric Coefficients",
        "Tangential Distortion Fourier Coefficients",
    };

    //Update
    std::vector<std::string> parameter_file_labels = {
        "mu_mv",
        "principal_point",
        "skew",
        "radial_distortion_coeff",
        "radial_asym_poly",
        "radial_asym_fourier",
        "tangential_asym_poly",
        "tangential_asym_fourier"
    };

    bool FULL = false;

    std::array<double, 3> evaluateDistortion(const std::array<double, 2>& pt) const;

    virtual const std::vector<std::vector<double>> getParameters() const override;
    virtual void setParameters(std::vector<std::vector<double>> parameters) override;
    virtual const std::vector<std::string> getParameterNames() const override;
    virtual const std::vector<std::string> getParameterLabels() const override;
};

#endif // GEN_FTHETA_H
