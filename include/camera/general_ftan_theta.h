#ifndef GEN_FTANTHETA_H
#define GEN_FTANTHETA_H

#include "camera.h"

class GenFTanTheta : public Camera {
public:

    // this prevents blocking the overloaded methods
    using Camera::project;
    using Camera::backproject;

    GenFTanTheta() = default;

    GenFTanTheta(
        const std::vector<double>& focal_length,
        const std::vector<double>& principal_point,
        const double& skew,
        const std::vector<int>& image_size,
        const std::vector<double>& radial_distortion_num = {},
        const std::vector<double>& radial_distortion_den = {},
        const std::vector<double>& tangential_distortion = {},
        const std::vector<double>& tangential_distortion_polycoeff = {},
        const std::vector<double>& tangential_distortion_OCVcoeff = {},
        const Point3& rotation = { 0.0, 0.0, 0.0 },
        const Point3& translation = { 0.0, 0.0, 0.0 }
    );
    GenFTanTheta(const GenFTanTheta& model);

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
    std::vector<double> getRadialDistNumCoeffs() const;
    std::vector<double> getRadialDistDenCoeffs() const;
    std::vector<double> getTangentialDistCoeffs() const;
    std::vector<double> getTangentialPolynominalDistCoeffs() const;
    std::vector<double> getTangentialDistOCVCoeffs() const;
    void getBackprojectSettings(double &threshold, int &iterations ) const;

    // model specific setter methods
    void setFocalLength(std::vector<double> focal_length);
    void setPrincipalPoint(std::vector<double> principal_point);
    void setSkew(double skew);
    void setRadialDistNumCoeffs(std::vector<double> radial_dist_num_coeffs);
    void setRadialDistDenCoeffs(std::vector<double> radial_dist_den_coeffs);
    void setTangentialDistCoeffs(std::vector<double> tangential_dist_coeffs);
    void setTangentialPolynominalDistCoeffs(std::vector<double> tangential_polynomial_dist_coeffs);
    void setTangentialDistOCVCoeffs(std::vector<double> tangential_dist_ocv);
    void setBackprojectSettings(double threshold, int iterations);

    // load method
    static std::shared_ptr<GenFTanTheta> load(const std::string& fileName);

protected:
    std::vector<double> focal_length { 1,1 };
    std::vector<double> principal_point { 0,0 };
    double skew = 0;
    std::vector<double> radial_distortion_num { 1 };
    std::vector<double> radial_distortion_den { 1 };
    std::vector<double> tangential_distortion {};
    std::vector<double> tangential_distortion_polycoeff { 1 };
    std::vector<double> tangential_distortion_OCVcoeff_x {};
    std::vector<double> tangential_distortion_OCVcoeff_y {};

    // backproject settings
    double threshold = 1e-6;
    int iterations = 20;

    std::string model_name = "General FTan Theta";

    std::vector<std::string> parameter_names = {
        "Focal Length",
        "Principal Point",
        "Skew",
        "Radial Distortion Numerator Coefficients",
        "Radial Distortion Denominator Coefficients",
        "Tangential Distortion Coefficients",
        "Tangential Distortion Polynomial Coefficients",
        "Tangential Distortion OCV Coefficients"
    };

    //Update
    std::vector<std::string> parameter_file_labels = {
        "EFL",
        "principal_point",
        "skew",
        "radial_distortion_num_coeff",
        "radial_distortion_denom_coeff",
        "tangential_distortion_coeff",
        "tangential_distortion_poly_coeff"
        "tangential_distortion_ocv_coeff"
    };

    bool RADD = false;
    bool TANPOLY = false;
    bool TANOCV = false;
    bool TANDIST = false;

    std::array<double, 3> evaluateDistortion(const std::array<double, 2>& pt) const;

    virtual const std::vector<std::vector<double>> getParameters() const override;
    virtual void setParameters(std::vector<std::vector<double>> parameters) override;
    virtual const std::vector<std::string> getParameterNames() const override;
    virtual const std::vector<std::string> getParameterLabels() const override;
};

#endif // GEN_FTANTHETA_H
