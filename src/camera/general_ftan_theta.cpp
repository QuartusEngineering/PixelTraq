#include "camera/general_ftan_theta.h"
#include "camera/pinhole.h"
#include "utilities/common_math.h"
#include <iostream>

/**
 * @brief Parameterized constructor for the BrownConrady class.
 *
 * @param focal_length Vector of focal length values.
 * @param principal_point Vector of principal point values.
 * @param skew skew value.
 * @param image_size Vector of image size dimensions.
 * @param radial_distortion_num Vector of radial distortion numerator coefficients.
 * @param radial_distortion_den Vector of radial distortion denominator coefficients.
 * @param tangential_distortion Vector of tangential distortion coefficients.
 * @param tangential_distortion_polycoeff Vector of fourier tangential distortion polynomial coefficients.
 * @param tangential_distortion_OCVcoeff Vector of tangential distortion ocv coefficients.
 * @param rotation Rotation vector.
 * @param translation Translation vector.
 */
GenFTanTheta::GenFTanTheta(
    const std::vector<double>& focal_length,
    const std::vector<double>& principal_point,
    const double& skew,
    const std::vector<int>& image_size,
    const std::vector<double>& radial_distortion_num,
    const std::vector<double>& radial_distortion_den,
    const std::vector<double>& tangential_distortion,
    const std::vector<double>& tangential_distortion_polycoeff,
    const std::vector<double>& tangential_distortion_OCVcoeff,
    const Point3& rotation,
    const Point3& translation
)
    : Camera{ image_size, rotation, translation }, skew(skew), tangential_distortion(tangential_distortion)
{
    // Ensures that focal_length and principal_point have the right sizes, then sets fx, fy, cx, and cy.
    if (focal_length.size() == 2) {
        this->focal_length = focal_length;
    }
    else {
        throw std::invalid_argument("focal_length must have exactly 2 elements.");
    }

    if (principal_point.size() == 2) {
        this->principal_point = principal_point;
    }
    else {
        throw std::invalid_argument("principal_point must have exactly 2 elements.");
    }

    // preallocate full distortion polynomial coefficient arrays for speed
    this->radial_distortion_num = radial_distortion_num;
    this->radial_distortion_num.insert(this->radial_distortion_num.begin(), 1); //append 1 as first coefficient
    
    if (radial_distortion_den.size() != 0 && !CommonMath::isZero(radial_distortion_den))
    {
        this->radial_distortion_den = radial_distortion_den;
        this->radial_distortion_den.insert(this->radial_distortion_den.begin(), 1); //append 1 as first coefficient
        RADD = true;
    }
    if (tangential_distortion.size() != 0 && !CommonMath::isZero(tangential_distortion))
    {
        TANDIST = true;
    }
    if (tangential_distortion_polycoeff.size() != 0 && !CommonMath::isZero(tangential_distortion_polycoeff))
    {
        this->tangential_distortion_polycoeff = tangential_distortion_polycoeff;
        this->tangential_distortion_polycoeff.insert(this->tangential_distortion_polycoeff.begin(), 1); //append 1 as first coefficient
        TANPOLY = true;
    }
    if (tangential_distortion_OCVcoeff.size() != 0 && !CommonMath::isZero(tangential_distortion_OCVcoeff))
    {
        this->tangential_distortion_OCVcoeff_x = { 0,tangential_distortion_OCVcoeff[0],tangential_distortion_OCVcoeff[1] }; //append 0 as first coefficient
        this->tangential_distortion_OCVcoeff_y = { 0,tangential_distortion_OCVcoeff[2],tangential_distortion_OCVcoeff[3] }; //append 0 as first coefficient
        TANOCV = true;
    }
}

/**
 * @brief Copy constructor for the GenFTanTheta class.
 *
 * @param model The GenFTanTheta model to copy from.
 */
GenFTanTheta::GenFTanTheta(const GenFTanTheta& model) : Camera{ model.image_size, model.rotation, model.translation } // copy constructor
{
    focal_length = model.focal_length;
    principal_point = model.principal_point;
    skew = model.skew;
    radial_distortion_num = model.radial_distortion_num;
    radial_distortion_den = model.radial_distortion_den;
    tangential_distortion = model.tangential_distortion;
    tangential_distortion_polycoeff = model.tangential_distortion_polycoeff;
    tangential_distortion_OCVcoeff_x = model.tangential_distortion_OCVcoeff_x;
    tangential_distortion_OCVcoeff_y = model.tangential_distortion_OCVcoeff_y;
}

/**
 * @brief Projects a 3D point to a 2D point using intrinsic and extrinsic parameters.
 *
 * @param point_3d The 3D point to project.
 * @return The projected 2D point.
 */
std::array<double, 2>  GenFTanTheta::project(const Point3& point_3d) const {
    //normalize z component to extend vector to planar FPA
    double x = 1.0e12;
    double y = 1.0e12;

    if (point_3d[2] != 0.0) // handle divide by zero
    {
        x = point_3d[0] / point_3d[2];
        y = point_3d[1] / point_3d[2];
    }

    // evaluate distortion
    auto result = GenFTanTheta::evaluateDistortion({ x,y });
    double radial_scaling = result[0];
    double delta_x = result[1];
    double delta_y = result[2];

    // Apply radial and tangential distortion to calculate final projected points
    double yp = (y * radial_scaling + delta_y);
    double image_x = focal_length[0] * (x * radial_scaling + delta_x) + skew * yp + principal_point[0];
    double image_y = focal_length[1] * yp + principal_point[1];

    return { image_x, image_y };

}

/**
 * @brief Backprojects a 2D point to a 3D ray.
 *
 * @param point_2d The 2D point to backproject.
 * @return The backprojected 3D ray.
 */
Point3 GenFTanTheta::backproject(const std::array<double, 2>& point_2d) const {
    double x_distort = 1.0e12;
    double y_distort = 1.0e12;

    if (focal_length[1] != 0.0) // handle divide by zero
    {
        y_distort = (point_2d[1] - principal_point[1]) / focal_length[1];
    }
    if (focal_length[0] != 0.0) // handle divide by zero
    {
        x_distort = (point_2d[0] - principal_point[0] - skew * y_distort) / focal_length[0];
    }

    // Compensate for radial and tangential distortion
    double x = x_distort;
    double y = y_distort;
    double x_last = 1e6;
    double y_last = 1e6;

    int count = 0;
    do {
        // evaluate distortion
        auto result = GenFTanTheta::evaluateDistortion({ x,y });
        double radial_scaling = result[0];
        double delta_x = result[1];
        double delta_y = result[2];

        x_last = x;
        y_last = y;
        y = (y_distort - delta_y) / radial_scaling;
        x = (x_distort - delta_x) / radial_scaling;
        ++count;
    } while ((std::abs(x - x_last) > threshold || std::abs(y - y_last) > threshold) && count < iterations);

    double theta_r, tan_theta;
    tan_theta = std::sqrt(x * x + y * y);
    theta_r = std::atan(tan_theta);

    double phi_r = std::atan2(y, x);
    double ux = std::sin(theta_r) * std::cos(phi_r);
    double uy = std::sin(theta_r) * std::sin(phi_r);
    double uz = std::cos(theta_r);

    return { ux / uz, uy / uz, 1 }; // z normalization
}

/**
 * @brief Gets the focal length.
 *
 * @return The focal length.
 */
std::vector<double> GenFTanTheta::getFocalLength() const {
    return focal_length;
}

/**
 * @brief Gets the principal point.
 *
 * @return The principal point.
 */
std::vector<double> GenFTanTheta::getPrincipalPoint() const {
    return principal_point;
}

/**
 * @brief Gets the skew.
 *
 * @return The skew value.
 */
double GenFTanTheta::getSkew() const {
    return skew;
}

/**
 * @brief Gets the radial distortion numerator coefficients.
 * The radial distortion numerator is defined by the equation:
 * \f[
 * R_n = 1 + k_1 r^2 + k_2 r^4 + k_3 r^6 + \cdots
 * \f]
 * @return The radial distortion numerator coefficients.
 */
std::vector<double> GenFTanTheta::getRadialDistNumCoeffs() const {
    return std::vector<double>(radial_distortion_num.begin() + 1, radial_distortion_num.end());
}

/**
 * @brief Gets the radial distortion denominator coefficients.
 * The radial distortion denominator is defined by the equation:
 * \f[
 * R_d = 1 + k_1 r^2 + k_2 r^4 + k_3 r^6 + \cdots
 * \f]
 * @return The radial distortion denominator coefficients.
 */
std::vector<double> GenFTanTheta::getRadialDistDenCoeffs() const {
    return std::vector<double>(radial_distortion_den.begin() + 1, radial_distortion_den.end());
}

/**
 * @brief Gets the tangential distortion coefficients.
 *
 * @return The tangential distortion coefficients.
 */
std::vector<double> GenFTanTheta::getTangentialDistCoeffs() const {
    return tangential_distortion;
}

/**
 * @brief Gets the tangential polynomial distortion coefficients.
 *
 * @return The tangential polynomial distortion coefficients.
 */
std::vector<double> GenFTanTheta::getTangentialPolynominalDistCoeffs() const {
    return std::vector<double>(tangential_distortion_polycoeff.begin() + 1, tangential_distortion_polycoeff.end());
}

/**
 * @brief Gets the tangential distortion OCV coefficients.
 *
 * @return The tangential distortion OCV coefficients.
 */
std::vector<double> GenFTanTheta::getTangentialDistOCVCoeffs() const {
    
    std::vector<double> result = {};
    
    if ((tangential_distortion_OCVcoeff_x.size() != 0) && (tangential_distortion_OCVcoeff_y.size() != 0)) 
    {
        result = {
            tangential_distortion_OCVcoeff_x[0],
                tangential_distortion_OCVcoeff_x[1],
                tangential_distortion_OCVcoeff_y[0],
                tangential_distortion_OCVcoeff_y[1]
        };
    }

    return result;
}

/**
 * @brief Gets the model name.
 *
 * @return A string containing the model name.
 */
const std::string GenFTanTheta::getModelName() const {
    return model_name;
}

/**
 * @brief Gets a shared pointer to a Pinhole model.
 *
 * @return A shared pointer to a Pinhole model.
 */
std::shared_ptr<Pinhole> GenFTanTheta::getPinhole() const {
    return std::make_shared<Pinhole>(focal_length, principal_point, skew, image_size, rotation, translation);
}

/**
 * @brief Sets the focal length.
 *
 * @param focal_length A vector containing the new focal length values.
 */
void GenFTanTheta::setFocalLength(std::vector<double> focal_length)
{
    if (focal_length.size() == 2) {
        this->focal_length = focal_length;
    }
    else {
        throw std::invalid_argument("focal_length vector must have exactly 2 elements.");
    }
}

/**
 * @brief Sets the principal point.
 *
 * @param principal_point A vector containing the new principal point values.
 */
void GenFTanTheta::setPrincipalPoint(std::vector<double> principal_point)
{
    if (principal_point.size() == 2) {
        this->principal_point = principal_point;  // Update internal vector
    }
    else {
        throw std::invalid_argument("principal_point vector must have exactly 2 elements.");
    }
}

/**
 * @brief Sets the skew value.
 *
 * @param skew A double containing the new skew value.
 */
void GenFTanTheta::setSkew(double skew)
{
    this->skew = skew;
}

/**
 * @brief Sets the radial distortion numerator coefficients.
 *
 * @param radial_dist_num_coeffs A vector containing the new radial distortion numerator coefficients.
 */
void GenFTanTheta::setRadialDistNumCoeffs(std::vector<double> radial_dist_num_coeffs)
{
    std::vector<double> radial_dist_num_coeffs_ = std::move(radial_dist_num_coeffs);
    radial_dist_num_coeffs_.insert(radial_dist_num_coeffs_.begin(), 1);
    this->radial_distortion_num = radial_dist_num_coeffs_;
}

/**
 * @brief Sets the radial distortion denominator coefficients.
 *
 * @param radial_dist_den_coeffs A vector containing the new radial distortion denominator coefficients.
 */
void GenFTanTheta::setRadialDistDenCoeffs(std::vector<double> radial_dist_den_coeffs)
{
    std::vector<double> radial_dist_den_coeffs_ = std::move(radial_dist_den_coeffs);
    radial_dist_den_coeffs_.insert(radial_dist_den_coeffs_.begin(), 1);
    this->radial_distortion_den = radial_dist_den_coeffs_;

    if (radial_distortion_den.size() != 0 && !CommonMath::isZero(radial_distortion_den))
    {
        this->RADD = true;
    }
    else
    {
        this->RADD = false;
    };
}

/**
 * @brief Sets the tangential distortion coefficients.
 *
 * @param tangential_dist_coeffs A vector containing the new tangential distortion coefficients.
 */
void GenFTanTheta::setTangentialDistCoeffs(std::vector<double> tangential_dist_coeffs)
{
    this->tangential_distortion = tangential_dist_coeffs;

    if (tangential_distortion.size() != 0 && !CommonMath::isZero(tangential_distortion))
    {
        this->TANDIST = true;
    }
    else
    {
        this->TANDIST = false;
    };
}

/**
 * @brief Sets the tangential polynomial distortion coefficients.
 *
 * @param tangential_polynomial_dist_coeffs A vector containing the new tangential polynomial distortion coefficients.
 */
void GenFTanTheta::setTangentialPolynominalDistCoeffs(std::vector<double> tangential_polynomial_dist_coeffs)
{
    std::vector<double> tangential_distortion_polycoeff_ = std::move(tangential_polynomial_dist_coeffs);
    tangential_distortion_polycoeff_.insert(tangential_distortion_polycoeff_.begin(), 1);
    this->tangential_distortion_polycoeff = tangential_distortion_polycoeff_;

    if (tangential_distortion_polycoeff.size() != 0 && !CommonMath::isZero(tangential_distortion_polycoeff))
    {
        this->TANPOLY = true;
    }
    else
    {
        this->TANPOLY = false;
    };
}

/**
 * @brief Sets the tangential distortion OCV coefficients.
 *
 * @param tangential_dist_ocv A vector containing the new tangential distortion OCV coefficients.
 */
void GenFTanTheta::setTangentialDistOCVCoeffs(std::vector<double> tangential_dist_ocv)
{
    if (tangential_dist_ocv.size() % 4 == 0)
    {
        this->tangential_distortion_OCVcoeff_x = { tangential_dist_ocv[0], tangential_dist_ocv[1] };
        this->tangential_distortion_OCVcoeff_y = { tangential_dist_ocv[2], tangential_dist_ocv[3] };
        this->TANOCV = true;
    }
    else if (tangential_dist_ocv.size() == 0)
    {
        this->tangential_distortion_OCVcoeff_x = {};
        this->tangential_distortion_OCVcoeff_x = {};
        this->TANOCV = false;
    }
    else
    {
        throw std::invalid_argument("tangential_dist_ocv vector must have a length equal to 4");
    }
}

/**
 * @brief Sets the backprojection settings.
 *
 * @param threshold The new threshold value.
 * @param iterations The new number of iterations.
 */
void GenFTanTheta::setBackprojectSettings(double threshold, int iterations)
{
    if (threshold > 0.0)
    {
        this->threshold = threshold;
    }
    else
    {
        throw std::invalid_argument("threshold must be a number greater than zero");
    }
    if (iterations > 0)
    {
        this->iterations = iterations;
    }
    else
    {
        throw std::invalid_argument("iterations must be an integer greater than zero");
    }
}

/**
 * @brief Loads a GenFTanTheta model from a file.
 *
 * @param fileName The path of the file to load the model from.
 * @return A shared pointer to the loaded GenFTanTheta model.
 * @throws std::invalid_argument if the model contained in the imported file is not a GenFTanTheta model.
 */
std::shared_ptr<GenFTanTheta> GenFTanTheta::load(const std::string& fileName)
{
    auto camera_temp = Camera::load(fileName);
    auto type = camera_temp->getModelName();
    if (type == "General FTan Theta")
    {
        std::static_pointer_cast<GenFTanTheta>(camera_temp);
    }
    else
    {
        throw std::invalid_argument("Model contained in the imported file is not a General FTan Theta Model.");
    }
}

// private methods
std::array<double, 3> GenFTanTheta::evaluateDistortion(const std::array<double, 2>& pt) const {
    double x = pt[0];
    double y = pt[1];
    double r2 = x * x + y * y;

    // Calculate radial and tangential distortion scaling factors
    double radial_distfun_num = CommonMath::evaluatePolynomial(radial_distortion_num, r2);
    double radial_scaling = radial_distfun_num;
    if (RADD)
    {
        radial_scaling /= CommonMath::evaluatePolynomial(radial_distortion_den, r2);
    }

    // Tangential polynomial terms
    double tan_poly_distfun = 0;
    if (TANPOLY)
    {
        tan_poly_distfun = CommonMath::evaluatePolynomial(tangential_distortion_polycoeff, r2);
    }

    // OpenCV Style Terms
    double tan_poly_distfun_x = 0;
    double tan_poly_distfun_y = 0;
    if (TANOCV)
    {
        tan_poly_distfun_x = CommonMath::evaluatePolynomial(tangential_distortion_OCVcoeff_x, r2);
        tan_poly_distfun_y = CommonMath::evaluatePolynomial(tangential_distortion_OCVcoeff_y, r2);
    }

    // apply tangential distortion
    double delta_x = 0;
    double delta_y = 0;
    if (TANDIST)
    {
        delta_x = (2 * tangential_distortion[0] * x * y + tangential_distortion[1] * (r2 + 2 * x * x)) * tan_poly_distfun + tan_poly_distfun_x;
        delta_y = (tangential_distortion[0] * (r2 + 2 * y * y) + 2 * tangential_distortion[1] * x * y) * tan_poly_distfun + tan_poly_distfun_y;
    }

    return { radial_scaling , delta_x, delta_y };
}

const std::vector<std::vector<double>> GenFTanTheta::getParameters() const {
    return {
        getFocalLength(),
        getPrincipalPoint(),
        {getSkew()},
        getRadialDistNumCoeffs(),
        getRadialDistDenCoeffs(),
        getTangentialDistCoeffs(),
        getTangentialPolynominalDistCoeffs(),
        getTangentialDistOCVCoeffs()
    };
}

void GenFTanTheta::getBackprojectSettings(double& threshold, int& iterations) const
{
    threshold = this->threshold;
    iterations = this->iterations;
}

void GenFTanTheta::setParameters(std::vector<std::vector<double>> parameters)
{
    setFocalLength(parameters[0]);
    setPrincipalPoint(parameters[1]);
    setSkew(parameters[2][0]);
    setRadialDistNumCoeffs(parameters[3]);
    setRadialDistDenCoeffs(parameters[4]);
    setTangentialDistCoeffs(parameters[5]);
    setTangentialPolynominalDistCoeffs(parameters[6]);
    setTangentialDistOCVCoeffs(parameters[7]);
}

const std::vector<std::string> GenFTanTheta::getParameterNames() const {
    return parameter_names;
}

const std::vector<std::string> GenFTanTheta::getParameterLabels() const {
    return parameter_file_labels;
}