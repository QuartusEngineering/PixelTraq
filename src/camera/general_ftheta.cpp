#include "camera/general_ftheta.h"
#include "camera/pinhole.h"
#include "utilities/common_math.h"
#include <iostream>
#include <math.h>

/**
 * @brief Parameterized constructor for the Kannala class.
 *
 * @param focal_length Vector of focal length values.
 * @param principal_point Vector of principal point values.
 * @param skew skew value.
 * @param image_size Vector of image size dimensions.
 * @param radial_distortion_sym Vector of symmetric radial distortion coefficients.
 * @param radial_distortion_asym Vector of asymmetric radial distortion coefficients.
 * @param radial_distortion_four Vector of fourier radial distortion coefficients.
 * @param tangential_distortion_asym Vector of asymmetric tangential distortion coefficients.
 * @param tangential_distortion_four Vector of fourier tangential distortion coefficients.
 * @param rotation Rotation vector.
 * @param translation Translation vector.
 */
GenFTheta::GenFTheta(
    const std::vector<double>& focal_length,
    const std::vector<double>& principal_point,
    const double& skew,
    const std::vector<int>& image_size,
    const std::vector<double>& radial_distortion_sym,
    const std::vector<double>& radial_distortion_asym,
    const std::vector<double>& radial_distortion_four,
    const std::vector<double>& tangential_distortion_asym,
    const std::vector<double>& tangential_distortion_four,
    const Point3& rotation,
    const Point3& translation
)
    : Camera{ image_size, rotation, translation }, skew(skew)
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
    this->radial_distortion_sym = radial_distortion_sym;
    this->radial_distortion_sym.insert(this->radial_distortion_sym.begin(), 1); //append 1 as first coefficient

    if ((radial_distortion_asym.size() != 0 && !CommonMath::isZero(radial_distortion_asym)) ||
        (tangential_distortion_asym.size() != 0 && !CommonMath::isZero(tangential_distortion_asym)) ||
        (radial_distortion_four.size() != 0 && !CommonMath::isZero(radial_distortion_four)) ||
        (tangential_distortion_four.size() != 0 && !CommonMath::isZero(tangential_distortion_four)))
    {
        this->radial_distortion_asym = radial_distortion_asym;
        this->tangential_distortion_asym = tangential_distortion_asym;

        if (radial_distortion_four.size() % 2 == 0)
        {
            this->radial_distortion_four = radial_distortion_four;
        }
        else
        {
            throw std::invalid_argument("radial_distortion_four vector must have a length that is a multiple of 2");
        }

        if (tangential_distortion_four.size() % 2 == 0)
        {
            this->tangential_distortion_four = tangential_distortion_four;
        }
        else
        {
            throw std::invalid_argument("radial_distortion_four vector must have a length that is a multiple of 2");
        }

        FULL = true;
    }
}

/**
 * @brief Copy constructor for the Kannala class.
 *
 * @param model The Kannala model to copy from.
 */
GenFTheta::GenFTheta(const GenFTheta& model) : Camera{ model.image_size, model.rotation, model.translation } // copy constructor
{
    focal_length = model.focal_length;
    principal_point = model.principal_point;
    skew = model.skew;
    radial_distortion_sym = model.radial_distortion_sym;
    radial_distortion_asym = model.radial_distortion_asym;
    radial_distortion_four = model.radial_distortion_four;
    tangential_distortion_asym = model.tangential_distortion_asym;
    tangential_distortion_four = model.tangential_distortion_four;
}

/**
 * @brief Projects a 3D point to a 2D point using intrinsic and extrinsic parameters.
 *
 * @param point_3d The 3D point to project.
 * @return The projected 2D point.
 */
std::array<double, 2>  GenFTheta::project(const Point3& point_3d) const {
    //normalize z component to extend vector to planar FPA
    double x = 1.0e12;
    double y = 1.0e12;

    if (point_3d[2] != 0.0) // handle divide by zero
    {
        x = point_3d[0] / point_3d[2];
        y = point_3d[1] / point_3d[2];
    }
    else
    {
        return { 1.0e12, 1.0e12 };
    }

    // evaluate distortion
    auto result = GenFTheta::evaluateDistortion({ x,y });
    double radial_scaling = result[0];
    double tangential_scaling = result[1];
    double ftheta_scaling = result[2];
    // apply ftheta projection
    x = x * ftheta_scaling;
    y = y * ftheta_scaling;

    // Apply radial and tangential distortion to calculate final projected points
    double yp = (y * radial_scaling + x * tangential_scaling);
    double image_x = focal_length[0] * (x * radial_scaling - y * tangential_scaling) + skew * yp + principal_point[0];
    double image_y = focal_length[1] * yp + principal_point[1];

    return { image_x, image_y };

}

/**
 * @brief Backprojects a 2D point to a 3D ray.
 *
 * @param point_2d The 2D point to backproject.
 * @return The backprojected 3D ray.
 */
Point3 GenFTheta::backproject(const std::array<double, 2>& point_2d) const {
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

    if (x_distort == 0.0 && y_distort == 0.0) // handle divide by zero
    {
        return { 0.0 , 0.0 , 1 };
    }

    // Compensate for radial and tangential distortion
    double x = x_distort;
    double y = y_distort;
    double x_last = 1e6;
    double y_last = 1e6;
    
    int count = 0;
    do {
        // evaluate distortion
        auto result = GenFTheta::evaluateDistortion({ x,y });
        double radial_scaling = result[0];
        double tangential_scaling = result[1];
        double ftheta_scaling = result[2];

        x_last = x;
        y_last = y;
        double  det = (radial_scaling * radial_scaling + tangential_scaling * tangential_scaling);
        x = (radial_scaling * x_distort + tangential_scaling * y_distort) / (det * ftheta_scaling);
        y = (radial_scaling * y_distort - tangential_scaling * x_distort) / (det * ftheta_scaling);
        ++count;
    } while ((std::abs(x - x_last) > threshold || std::abs(y - y_last) > threshold) && count < iterations);

    double theta_r, r_xy;
    r_xy = std::sqrt(x * x + y * y);
    theta_r = std::atan(r_xy);

    double phi_r = std::atan2(y, x);
    double ux = std::sin(theta_r) * std::cos(phi_r);
    double uy = std::sin(theta_r) * std::sin(phi_r);
    double uz = std::cos(theta_r);

    return { ux / uz , uy / uz , 1 };
}

/**
 * @brief Gets the focal length.
 *
 * @return The focal length.
 */
std::vector<double> GenFTheta::getFocalLength() const {
    return focal_length;
}

/**
 * @brief Gets the principal point.
 *
 * @return The principal point.
 */
std::vector<double> GenFTheta::getPrincipalPoint() const {
    return principal_point;
}

/**
 * @brief Gets the skew.
 *
 * @return The skew value.
 */
double GenFTheta::getSkew() const {
    return skew;
}

/**
 * @brief Gets the symmetric radial distortion coefficients.
 * The radial distortion is defined by the equation:
 * \f[
 * R_n = 1 + k_1 r^2 + k_2 r^4 + k_3 r^6 + \cdots
 * \f]
 * @return The symmetric radial distortion coefficients.
 */
std::vector<double> GenFTheta::getRadialDistSymCoeffs() const {
    return std::vector<double>(radial_distortion_sym.begin() + 1, radial_distortion_sym.end());
}

/**
 * @brief Gets the asymmetric radial distortion coefficients.
 *
 * @return The asymmetric radial distortion coefficients.
 */
std::vector<double> GenFTheta::getRadialDistAsymCoeffs() const {
    return radial_distortion_asym;
}

/**
 * @brief Gets the fourier radial distortion coefficients.
 *
 * @return The fourier radial distortion coefficients.
 */
std::vector<double> GenFTheta::getRadialDistFourCoeffs() const {
    return radial_distortion_four;
}

/**
 * @brief Gets the asymmetric tangential distortion coefficients.
 *
 * @return The asymmetric tangential distortion coefficients.
 */
std::vector<double> GenFTheta::getTangentialDistAsymCoeffs() const {
    return tangential_distortion_asym;
}

/**
 * @brief Gets the fourier tangential distortion coefficients.
 *
 * @return The fourier tangential distortion coefficients.
 */
std::vector<double> GenFTheta::getTangentialDistFourCoeffs() const {
    return tangential_distortion_four;
}

/**
 * @brief Gets the backprojection settings.
 *
 * @param threshold The threshold value to be set.
 * @param iterations The number of iterations to be set.
 */
void GenFTheta::getBackprojectSettings(double& threshold, int& iterations) const
{
    threshold = this->threshold;
    iterations = this->iterations;
}

/**
 * @brief Gets the model name.
 *
 * @return A string containing the model name.
 */
const std::string GenFTheta::getModelName() const {
    return model_name;
}

/**
 * @brief Gets a shared pointer to a Pinhole model.
 *
 * @return A shared pointer to a Pinhole model.
 */
std::shared_ptr<Pinhole> GenFTheta::getPinhole() const {
    return std::make_shared<Pinhole>(focal_length, principal_point, skew, image_size, rotation, translation);
}

/**
 * @brief Sets the focal length.
 *
 * @param focal_length A vector containing the new focal length values.
 */
void GenFTheta::setFocalLength(std::vector<double> focal_length) {
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
void GenFTheta::setPrincipalPoint(std::vector<double> principal_point) {
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
void GenFTheta::setSkew(double skew) {
    this->skew = skew;
}

/**
 * @brief Sets the symmetric radial distortion coefficients.
 *
 * @param radial_dist_sym_coeffs A vector containing the new symmetric radial distortion coefficients.
 */
void GenFTheta::setRadialDistSymCoeffs(std::vector<double> radial_dist_sym_coeffs) {
    auto radial_dist_sym_coeffs_ = radial_dist_sym_coeffs;
    radial_dist_sym_coeffs_.insert(radial_dist_sym_coeffs_.begin(), 1);
    this->radial_distortion_sym = radial_dist_sym_coeffs_;
}

/**
 * @brief Sets the asymmetric radial distortion coefficients.
 *
 * @param radial_dist_asym_coeffs A vector containing the new asymmetric radial distortion coefficients.
 */
void GenFTheta::setRadialDistAsymCoeffs(std::vector<double> radial_dist_asym_coeffs) {
    this->radial_distortion_asym = radial_dist_asym_coeffs;
    if (radial_distortion_asym.size() != 0 && !CommonMath::isZero(radial_distortion_asym))
    {
        this->FULL = true;
    }
    else
    {
        this->FULL = false;
    }
}

/**
 * @brief Sets the four-term radial distortion coefficients.
 *
 * @param radial_dist_four_coeffs A vector containing the new four-term radial distortion coefficients.
 */
void GenFTheta::setRadialDistFourCoeffs(std::vector<double> radial_dist_four_coeffs) {
    if (radial_dist_four_coeffs.size() % 2 == 0)
    {
        this->radial_distortion_four = radial_dist_four_coeffs;
        if (radial_distortion_four.size() != 0 && !CommonMath::isZero(radial_distortion_four))
        {
            this->FULL = true;
        }
        else
        {
            this->FULL = false;
        }
    }
    else
    {
        throw std::invalid_argument("radial_distortion_four vector must have a length that is a multiple of 2");
    }
}

/**
 * @brief Sets the asymmetric tangential distortion coefficients.
 *
 * @param tangential_dist_asym_coeffs A vector containing the new asymmetric tangential distortion coefficients.
 */
void GenFTheta::setTangentialDistAsymCoeffs(std::vector<double> tangential_dist_asym_coeffs) {
    this->tangential_distortion_asym = tangential_dist_asym_coeffs;
    if (tangential_distortion_asym.size() != 0 && !CommonMath::isZero(tangential_distortion_asym))
    {
        this->FULL = true;
    }
    else
    {
        this->FULL = false;
    }
}

/**
 * @brief Sets the four-term tangential distortion coefficients.
 *
 * @param tangential_dist_four_coeffs A vector containing the new four-term tangential distortion coefficients.
 */
void GenFTheta::setTangentialDistFourCoeffs(std::vector<double> tangential_dist_four_coeffs) {
    if (tangential_dist_four_coeffs.size() % 2 == 0)
    {
        this->tangential_distortion_four = tangential_dist_four_coeffs;
        if (tangential_distortion_four.size() != 0 && !CommonMath::isZero(tangential_distortion_four))
        {
            this->FULL = true;
        }
        else
        {
            this->FULL = false;
        }
    }
    else
    {
        throw std::invalid_argument("radial_distortion_four vector must have a length that is a multiple of 2");
    }
}

/**
 * @brief Sets the backprojection settings.
 *
 * @param threshold The new threshold value.
 * @param iterations The new number of iterations.
 */
void GenFTheta::setBackprojectSettings(double threshold, int iterations)
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
 * @brief Loads a GenFTheta model from a file.
 *
 * @param fileName The path of the file to load the model from.
 * @return A shared pointer to the loaded GenFTheta model.
 * @throws std::invalid_argument if the model contained in the imported file is not a GenFTheta model.
 */
std::shared_ptr<GenFTheta> GenFTheta::load(const std::string& fileName)
{
    auto camera_temp = Camera::load(fileName);
    auto type = camera_temp->getModelName();
    if (type == "General FTheta")
    {
        std::static_pointer_cast<GenFTheta>(camera_temp);
    }
    else
    {
        throw std::invalid_argument("Model contained in the imported file is not a General FTheta Model.");
    }
}

// private methods
std::array<double, 3> GenFTheta::evaluateDistortion(const std::array<double, 2>& pt) const {
    double x = pt[0];
    double y = pt[1];
    double theta, r_xy, theta2;
    r_xy = std::sqrt(x * x + y * y);
    theta = std::atan(r_xy);
    theta2 = theta * theta;

    // Conversion from ftantheta to ftheta
    double ftheta_scaling = 0;
    if (r_xy != 0.0) // handle divide by zero
    {
        ftheta_scaling = theta / r_xy;
    }

    // Calculate radial and tangential distortion scaling factors
    double radial_scaling = CommonMath::evaluatePolynomial(radial_distortion_sym, theta2);
    double tangential_scaling = 0;
    double radial_asym_four = 1;
    double tangential_asym_four = 1;
    if (FULL)
    {
        double phi = std::atan2(y, x);
        radial_scaling += CommonMath::evaluatePolynomial(radial_distortion_asym, theta2) * CommonMath::evaluateFourier(radial_distortion_four, phi);
        tangential_scaling = CommonMath::evaluatePolynomial(tangential_distortion_asym, theta2) * CommonMath::evaluateFourier(tangential_distortion_four, phi);
    }

    return { radial_scaling , tangential_scaling , ftheta_scaling };
}

const std::vector<std::vector<double>> GenFTheta::getParameters() const {
    return {
        getFocalLength(),
        getPrincipalPoint(),
        { getSkew() },
        getRadialDistSymCoeffs(),
        getRadialDistAsymCoeffs(),
        getRadialDistFourCoeffs(),
        getTangentialDistAsymCoeffs(),
        getTangentialDistFourCoeffs()
    };
}

void GenFTheta::setParameters(std::vector<std::vector<double>> parameters) {
    setFocalLength(parameters[0]);
    setPrincipalPoint(parameters[1]);
    setSkew(parameters[2][0]);
    setRadialDistSymCoeffs(parameters[3]);
    setRadialDistAsymCoeffs(parameters[4]);
    setRadialDistFourCoeffs(parameters[5]);
    setTangentialDistAsymCoeffs(parameters[6]);
    setTangentialDistFourCoeffs(parameters[7]);
}

const std::vector<std::string> GenFTheta::getParameterNames() const {
    return parameter_names;
}

const std::vector<std::string> GenFTheta::getParameterLabels() const {
    return parameter_file_labels;
}