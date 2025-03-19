#include "camera/brown_conrady.h"
#include <iostream>
#include <stdexcept>
#include "utilities/common_math.h"

/**
 * @brief Default constructor for the BrownConrady class.
 */
BrownConrady::BrownConrady()
{
    this->internalModel = std::make_unique<GenFTanTheta>();
}

/**
 * @brief Parameterized constructor for the BrownConrady class.
 *
 * @param focal_length Vector of focal length values.
 * @param principal_point Vector of principal point values.
 * @param image_size Vector of image size dimensions.
 * @param radial_distortion Vector of symmetric radial distortion coefficients.
 * @param tangential_distortion Vector of tangential distortion coefficients.
 * @param tangential_distortion_polycoeff Vector of fourier tangential distortion polynomial coefficients.
 * @param rotation Rotation vector.
 * @param translation Translation vector.
 */
BrownConrady::BrownConrady(
    const std::vector<double>& focal_length,
    const std::vector<double>& principal_point,
    const std::vector<int>& image_size,
    const std::vector<double>& radial_distortion,
    const std::vector<double>& tangential_distortion,
    const std::vector<double>& tangential_distortion_polycoeff,
    const Point3& rotation,
    const Point3& translation
)
    : Camera{ image_size, rotation, translation }
{
    std::vector<double> empty_list = {};
    this->internalModel = std::make_unique<GenFTanTheta>(
        focal_length, principal_point, 0, image_size, radial_distortion, empty_list,
        tangential_distortion, tangential_distortion_polycoeff, empty_list, rotation, translation
    );
}

/**
 * @brief Copy constructor for the BrownConrady class.
 *
 * @param model The BrownConrady model to copy from.
 */
BrownConrady::BrownConrady(const BrownConrady& model) : Camera{ model.image_size, model.rotation, model.translation } // copy constructor
{
    this->internalModel = std::make_unique<GenFTanTheta>();
    *(this->internalModel) = *(model.internalModel);
}

/**
 * @brief Projects a 3D point to a 2D point using intrinsic and extrinsic parameters.
 *
 * @param point_3d The 3D point to project.
 * @return The projected 2D point.
 */
std::array<double, 2>  BrownConrady::project(const Point3& point_3d) const {
    return internalModel->project(point_3d); 
}

/**
 * @brief Backprojects a 2D point to a 3D ray.
 *
 * @param point_2d The 2D point to backproject.
 * @return The backprojected 3D ray.
 */
Point3 BrownConrady::backproject(const std::array<double, 2>& point_2d) const {
    return internalModel->backproject(point_2d);
}

/**
 * @brief Gets the focal length.
 *
 * @return The focal length.
 */
std::vector<double> BrownConrady::getFocalLength() const {
    return internalModel->getFocalLength();
}

/**
 * @brief Gets the principal point.
 *
 * @return The principal point.
 */
std::vector<double> BrownConrady::getPrincipalPoint() const {
    return internalModel->getPrincipalPoint();
}

/**
 * @brief Gets the radial distortion coefficients.
 * The radial distortion is defined by the equation:
 * \f[
 * R_n = 1 + k_1 r^2 + k_2 r^4 + k_3 r^6 + \cdots
 * \f]
 * @return The radial distortion coefficients.
 */
std::vector<double> BrownConrady::getRadialDistCoeffs() const {
    return internalModel->getRadialDistNumCoeffs();
}

/**
 * @brief Gets the tangential distortion coefficients.
 *
 * @return The tangential distortion coefficients.
 */
std::vector<double> BrownConrady::getTangentialDistCoeffs() const {
    return internalModel->getTangentialDistCoeffs();
}

/**
 * @brief Gets the tangential polynomial distortion coefficients.
 *
 * @return The tangential polynomial distortion coefficients.
 */
std::vector<double> BrownConrady::getTangentialPolynominalDistCoeffs() const {
    return internalModel->getTangentialPolynominalDistCoeffs();
}

/**
 * @brief Gets the backprojection settings.
 *
 * @param threshold The threshold value to be set.
 * @param iterations The number of iterations to be set.
 */
void BrownConrady::getBackprojectSettings(double& threshold, int& iterations) const
{
    internalModel->getBackprojectSettings(threshold, iterations);
}

/**
 * @brief Gets the model name.
 *
 * @return A string containing the model name.
 */
const std::string BrownConrady::getModelName() const {
    return model_name;
}

/**
 * @brief Gets a shared pointer to a Pinhole model.
 *
 * @return A shared pointer to a Pinhole model.
 */
std::shared_ptr<Pinhole> BrownConrady::getPinhole() const{
    return std::make_shared<Pinhole>(getFocalLength(), getPrincipalPoint(), 0, getImageSize(), getRotation(), getTranslation());
}

/**
 * @brief Sets the focal length.
 *
 * @param focal_length A vector containing the new focal length values.
 */
void BrownConrady::setFocalLength(std::vector<double> focal_length)
{
    internalModel->setFocalLength(focal_length);
}

/**
 * @brief Sets the principal point.
 *
 * @param principal_point A vector containing the new principal point values.
 */
void BrownConrady::setPrincipalPoint(std::vector<double> principal_point)
{
    internalModel->setPrincipalPoint(principal_point);
}

/**
 * @brief Sets the radial distortion coefficients.
 *
 * @param radial_dist_coeffs A vector containing the new radial distortion coefficients.
 */
void BrownConrady::setRadialDistCoeffs(std::vector<double> radial_dist_coeffs)
{
    internalModel->setRadialDistNumCoeffs(radial_dist_coeffs);
}

/**
 * @brief Sets the tangential distortion coefficients.
 *
 * @param tangential_dist_coeffs A vector containing the new tangential distortion coefficients.
 */
void BrownConrady::setTangentialDistCoeffs(std::vector<double> tangential_dist_coeffs)
{
    internalModel->setTangentialDistCoeffs(tangential_dist_coeffs);
}

/**
 * @brief Sets the tangential polynomial distortion coefficients.
 *
 * @param tangential_polynomial_dist_coeffs A vector containing the new tangential polynomial distortion coefficients.
 */
void BrownConrady::setTangentialPolynominalDistCoeffs(std::vector<double> tangential_polynomial_dist_coeffs)
{
    internalModel->setTangentialPolynominalDistCoeffs(tangential_polynomial_dist_coeffs);
}

/**
 * @brief Sets the backprojection settings.
 *
 * @param threshold The new threshold value.
 * @param iterations The new number of iterations.
 */
void BrownConrady::setBackprojectSettings(double threshold, int iterations)
{
    internalModel->setBackprojectSettings(threshold, iterations);
}

/**
 * @brief Loads a BrownConrady model from a file.
 *
 * @param fileName The path of the file to load the model from.
 * @return A shared pointer to the loaded BrownConrady model.
 * @throws std::invalid_argument if the model contained in the imported file is not a BrownConrady model.
 */
std::shared_ptr<BrownConrady> BrownConrady::load(const std::string& fileName)
{
    auto camera_temp = Camera::load(fileName);
    auto type = camera_temp->getModelName();
    if (type == "Brown Conrady")
    {
        return std::static_pointer_cast<BrownConrady>(camera_temp);
    }
    else
    {
        throw std::invalid_argument("Model contained in the imported file is not a Brown Conrady Model.");
    }
}

// private methods
const std::vector<std::vector<double>> BrownConrady::getParameters() const {
    return {
        getFocalLength(),
        getPrincipalPoint(),
        getRadialDistCoeffs(),
        getTangentialDistCoeffs(),
        getTangentialPolynominalDistCoeffs()
    };
}

void BrownConrady::setParameters(std::vector<std::vector<double>> parameters)
{
    setFocalLength(parameters[0]);
    setPrincipalPoint(parameters[1]);
    setRadialDistCoeffs(parameters[2]);
    setTangentialDistCoeffs(parameters[3]);
    setTangentialPolynominalDistCoeffs(parameters[4]);
}

const std::vector<std::string> BrownConrady::getParameterNames() const {
    return parameter_names;
}

const std::vector<std::string> BrownConrady::getParameterLabels() const {
    return parameter_file_labels;
}
