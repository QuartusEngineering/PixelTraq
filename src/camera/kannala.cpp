#include "camera/kannala.h"
#include <iostream>

/**
 * @brief Default constructor for the Kannala class.
 */
Kannala::Kannala()
{
    this->internalModel = std::make_unique<GenFTheta>();
}

/**
 * @brief Parameterized constructor for the Kannala class.
 *
 * @param focal_length Vector of focal length values.
 * @param principal_point Vector of principal point values.
 * @param image_size Vector of image size dimensions.
 * @param radial_distortion_sym Vector of symmetric radial distortion coefficients.
 * @param radial_distortion_asym Vector of asymmetric radial distortion coefficients.
 * @param radial_distortion_four Vector of fourier radial distortion coefficients.
 * @param tangential_distortion_asym Vector of asymmetric tangential distortion coefficients.
 * @param tangential_distortion_four Vector of fourier tangential distortion coefficients.
 * @param rotation Rotation vector.
 * @param translation Translation vector.
 */
Kannala::Kannala(
    const std::vector<double>& focal_length,
    const std::vector<double>& principal_point,
    const std::vector<int>& image_size,
    const std::vector<double>& radial_distortion_sym,
    const std::vector<double>& radial_distortion_asym,
    const std::vector<double>& radial_distortion_four,
    const std::vector<double>& tangential_distortion_asym,
    const std::vector<double>& tangential_distortion_four,
    const Point3& rotation,
    const Point3& translation
)
    : Camera{ image_size, rotation, translation }
{
    this->internalModel = std::make_unique<GenFTheta>(
        focal_length, principal_point, 0, image_size, radial_distortion_sym, radial_distortion_asym,
        radial_distortion_four, tangential_distortion_asym, tangential_distortion_four, rotation, translation
    );
}

/**
 * @brief Copy constructor for the Kannala class.
 *
 * @param model The Kannala model to copy from.
 */
Kannala::Kannala(const Kannala& model) : Camera{ model.image_size, model.rotation, model.translation } // copy constructor
{
    this->internalModel = std::make_unique<GenFTheta>();
    *(this->internalModel) = *(model.internalModel);
}

/**
 * @brief Projects a 3D point to a 2D point using intrinsic and extrinsic parameters.
 *
 * @param point_3d The 3D point to project.
 * @return The projected 2D point.
 */
std::array<double, 2>  Kannala::project(const Point3& point_3d) const {
    return internalModel->project(point_3d);
}

/**
 * @brief Backprojects a 2D point to a 3D ray.
 *
 * @param point_2d The 2D point to backproject.
 * @return The backprojected 3D ray.
 */
Point3  Kannala::backproject(const std::array<double, 2>& point_2d) const {
    return internalModel->backproject(point_2d);
}

/**
 * @brief Gets the focal length.
 *
 * @return The focal length.
 */
std::vector<double> Kannala::getFocalLength() const {
    return internalModel->getFocalLength();
}

/**
 * @brief Gets the principal point.
 *
 * @return The principal point.
 */
std::vector<double> Kannala::getPrincipalPoint() const {
    return internalModel->getPrincipalPoint();
}

/**
 * @brief Gets the symmetric radial distortion coefficients.
 * The radial distortion is defined by the equation:
 * \f[
 * R_n = 1 + k_1 r^2 + k_2 r^4 + k_3 r^6 + \cdots
 * \f]
 * @return The symmetric radial distortion coefficients.
 */
std::vector<double> Kannala::getRadialDistSymCoeffs() const {
    return internalModel->getRadialDistSymCoeffs();
}

/**
 * @brief Gets the asymmetric radial distortion coefficients.
 * 
 * @return The asymmetric radial distortion coefficients.
 */
std::vector<double> Kannala::getRadialDistAsymCoeffs() const {
    return internalModel->getRadialDistAsymCoeffs();
}

/**
 * @brief Gets the fourier radial distortion coefficients.
 *
 * @return The fourier radial distortion coefficients.
 */
std::vector<double> Kannala::getRadialDistFourCoeffs() const {
    return internalModel->getRadialDistFourCoeffs();
}

/**
 * @brief Gets the asymmetric tangential distortion coefficients.
 *
 * @return The asymmetric tangential distortion coefficients.
 */
std::vector<double> Kannala::getTangentialDistAsymCoeffs() const {
    return internalModel->getTangentialDistAsymCoeffs();
}

/**
 * @brief Gets the fourier tangential distortion coefficients.
 *
 * @return The fourier tangential distortion coefficients.
 */
std::vector<double> Kannala::getTangentialDistFourCoeffs() const {
    return internalModel->getTangentialDistFourCoeffs();
}

/**
 * @brief Gets the backprojection settings.
 *
 * @param threshold The threshold value to be set.
 * @param iterations The number of iterations to be set.
 */
void Kannala::getBackprojectSettings(double &threshold, int &iterations) const
{
    internalModel->getBackprojectSettings(threshold, iterations);
}

/**
 * @brief Gets the model name.
 *
 * @return A string containing the model name.
 */
const std::string Kannala::getModelName() const {
    return model_name;
}

/**
 * @brief Gets a shared pointer to a Pinhole model.
 *
 * @return A shared pointer to a Pinhole model.
 */
std::shared_ptr<Pinhole> Kannala::getPinhole() const {
    return std::make_shared<Pinhole>(getFocalLength(), getPrincipalPoint(), 0, getImageSize(), getRotation(), getTranslation());
}

/**
 * @brief Sets the focal length.
 *
 * @param focal_length A vector containing the new focal length values.
 */
void Kannala::setFocalLength(std::vector<double> focal_length) {
    internalModel->setFocalLength(focal_length);
}

/**
 * @brief Sets the principal point.
 *
 * @param principal_point A vector containing the new principal point values.
 */
void Kannala::setPrincipalPoint(std::vector<double> principal_point) {
    internalModel->setPrincipalPoint(principal_point);
}

/**
 * @brief Sets the symmetric radial distortion coefficients.
 *
 * @param radial_dist_sym_coeffs A vector containing the new symmetric radial distortion coefficients.
 */
void Kannala::setRadialDistSymCoeffs(std::vector<double> radial_dist_sym_coeffs) {
    internalModel->setRadialDistSymCoeffs(radial_dist_sym_coeffs);
}

/**
 * @brief Sets the asymmetric radial distortion coefficients.
 *
 * @param radial_dist_asym_coeffs A vector containing the new asymmetric radial distortion coefficients.
 */
void Kannala::setRadialDistAsymCoeffs(std::vector<double> radial_dist_asym_coeffs) {
    internalModel->setRadialDistAsymCoeffs(radial_dist_asym_coeffs);
}

/**
 * @brief Sets the four-term radial distortion coefficients.
 *
 * @param radial_dist_four_coeffs A vector containing the new four-term radial distortion coefficients.
 */
void Kannala::setRadialDistFourCoeffs(std::vector<double> radial_dist_four_coeffs) {
    internalModel->setRadialDistFourCoeffs(radial_dist_four_coeffs);
}

/**
 * @brief Sets the asymmetric tangential distortion coefficients.
 *
 * @param tangential_dist_asym_coeffs A vector containing the new asymmetric tangential distortion coefficients.
 */
void Kannala::setTangentialDistAsymCoeffs(std::vector<double> tangential_dist_asym_coeffs) {
    internalModel->setTangentialDistAsymCoeffs(tangential_dist_asym_coeffs);
}

/**
 * @brief Sets the four-term tangential distortion coefficients.
 *
 * @param tangential_dist_four_coeffs A vector containing the new four-term tangential distortion coefficients.
 */
void Kannala::setTangentialDistFourCoeffs(std::vector<double> tangential_dist_four_coeffs) {
    internalModel->setTangentialDistFourCoeffs(tangential_dist_four_coeffs);
}

/**
 * @brief Sets the backprojection settings.
 *
 * @param threshold The new threshold value.
 * @param iterations The new number of iterations.
 */
void Kannala::setBackprojectSettings(double threshold, int iterations)
{
    internalModel->setBackprojectSettings(threshold, iterations);
}

/**
 * @brief Loads a Kannala model from a file.
 *
 * @param fileName The path of the file to load the model from.
 * @return A shared pointer to the loaded Kannala model.
 * @throws std::invalid_argument if the model contained in the imported file is not a Kannala model.
 */
std::shared_ptr<Kannala> Kannala::load(const std::string& fileName)
{
    auto camera_temp = Camera::load(fileName);
    auto type = camera_temp->getModelName();
    if (type == "Kannala")
    {
        return std::static_pointer_cast<Kannala>(camera_temp);
    }
    else
    {
        throw std::invalid_argument("Model contained in the imported file is not a Kannala Model.");
    }
}

// private methods
const std::vector<std::vector<double>> Kannala::getParameters() const {
    return
    {
        getFocalLength(),
        getPrincipalPoint(),
        getRadialDistSymCoeffs(),
        getRadialDistAsymCoeffs(),
        getRadialDistFourCoeffs(),
        getTangentialDistAsymCoeffs(),
        getTangentialDistFourCoeffs()
    };
}

void Kannala::setParameters(std::vector<std::vector<double>> parameters) {
    setFocalLength(parameters[0]);
    setPrincipalPoint(parameters[1]);
    setRadialDistSymCoeffs(parameters[2]);
    setRadialDistAsymCoeffs(parameters[3]);
    setRadialDistFourCoeffs(parameters[4]);
    setTangentialDistAsymCoeffs(parameters[5]);
    setTangentialDistFourCoeffs(parameters[6]);
}

const std::vector<std::string> Kannala::getParameterNames() const {
    return parameter_names;
}

const std::vector<std::string> Kannala::getParameterLabels() const {
    return parameter_file_labels;
}