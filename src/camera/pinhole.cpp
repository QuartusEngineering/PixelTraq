#include <iostream>
#include <fstream>
#include "camera/pinhole.h"

/**
 * @brief Parameterized constructor for the Pinhole class.
 *
 * @param focal_length Vector of focal length values.
 * @param principal_point Vector of principal point values.
 * @param skew value of skew term.
 * @param image_size Vector of image size dimensions.
 * @param rotation Rotation vector.
 * @param translation Translation vector.
 */
Pinhole::Pinhole(const std::vector<double>& focal_length, const std::vector<double>& principal_point, const double& skew, const std::vector<int>& image_size,const Point3& rotation ,const Point3& translation)
    : Camera{ image_size, rotation, translation } {
    // Ensure focal_length and principal_point vectors have the correct sizes
    if (focal_length.size() == 2) {
        this->focal_length = focal_length;  // Update internal vector
    }
    else {
        throw std::invalid_argument("focal_length vector must have exactly 2 elements.");
    }

    if (principal_point.size() == 2) {
        this->principal_point = principal_point;  // Update internal vector
    }
    else {
        throw std::invalid_argument("principal_point vector must have exactly 2 elements.");
    }

    this->skew = skew;
}

/**
 * @brief Copy constructor for the Pinhole class.
 *
 * @param model The Pinhole model to copy from.
 */
Pinhole::Pinhole(const Pinhole& model) : Camera{ model.image_size, model.rotation, model.translation } // copy constructor
{
    focal_length = model.focal_length;
    principal_point = model.principal_point;
    skew = model.skew;
}

/**
 * @brief Projects a 3D point to a 2D point using intrinsic and extrinsic parameters.
 *
 * @param point_3d The 3D point to project.
 * @return The projected 2D point.
 */
std::array<double, 2> Pinhole::project(const Point3& point_3d) const {
    double x = point_3d[0];
    double y = point_3d[1];
    double z = point_3d[2];
    double u = 1.0e12;
    double v = 1.0e12;

    if (z != 0.0) // handle divide by zero
    {
        u = focal_length[0] * x / z + skew * y / z + principal_point[0];
        v = focal_length[1] * y / z + principal_point[1];
    }

    return { u, v };
}

/**
 * @brief Backprojects a 2D point to a 3D ray.
 *
 * @param point_2d The 2D point to backproject.
 * @return The backprojected 3D ray.
 */
Point3 Pinhole::backproject(const std::array<double, 2>& point_2d) const {
    double u = point_2d[0];
    double v = point_2d[1];
    double x = 1.0e12;
    double y = 1.0e12;

    // backproject to normalized coordinates
    if (focal_length[1] != 0.0) // handle divide by zero
    {
        y = (v - principal_point[1]) / focal_length[1];
    }
    if (focal_length[0] != 0.0) // handle divide by zero
    {
        x = (u - principal_point[0] - y * skew) / focal_length[0];
    }
    double z = 1.0;  // Assume Z = 1 for direction

    return { x, y, z };
}

/**
 * @brief Gets the principal point.
 *
 * @return The principal point.
 */
std::vector<double> Pinhole::getPrincipalPoint() const {
    return principal_point;
}

/**
 * @brief Gets the focal length.
 *
 * @return The focal length.
 */
std::vector<double> Pinhole::getFocalLength() const {
    return focal_length;
}

/**
 * @brief Gets the skew.
 *
 * @return The skew.
 */
double Pinhole::getSkew() const {
    return skew;
}

/**
 * @brief Gets the model name.
 *
 * @return A string containing the model name.
 */
const std::string Pinhole::getModelName() const {
    return model_name;
}

/**
 * @brief Gets a shared pointer to a Pinhole model.
 *
 * @return A shared pointer to a Pinhole model.
 */
std::shared_ptr<Pinhole> Pinhole::getPinhole() const {
    return std::make_shared<Pinhole>(focal_length, principal_point, skew, image_size);
}

/**
 * @brief Sets the principal point.
 *
 * @param principal_point A vector containing the new principal point values.
 */
void Pinhole::setPrincipalPoint(std::vector<double> principal_point) {
    if (principal_point.size() == 2) {
        this->principal_point = principal_point;  // Update internal vector
    }
    else {
        throw std::invalid_argument("principal_point vector must have exactly 2 elements.");
    }
}

/**
 * @brief Sets the focal length.
 *
 * @param focal_length A vector containing the new focal length values.
 */
void Pinhole::setFocalLength(std::vector<double> focal_length) {
    if (focal_length.size() == 2) {
        this->focal_length = focal_length;
    }
    else {
        throw std::invalid_argument("focal_length vector must have exactly 2 elements.");
    }
}

/**
 * @brief Sets the skew.
 *
 * @param skew A double containing the new skew value.
 */
void Pinhole::setSkew(double skew) {
    this->skew = skew;
}

/**
 * @brief Loads a Pinhole model from a file.
 *
 * @param fileName The path of the file to load the model from.
 * @return A shared pointer to the loaded Pinhole model.
 * @throws std::invalid_argument if the model contained in the imported file is not a Pinhole model.
 */
std::shared_ptr<Pinhole> Pinhole::load(const std::string& fileName)
{
    // add validation
    auto camera_temp = Camera::load(fileName);
    auto type = camera_temp->getModelName();
    if (type == "Pinhole")
    {
        return std::static_pointer_cast<Pinhole>(camera_temp);
    }
    else
    {
        throw std::invalid_argument("Model contained in the imported file is not a Pinhole Model.");
    }
}

// private methods
const std::vector<std::vector<double>> Pinhole::getParameters() const {
    return {
        getFocalLength(),
        getPrincipalPoint(),
        {getSkew()}
    };
}

const std::vector<std::string> Pinhole::getParameterNames() const {
    return parameter_names;
}

const std::vector<std::string> Pinhole::getParameterLabels() const {
    return parameter_file_labels;
}

void Pinhole::setParameters(std::vector<std::vector<double>> parameters)
{
    setFocalLength(parameters[0]);
    setPrincipalPoint(parameters[1]);
    setSkew(parameters[2][0]);
}