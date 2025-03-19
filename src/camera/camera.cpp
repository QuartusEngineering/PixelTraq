#include "camera/camera.h"
#include "camera/pinhole.h"      
#include "camera/brown_conrady.h" 
#include "camera/general_ftheta.h"
#include "camera/general_ftan_theta.h"
#include "camera/kannala.h"
#include <fstream>
#include <iostream>
#include <algorithm>

/**
 * @brief Default constructor for the Camera class.
 */
Camera::Camera() {
    rotation_matrix = CommonMath::eulerToRot(this->rotation);
    inv_rotation_matrix = CommonMath::rotationInverse(this->rotation_matrix);
    inv_translation = createInverseTranslation(this->translation);
}

/**
 * @brief Parameterized constructor for the Camera class.
 * @param image_size Vector containing the width and height of the image.
 * @param rotation Rotation angles in radians.
 * @param translation Translation vector.
 * @throws std::invalid_argument if image_size does not contain exactly 2 elements.
 */
Camera::Camera(std::vector<int> image_size, Point3 rotation, Point3 translation) {

    if (image_size.size() == 2) {
        this->image_size = image_size;  // Update internal vector
    }
    else {
        throw std::invalid_argument("image_size vector must have exactly 2 elements.");
    }

    setRotation(rotation);
    setTranslation(translation);
    rotation_matrix = CommonMath::eulerToRot(this->rotation);
    inv_rotation_matrix = CommonMath::rotationInverse(this->rotation_matrix);
    inv_translation = createInverseTranslation(this->translation);
};

/**
 * @brief Returns a string with the camera parameters for display.
 * @return A string containing the camera parameter details.
 */
std::string Camera::getParameterDisplayString() const
{
    std::ostringstream oss;

    oss << getModelName() << " Camera Model" << std::endl;

    auto properties = getParameters();
    auto labels = getParameterNames();

    for (int i = 0; i < properties.size(); i++)
    {
        std::vector<double> values = properties[i];
        oss << labels[i] << ": [";
        for (int j = 0; j < values.size(); j++)
        {
            if (j != 0)
            {
                oss << ", " << values[j];
            }
            else
            {
                oss << values[j];
            }
        }
        oss << "]" << std::endl;
    }

    // Display translation
    oss << "Translation (tx, ty, tz): ";
    if (translation.size() == 3) {
        oss << "[" << translation[0] << ", " << translation[1] << ", " << translation[2] << "]" << std::endl;
    }
    else {
        oss << "Translation data is incomplete." << std::endl;
    }

    // Display rotation
    oss << "Rotation (rx, ry, rz): ";
    if (rotation.size() == 3) {
        oss << "[" << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << "]" << std::endl;
    }
    else {
        oss << "Rotation data is incomplete." << std::endl;
    }

    return oss.str();
}

/**
 * @brief Displays the camera parameters to the console.
 */
void Camera::display() const
{
    std::cout << getParameterDisplayString();
}

/**
 * @brief Static method to load a camera based on file type.
 * @param fileName The name of the file.
 * @return A shared pointer to the loaded Camera object.
 * @throws std::invalid_argument if the file does not exist.
 */
std::shared_ptr<Camera> Camera::load(const std::string& fileName) {

    // Map file extensions to their respective loading functions
    static const std::unordered_map<std::string, std::shared_ptr<Camera>(*)(const std::string&)> loaders = {
        {"json", Camera::loadCameraFromJson},
        // Future extensions can be added here
        // {"xml", Camera::loadCameraFromXml},
        // {"yaml", Camera::loadCameraFromYaml},
    };

    if (!Utils::exists(fileName))
    {
        throw std::invalid_argument("The file referenced does not exist");
    }

    std::string extension = getFileExtension(fileName);
    // Check if the file extension has a registered loader
    auto it = loaders.find(extension);
    if (it != loaders.end()) {
        return it->second(fileName);  // Call the appropriate loader function
    }
    else {
        std::cerr << "Filetype ." << extension << " is not implemented." << std::endl;
        return nullptr;
    }
}

/**
 * @brief Saves the Camera object to a file.
 * @param fileName The name of the file.
 */
void Camera::save(const std::string& fileName) const {
    // Map file extensions to their respective saving functions
    static const std::unordered_map<std::string, void(*)(const std::shared_ptr<const Camera>&, const std::string&)> savers = {
        {"json", Camera::saveCameraToJson},
        // Future extensions can be added here
        // { "xml", Camera::saveCameraToXml },
        // { "yaml", Camera::saveCameraToYaml },
    };

    std::string extension = getFileExtension(fileName);
    // Check if the file extension has a registered saver
    auto it = savers.find(extension);
    if (it != savers.end()) {
        it->second(shared_from_this(), fileName); // Call the appropriate saver function
    }
    else {
        std::cerr << "Filetype ." << extension << " is not implemented." << std::endl;
    }
}

/**
 * @brief Projects 3D points to 2D points using the camera model.
 * @param points_3d A vector of 3D points.
 * @return A vector of projected 2D points.
 */
std::vector<Point2> Camera::project(const std::vector<Point3>& points_3d) const {
    std::vector<std::array<double, 2>> projectedPoints;
    size_t N = points_3d.size();
    projectedPoints.reserve(N);
    projectedPoints.resize(N);
    #pragma omp parallel for
    for (size_t i = 0; i < N; ++i) {
        projectedPoints[i] = project(points_3d[i]);
    }
    return projectedPoints;
}

/**
 * @brief Projects 2D vectors of 3D points to 2D points using the camera model.
 * @param points_3d A 2D vector of 3D points.
 * @return A 2D vector of projected 2D points.
 */
std::vector<std::vector<Point2>> Camera::project(const std::vector<std::vector<Point3>>& points_3d) const {
    size_t height = points_3d.size();
    size_t width = points_3d[0].size();
    std::vector<std::vector<Point2>> projectedImage(height, std::vector<Point2>(width));

#pragma omp parallel for
    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            projectedImage[y][x] = project(points_3d[y][x]);
        }
    }
    return projectedImage;
}

/**
 * @brief Backprojects 2D points to 3D rays using the camera model.
 * @param points_2d A vector of 2D points.
 * @return A vector of backprojected 3D rays.
 */
std::vector<Point3> Camera::backproject(const std::vector<Point2>& points_2d) const {
    std::vector<Point3> backprojectedPoints;
    backprojectedPoints.reserve(points_2d.size());
    for (const auto& point : points_2d) {
        backprojectedPoints.push_back(backproject(point));
    }
    return backprojectedPoints;
}

/**
 * @brief Backprojects a 2D vector of 2D points to 3D rays using the camera model.
 * @param image A 2D vector of 2D points.
 * @return A 2D vector of backprojected 3D rays.
 */
std::vector<std::vector<std::array<double, 3>>> Camera::backproject(const std::vector<std::vector<double>>& image) const {
    size_t height = image.size();
    size_t width = image[0].size();
    std::vector<std::vector<std::array<double, 3>>> backprojectedImage(height, std::vector<std::array<double, 3>>(width));

#pragma omp parallel for
    for (size_t y = 1; y <= height; ++y) {
        for (size_t x = 1; x <= width; ++x) {
            backprojectedImage[y-1][x-1] = backproject(std::array<double, 2> { static_cast<double>(y), static_cast<double>(x) });
        }
    }
    return backprojectedImage;
}

/**
 * @brief Gets the image size.
 * @return A vector containing the width and height of the image.
 */
std::vector<int> Camera::getImageSize() const {
    return image_size;
}

/**
 * @brief Sets the image size.
 * @param image_size Vector containing the width and height of the image.
 * @throws std::invalid_argument if image_size does not contain exactly 2 elements.
 */
void Camera::setImageSize(std::vector<int> image_size) {
    if (image_size.size() == 2) {
        this->image_size = image_size;  // Update internal vector
    }
    else {
        throw std::invalid_argument("image_size vector must have exactly 2 elements.");
    }
}

/**
 * @brief Sets the translation vector.
 * @param translation The translation vector.
 * @throws std::invalid_argument if translation does not contain exactly 3 elements.
 */
void Camera::setTranslation(Point3 translation)
{
    if (translation.size() == 3) {
        this->translation = translation;
        this->inv_translation = createInverseTranslation(this->translation);
    }
    else {
        throw std::invalid_argument("rotation vector must have exactly 2 elements.");
    }
}

/**
 * @brief Sets the rotation vector.
 * @param rotation The rotation vector.
 * @throws std::invalid_argument if rotation does not contain exactly 3 elements.
 */
void Camera::setRotation(Point3 rotation)
{
    if (rotation.size() == 3) {
        this->rotation = rotation;

        this->rotation_matrix = CommonMath::eulerToRot(this->rotation);
        this->inv_rotation_matrix = CommonMath::rotationInverse(this->rotation_matrix);
        this->inv_translation = createInverseTranslation(this->translation);
    }
    else {
        throw std::invalid_argument("rotation vector must have exactly 2 elements.");
    }
}

/**
 * @brief Transforms world points to camera points.
 * @param world_points A vector of world points.
 * @return A vector of transformed camera points.
 */
const std::vector<Point3> Camera::worldToCameraPnts(const std::vector<Point3>& world_points) const {
    return CommonMath::transformPoints(world_points, rotation_matrix, translation);
}

/**
 * @brief Transforms a single world point to a camera point.
 * @param world_point A single world point.
 * @return The transformed camera point.
 */
const Point3 Camera::worldToCameraPnts(const Point3& world_point) const {
    return CommonMath::transformPoint(world_point, rotation_matrix, translation);
}

/**
 * @brief Transforms camera points to world points.
 * @param camera_points A vector of camera points.
 * @return A vector of transformed world points.
 */
const std::vector<Point3> Camera::cameraToWorldPnts(const std::vector<Point3>& world_points) const {
    return CommonMath::transformPoints(world_points, inv_rotation_matrix, inv_translation);
}

/**
 * @brief Transforms a single camera point to a world point.
 * @param camera_point A single camera point.
 * @return The transformed world point.
 */
const Point3 Camera::cameraToWorldPnts(const Point3& world_point) const {
    return CommonMath::transformPoint(world_point, inv_rotation_matrix, inv_translation);
}

std::string Camera::getFileExtension(const std::string& fileName) {
    size_t dotPos = fileName.find_last_of('.');
    if (dotPos == std::string::npos) return ""; // No extension found
    return fileName.substr(dotPos + 1);
}

std::shared_ptr<Camera> Camera::loadCameraFromJson(const std::string& fileName) {
    std::ifstream file(fileName);

    if (!file.is_open()) {
        std::cerr << "Could not open file: " << fileName << std::endl;
        return nullptr;
    }

    nlohmann::json j;
    file >> j;

    if (j.contains("Intrinsics") && j["Intrinsics"].contains("class_name")) {
        std::string className = j["Intrinsics"]["class_name"].get<std::string>();

        std::shared_ptr<Camera> camera;
        if (className == "PinholeModel") {
            camera = std::make_shared<Pinhole>();
        }
        else if (className == "BrownConradyModel") {
            camera = std::make_shared<BrownConrady>();
        }
        else if (className == "GenFThetaModel") {
            camera = std::make_shared<GenFTheta>();
        }
        else if (className == "GenFTanThetaModel") {
            camera = std::make_shared<GenFTanTheta>();
        }
        else if (className == "KannalaModel") {
            camera = std::make_shared<Kannala>();
        }
        else {
            throw std::invalid_argument("Model contained in the imported file is not a valid model.");
        }

        // Load extrinsic parameters common to all cameras
        camera->loadExtrinsicsFromJson(j);

        // Load intrinsic parameters
        camera->loadIntrinsicsFromJson(j);

        return camera;
    }
    //change to exception
    std::cerr << "Invalid configuration file format." << std::endl;
    return nullptr;
}

void Camera::loadExtrinsicsFromJson(const nlohmann::json& j) {
    if (j.contains("Extrinsics")) {
        const auto& extrinsics = j["Extrinsics"];
        if (extrinsics.contains("translation") && extrinsics["translation"].is_array()) {
            translation = extrinsics["translation"].get<std::array<double, 3>>();
        }
        if (extrinsics.contains("rotation") && extrinsics["rotation"].is_array()) {
            rotation = extrinsics["rotation"].get<std::array<double, 3>>();
        }
    }
    // Always recalculate the rotation_matrix
    if (rotation.empty()) {
        rotation = { 0.0, 0.0, 0.0 }; // Default rotation (identity)
    }
    rotation_matrix = CommonMath::eulerToRot(rotation);
    inv_rotation_matrix = CommonMath::rotationInverse(rotation_matrix);
    inv_translation = createInverseTranslation(translation);
}

void Camera::loadIntrinsicsFromJson(const nlohmann::json& j) {
    if (j.contains("Intrinsics")) {
        const auto& intrinsics = j["Intrinsics"];
        auto labels = getParameterLabels();
        std::vector<std::vector<double>> parameters = {};
        for (size_t i = 0; i < labels.size(); i++)
        {
            if (intrinsics.contains(labels[i])) {
                if (intrinsics[labels[i]].is_array()) {
                    parameters.push_back(intrinsics[labels[i]].get<std::vector<double>>());
                }
                else {
                    parameters.push_back({ intrinsics[labels[i]].get<double>() });
                }
            }
        }
        setParameters(parameters);

        std::vector<int> image_size = {};
        if (intrinsics.contains("image_size")) {
            image_size = intrinsics["image_size"].get<std::vector<int>>();
        }
        this->image_size = image_size;
    }
}

void Camera::saveCameraToJson(const std::shared_ptr<const Camera>& camera, const std::string& fileName) {
    nlohmann::json j;

    j["MetaData"] = {
        {"Note", "This model was exported from the PixelTraq library. This is not the original camera model and parameters may have been modified. For full metadata, contact the calibration supplier."}
    };

    auto properties = camera->getParameters();
    auto names = camera->getParameterNames();
    auto labels = camera->getParameterLabels();

    for (int i = 0; i < properties.size(); i++)
    {
        j["Intrinsics"][labels[i]] = properties[i];
    }
    j["Intrinsics"]["image_size"] = camera->getImageSize();

    auto name_temp = camera->getModelName();
    std::string name;
    std::remove_copy_if(name_temp.begin(), name_temp.end(), std::back_inserter(name), [](char c) { return c == ' '; });
    j["Intrinsics"]["class_name"] = name + "Model";
    j["Intrinsics"]["coordinate_convention"] = "TL0_0";

    j["Extrinsics"] = {
        {"rotation", camera->getRotation()},
        {"translation", camera->getTranslation()},
        {"class_name", "RealObject"}
    };

    // Write the JSON object to the file
    std::ofstream file(fileName);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file for writing");
    }
    file << j.dump(4); // Pretty-print with 4 spaces
    file.close();
}

Point3 Camera::createInverseTranslation(const Point3& translation) {
    // inv_translation = -inv(R) * translation
    Point3 result{};

    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            result[i] += -inv_rotation_matrix[i][j] * translation[j];
        }
    }
    return result;
}