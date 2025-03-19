#include <iostream>
#include <external/nlohmann/json.hpp>
#include <fstream>
#include <cassert>
#include <string>
#include <gtest/gtest.h>
#include "test_camera_classes.h"
#include "pixeltraq.h"

bool isClose(double a, double b, double epsilon) {
    return std::abs(a - b) < epsilon;
}

// Function to extract arrays from JSON file
bool extractJsonArrays(
    const std::string& jsonFilePath,
    std::vector<std::array<double, 2>>& projected_array,
    std::vector<std::array<double, 3>>& backprojected_array,
    std::vector<std::array<double, 2>>& input_array,
    std::vector<std::array<double, 3>>* transformed_array // Optional parameter
) {
    // Open the JSON file
    std::ifstream file(jsonFilePath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << jsonFilePath << std::endl;
        return false;
    }

    // Parse the JSON file
    nlohmann::json jsonData;
    try {
        file >> jsonData;
    }
    catch (const nlohmann::json::parse_error& e) {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        return false;
    }

    // Extract the Projected array
    try {
        projected_array = jsonData.at("Projected").get<std::vector<std::array<double, 2>>>();
    }
    catch (const nlohmann::json::out_of_range& e) {
        std::cerr << "Error: Missing 'Projected' key in JSON" << std::endl;
        return false;
    }

    // Extract the Backprojected array
    try {
        backprojected_array = jsonData.at("Backprojected").get<std::vector<std::array<double, 3>>>();
    }
    catch (const nlohmann::json::out_of_range& e) {
        std::cerr << "Error: Missing 'Backprojected' key in JSON" << std::endl;
        return false;
    }

    try {
        input_array = jsonData.at("Input").get<std::vector<std::array<double, 2>>>();
    }
    catch (const nlohmann::json::out_of_range& e) {
        std::cerr << "Error: Missing 'input' key in JSON" << std::endl;
        return false;
    }

    if (transformed_array != nullptr) {
        try {
            *transformed_array = jsonData.at("Transformed").get<std::vector<std::array<double, 3>>>();
        }
        catch (const nlohmann::json::out_of_range& e) {
            std::cout << "Warning: Missing 'Transformed' key in JSON, skipping" << std::endl;
        }
    }

    return true;
}


TEST(CameraClass, Pinhole_Project_Backproject) {
    std::string camera_json = std::string(TEST_DATA_DIR) + "/pinhole.json";
    // Load the camera model
    std::shared_ptr<Camera> camera_mdl = Camera::load(camera_json);

    ASSERT_NE(camera_mdl, nullptr) << "Failed to load camera model";


    // Extract Projected and Backprojected arrays
    std::vector<std::array<double, 2>> projected_array;
    std::vector<std::array<double, 3>> backprojected_array;
    std::vector<std::array<double, 2>> input_array;
    
    // Use extractJsonArrays to load arrays from JSON
    bool success = extractJsonArrays(camera_json, projected_array, backprojected_array, input_array);
    ASSERT_TRUE(success) << "Failed to extract arrays from JSON";

    std::vector<std::array<double, 3>> grid_rays = camera_mdl->backproject(input_array);
    std::vector<std::array<double, 2>> distorted_pixels = camera_mdl->project(backprojected_array);

    std::vector<std::array<double, 2>> reprojected_pixels = camera_mdl->project(grid_rays);


    EXPECT_TRUE(compareArrays(reprojected_pixels, input_array))
        << "2D -> 3D -> 2D points are not consistent";

    EXPECT_TRUE(compareArrays(backprojected_array, grid_rays))
        << "Backprojections do not match";

    
    EXPECT_TRUE(compareArrays(projected_array, distorted_pixels))
        << "Projections do not match";

}
TEST(CameraClass, BC_Project_Backproject) {
    // Define the JSON file path
    std::string camera_json = std::string(TEST_DATA_DIR) + "/brown_conrady.json";

    // Load the camera model
    std::shared_ptr<Camera> camera_mdl = Camera::load(camera_json);
    ASSERT_NE(camera_mdl, nullptr) << "Failed to load camera model";


    // Extract Projected and Backprojected arrays
    std::vector<std::array<double, 2>> projected_array;
    std::vector<std::array<double, 3>> backprojected_array;
    std::vector<std::array<double, 2>> input_array;

    // Use extractJsonArrays to load arrays from JSON
    bool success = extractJsonArrays(camera_json, projected_array, backprojected_array, input_array);
    ASSERT_TRUE(success) << "Failed to extract arrays from JSON";

    std::vector<std::array<double, 3>> grid_rays = camera_mdl->backproject(input_array);
    std::vector<std::array<double, 2>> distorted_pixels = camera_mdl->project(backprojected_array);
    std::vector<std::array<double, 2>> reprojected_pixels = camera_mdl->project(grid_rays);

    // normalize to z = 1 normalization
    for (int i = 0; i < backprojected_array.size(); i++)
    {
        backprojected_array[i][0] = backprojected_array[i][0] / backprojected_array[i][2];
        backprojected_array[i][1] = backprojected_array[i][1] / backprojected_array[i][2];
        backprojected_array[i][2] = 1;
    };

    EXPECT_TRUE(compareArrays(reprojected_pixels, input_array))
        << "2D -> 3D -> 2D points are not consistent";

    EXPECT_TRUE(compareArrays(backprojected_array, grid_rays))
        << "Backprojections do not match";

    EXPECT_TRUE(compareArrays(projected_array, distorted_pixels))
        << "Projections do not match";
}
TEST(CameraClass, Kan_Rad_Project_Backproject) {
    // Define the JSON file path
    std::string camera_json = std::string(TEST_DATA_DIR) + "/kannala_radial.json";

    // Load the camera model
    std::shared_ptr<Camera> camera_mdl = Camera::load(camera_json);
    ASSERT_NE(camera_mdl, nullptr) << "Failed to load camera model";



    const auto& rotation_matrix = camera_mdl->getRotationMatrix();
    const auto& translation = camera_mdl->getTranslation();
    const auto& inv_rotation_matrix = camera_mdl->getInvRotationMatrix();
    const auto& inv_translation = camera_mdl->getInvTranslation(); 

    // Extract Projected and Backprojected arrays
    std::vector<std::array<double, 2>> projected_array;
    std::vector<std::array<double, 3>> backprojected_array;
    std::vector<std::array<double, 2>> input_array;
    std::vector<std::array<double, 3>> transformed_array;


    // Use extractJsonArrays to load arrays from JSON
    bool success = extractJsonArrays(camera_json, projected_array, backprojected_array, input_array, &transformed_array);
    ASSERT_TRUE(success) << "Failed to extract arrays from JSON";

    // normalize to z = 1 normalization
    for (int i = 0; i < backprojected_array.size(); i++)
    {
        backprojected_array[i][0] = backprojected_array[i][0] / backprojected_array[i][2];
        backprojected_array[i][1] = backprojected_array[i][1] / backprojected_array[i][2];
        backprojected_array[i][2] = 1;
    };

    std::vector<std::array<double, 3>> grid_rays = camera_mdl->backproject(input_array);
    std::vector<std::array<double, 3>> transformed_pnts = CommonMath::transformPoints(grid_rays, rotation_matrix, translation);
    std::vector<std::array<double, 3>> inv_pnts = CommonMath::transformPoints(transformed_pnts, inv_rotation_matrix, inv_translation);
    //auto img_pnts = camera_mdl->worldToImagePnts(backprojected_array);
    //auto world_pnts = camera_mdl->imageToWorldPnts(input_array);

    std::vector<std::array<double, 2>> distorted_pixels = camera_mdl->project(backprojected_array);
    std::vector<std::array<double, 2>> reprojected_pixels = camera_mdl->project(grid_rays);

    EXPECT_TRUE(compareArrays(reprojected_pixels, input_array))
        << "2D -> 3D -> 2D points are not consistent";

    EXPECT_TRUE(compareArrays(backprojected_array, grid_rays))
        << "Backprojections do not match";

    EXPECT_TRUE(compareArrays(projected_array, distorted_pixels))
        << "Projections do not match";

    //Need to breakup extrinsic and intrinsic
   /* assert(compareArrays(backprojected_array, grid_rays) &&
        "Kannala radial backprojections do not match");


    assert(compareArrays(transformed_pnts, transformed_array) &&
        "Transformed points do not match");

    assert(compareArrays(grid_rays, inv_pnts) &&
        "inverse transformation does not match");


    assert(compareArrays(projected_array, distorted_pixels) &&
        "Kannala Radial projections do not match");

    std::cout << "test_kannala_radial passed!" << std::endl;*/
}


TEST(CameraClass, Kan_Full_Project_Backproject) {
    // Define the JSON file path
    std::string camera_json = std::string(TEST_DATA_DIR) + "/kannala_full.json";

    // Load the camera model
    std::shared_ptr<Camera> camera_mdl = Camera::load(camera_json);
    ASSERT_NE(camera_mdl, nullptr) << "Failed to load camera model";


    // Extract Projected and Backprojected arrays
    std::vector<std::array<double, 2>> projected_array;
    std::vector<std::array<double, 3>> backprojected_array;
    std::vector<std::array<double, 2>> input_array;

    // Use extractJsonArrays to load arrays from JSON
    bool success = extractJsonArrays(camera_json, projected_array, backprojected_array, input_array);
    ASSERT_TRUE(success) << "Failed to extract arrays from JSON";

    // normalize to z = 1 normalization
    for (int i = 0; i < backprojected_array.size(); i++)
    {
        backprojected_array[i][0] = backprojected_array[i][0] / backprojected_array[i][2];
        backprojected_array[i][1] = backprojected_array[i][1] / backprojected_array[i][2];
        backprojected_array[i][2] = 1;
    };

    std::vector<std::array<double, 3>> grid_rays = camera_mdl->backproject(input_array);
    std::vector<std::array<double, 2>> distorted_pixels = camera_mdl->project(backprojected_array);
    std::vector<std::array<double, 2>> reprojected_pixels = camera_mdl->project(grid_rays);

    EXPECT_TRUE(compareArrays(reprojected_pixels, input_array))
        << "2D -> 3D -> 2D points are not consistent";

    EXPECT_TRUE(compareArrays(backprojected_array, grid_rays))
        << "Backprojections do not match";

    EXPECT_TRUE(compareArrays(projected_array, distorted_pixels))
        << "Projections do not match";
}