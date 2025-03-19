#include <gtest/gtest.h>
#include <stdexcept>
#include <fstream>
#include <vector>
#include <array>
#include "pixeltraq.h"

// Test case for zero-argument constructor
TEST(PinholeTest, ZeroArgConstructor_StandardCall_ValidModel) {
    Pinhole model;
    EXPECT_EQ(model.getFocalLength(), std::vector<double>({ 1, 1 }));
    EXPECT_EQ(model.getPrincipalPoint(), std::vector<double>({ 0, 0 }));
    EXPECT_EQ(model.getSkew(), 0);
}

// Test case for many-argument constructor with correct arguments
TEST(PinholeTest, ManyArgConstructor_StandardCall_ValidModel) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };

    Pinhole model(focal_length, principal_point, skew, image_size);

    EXPECT_EQ(model.getFocalLength(), focal_length);
    EXPECT_EQ(model.getPrincipalPoint(), principal_point);
    EXPECT_EQ(model.getSkew(), skew);
    EXPECT_EQ(model.getImageSize(), image_size);
}

// Test case for many-argument constructor with incorrect sized arguments
TEST(PinholeTest, ManyArgConstructor_IncorrectSizedArgs_ThrowException) {
    std::vector<double> incorrect_focal_length = { 2.0 };
    std::vector<double> correct_principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };

    EXPECT_THROW(Pinhole(incorrect_focal_length, correct_principal_point, skew, image_size), std::invalid_argument);

    std::vector<double> correct_focal_length = { 2.0, 2.0 };
    std::vector<double> incorrect_principal_point = { 1.0 };

    EXPECT_THROW(Pinhole(correct_focal_length, incorrect_principal_point, skew, image_size), std::invalid_argument);

    std::vector<int> incorrect_image_size = { 480 };

    EXPECT_THROW(Pinhole(correct_focal_length, correct_principal_point, skew, incorrect_image_size), std::invalid_argument);
}

// Test case for copy constructor
TEST(PinholeTest, CopyConstructor_Called_AllPropertiesIndependent) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };

    Pinhole original(focal_length, principal_point, skew, image_size);
    Pinhole copy(original);

    EXPECT_EQ(copy.getFocalLength(), focal_length);
    EXPECT_EQ(copy.getPrincipalPoint(), principal_point);
    EXPECT_EQ(copy.getSkew(), skew);
    EXPECT_EQ(copy.getImageSize(), image_size);

    // Modify the original and ensure the copy remains unchanged
    focal_length[0] = 3.0;
    principal_point[0] = 2.0;
    skew = 0.8; 
    image_size = { 0, 0 };
    original.setFocalLength(focal_length);
    original.setPrincipalPoint(principal_point);
    original.setSkew(skew);
    original.setImageSize(image_size);

    EXPECT_EQ(copy.getFocalLength(), std::vector<double>({ 2.0, 2.0 }));
    EXPECT_EQ(copy.getPrincipalPoint(), std::vector<double>({ 1.0, 1.0 }));
    EXPECT_EQ(copy.getSkew(), 0.5);
    EXPECT_EQ(copy.getImageSize(), std::vector<int>({ 640, 480 }));
}

// Test case for project with normal inputs resulting in normal results
TEST(PinholeTest, Project_NormalInputs_NormalResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 0.5, 0.5 };
    double skew = 0;
    std::vector<int> image_size = { 2, 2 };

    Pinhole model(focal_length, principal_point, skew, image_size);

    std::array<double, 3> point3D = { 1.0, 1.0, 1.0 };
    std::array<double, 2> expected_projection = { 1.5, 1.5 }; // Sample expected result
    EXPECT_EQ(model.project(point3D), expected_projection);

    double skew2 = 1;
    Pinhole model2(focal_length, principal_point, skew2, image_size);

    std::array<double, 3> point3D2 = { 1.0, 1.0, 1.0 };
    std::array<double, 2> expected_projection2 = { 2.5, 1.5 }; // Sample expected result
    EXPECT_EQ(model2.project(point3D2), expected_projection2);
}

// Test case for project with zero inputs resulting in valid results
TEST(PinholeTest, Project_ZerosInputs_ValidResult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };

    Pinhole model(focal_length, principal_point, skew, image_size);

    std::array<double,3> point3D = { 0.0, 0.0, 1.0 };
    std::array<double, 2> expected_projection = { 1.0, 1.0 }; // Sample expected result

    EXPECT_EQ(model.project(point3D), expected_projection);

    std::array<double, 3> point3D2 = { 0.0, 0.0, 0 };
    std::array<double, 2> expected_projection2 = { 1.0e12, 1.0e12 }; // Sample expected result

    EXPECT_EQ(model.project(point3D2), expected_projection2);
}

// Test case for project with zero parameters resulting in valid results
TEST(PinholeTest, Project_ZeroParameters_ValidResult) {
    Pinhole model;

    std::array<double, 3> point3D = { 0.0, 0.0, 1.0 };
    std::array<double, 2> expected_projection = { 0.0, 0.0 }; // Sample expected result

    EXPECT_EQ(model.project(point3D), expected_projection);

    std::vector<double> focal_length = { 0.0, 0.0 };
    std::vector<double> principal_point = { 0.0, 0.0 };
    double skew = 0.0;
    std::vector<int> image_size = { 0, 0 };

    Pinhole model2(focal_length, principal_point, skew, image_size);

    std::array<double, 3> point3D2 = { 1.0, 1.0, 1.0 };
    std::array<double, 2> expected_projection2 = { 0.0, 0.0 }; // Sample expected result

    EXPECT_EQ(model2.project(point3D2), expected_projection2);
}

// Test case for backproject with normal inputs resulting in normal results
TEST(PinholeTest, BackProject_NormalInputs_NormalResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 0.5, 0.5 };
    double skew = 0.0;
    std::vector<int> image_size = { 640, 480 };

    Pinhole model(focal_length, principal_point, skew, image_size);

    std::array<double, 2> point2D = { 1.5, 1.5 };
    std::array<double, 3> expected_backprojection = { 1.0, 1.0, 1.0 }; // Sample expected result

    EXPECT_EQ(model.backproject(point2D), expected_backprojection);

    skew = 1;

    Pinhole model2(focal_length, principal_point, skew, image_size);

    std::array<double, 2> point2D2 = { 2.5, 1.5 };
    std::array<double, 3> expected_backprojection2 = { 1.0, 1.0, 1.0 }; // Sample expected result

    EXPECT_EQ(model2.backproject(point2D2), expected_backprojection2);
}

// Test case for backproject with zero inputs resulting in valid results
TEST(PinholeTest, BackProject_ZerosInputs_ValidResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 0.5, 0.5 };
    double skew = 0.0;
    std::vector<int> image_size = { 2, 2 };

    Pinhole model(focal_length, principal_point, skew, image_size);

    std::array<double, 2> point2D = { 0.5, 0.5 };
    std::array<double, 3> expected_backprojection = { 0.0, 0.0, 1.0 }; // Sample expected result
    EXPECT_EQ(model.backproject(point2D), expected_backprojection);

    skew = 1.0;

    Pinhole model2(focal_length, principal_point, skew, image_size);

    std::array<double, 2> point2D2 = { 0.5, 0.5 };
    std::array<double, 3> expected_backprojection2 = { 0.0, 0.0, 1.0 }; // Sample expected result
    auto res = model2.backproject(point2D2);
    EXPECT_EQ(model2.backproject(point2D2), expected_backprojection2);
}

// Test case for backproject with zero parameters resulting in valid results
TEST(PinholeTest, BackProject_ZeroParameters_ValidResult) {
    Pinhole model;

    std::array<double, 2> point2D = { 0.0, 0.0 };
    std::array<double, 3> expected_backprojection = { 0.0, 0.0, 1.0 }; // Sample expected result

    EXPECT_EQ(model.backproject(point2D), expected_backprojection);

    std::vector<double> focal_length = { 0.0, 0.0 };
    std::vector<double> principal_point = { 0.0, 0.0 };
    double skew = 0.0;
    std::vector<int> image_size = { 0, 0 };

    Pinhole model2(focal_length, principal_point, skew, image_size);

    std::array<double, 2> point2D2 = { 0.0, 0.0 };
    std::array<double, 3> expected_backprojection2 = { 1.0e12, 1.0e12, 1.0 }; // Sample expected result

    EXPECT_EQ(model2.backproject(point2D2), expected_backprojection2);
}

// Test case for getPrincipalPoint using zero-argument constructor
TEST(PinholeTest, GetPrincipalPoint_ZeroArgConstructor_ValidResult) {
    Pinhole model;
    std::vector<double> expected_principal_point = { 0.0, 0.0 };
    EXPECT_EQ(model.getPrincipalPoint(), expected_principal_point);
}

// Test case for getPrincipalPoint using many-argument constructor
TEST(PinholeTest, GetPrincipalPoint_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };

    Pinhole model(focal_length, principal_point, skew, image_size);
    EXPECT_EQ(model.getPrincipalPoint(), principal_point);
}

// Test case for getFocalLength using zero-argument constructor
TEST(PinholeTest, GetFocalLength_ZeroArgConstructor_ValidResult) {
    Pinhole model;
    std::vector<double> expected_focal_length = { 1.0, 1.0 };
    EXPECT_EQ(model.getFocalLength(), expected_focal_length);
}

// Test case for getFocalLength using many-argument constructor
TEST(PinholeTest, GetFocalLength_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };

    Pinhole model(focal_length, principal_point, skew, image_size);
    EXPECT_EQ(model.getFocalLength(), focal_length);
}

// Test case for getSkew using zero-argument constructor
TEST(PinholeTest, GetSkew_ZeroArgConstructor_ValidResult) {
    Pinhole model;
    double expected_skew = 0.0;
    EXPECT_EQ(model.getSkew(), expected_skew);
}

// Test case for getSkew using many-argument constructor
TEST(PinholeTest, GetSkew_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };

    Pinhole model(focal_length, principal_point, skew, image_size);
    EXPECT_EQ(model.getSkew(), skew);
}

// Test case for getImageSize using zero-argument constructor
TEST(PinholeTest, GetImageSize_ZeroArgConstructor_ValidResult) {
    Pinhole model;
    std::vector<int> expected_image_size = { 0,0 };
    EXPECT_EQ(model.getImageSize(), expected_image_size);
}

// Test case for getImageSize using many-argument constructor
TEST(PinholeTest, GetImageSize_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };

    Pinhole model(focal_length, principal_point, skew, image_size);
    EXPECT_EQ(model.getImageSize(), image_size);
}

// Test case for setPrincipalPoint with valid size inputs
TEST(PinholeTest, SetPrincipalPoint_ValidSizeInputs_SetParameters) {
    Pinhole model;
    std::vector<double> principal_point = { 1.0, 1.0 };
    EXPECT_NO_THROW(model.setPrincipalPoint(principal_point));
    EXPECT_EQ(model.getPrincipalPoint(), principal_point);
}

// Test case for setPrincipalPoint with invalid size inputs
TEST(PinholeTest, SetPrincipalPoint_InvalidSizeInputs_ThrowException) {
    Pinhole model;
    std::vector<double> invalid_principal_point = { 1.0 };
    EXPECT_THROW(model.setPrincipalPoint(invalid_principal_point), std::invalid_argument);
}

// Test case for setFocalLength with valid size inputs
TEST(PinholeTest, SetFocalLength_ValidSizeInputs_SetParameters) {
    Pinhole model;
    std::vector<double> focal_length = { 2.0, 2.0 };
    EXPECT_NO_THROW(model.setFocalLength(focal_length));
    EXPECT_EQ(model.getFocalLength(), focal_length);
}

// Test case for setFocalLength with invalid size inputs
TEST(PinholeTest, SetFocalLength_InvalidSizeInputs_ThrowException) {
    Pinhole model;
    std::vector<double> invalid_focal_length = { 2.0 };
    EXPECT_THROW(model.setFocalLength(invalid_focal_length), std::invalid_argument);
}

// Test case for setSkew with valid size inputs
TEST(PinholeTest, SetSkew_ValidSizeInputs_SetParameters) {
    Pinhole model;
    double skew = 0.5;
    EXPECT_NO_THROW(model.setSkew(skew));
    EXPECT_EQ(model.getSkew(), skew);
}

// Test case for setImageSize with valid size inputs
TEST(PinholeTest, SetImageSize_ValidSizeInputs_SetParameters) {
    Pinhole model;
    std::vector<int> image_size = { 640, 480 };
    EXPECT_NO_THROW(model.setImageSize(image_size));
    EXPECT_EQ(model.getImageSize(), image_size);
}

// Test case for setImageSize with invalid size inputs
TEST(PinholeTest, SetImageSize_InvalidSizeInputs_ThrowException) {
    Pinhole model;
    std::vector<int> invalid_image_size = { 0 };
    EXPECT_THROW(model.setImageSize(invalid_image_size), std::invalid_argument);
}

// Test case for getPinhole with valid model returning a Pinhole object with matching parameters
TEST(PinholeTest, GetPinhole_ValidModel_ReturnsPinholeWithMatchingParams) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };

    Pinhole model(focal_length, principal_point, skew, image_size);
    auto model_copy = model.getPinhole();

    EXPECT_EQ(model_copy->getFocalLength(), focal_length);
    EXPECT_EQ(model_copy->getPrincipalPoint(), principal_point);
    EXPECT_EQ(model_copy->getSkew(), skew);
    EXPECT_EQ(model_copy->getImageSize(), image_size);
}

// Test case for getModelName with valid model returning the expected name
TEST(PinholeTest, GetModelName_ValidModel_ReturnsExpectedName) {
    Pinhole model;
    std::string expected_model_name = "Pinhole";
    EXPECT_EQ(model.getModelName(), expected_model_name);
}

// Helper function to write JSON to a file
void writeJsonToFile(const std::string& filePath, const nlohmann::json& j) {
    std::ofstream file(filePath);
    file << j.dump(4); // Pretty print with 4 spaces
}

// Test case for loading valid JSON of the same class resulting in a valid model
TEST(PinholeTest, Load_ValidJsonOfSameClass_ValidModel) {
    std::string filePath = "valid_pinhole.json";
    nlohmann::json valid_json = {
        {"Intrinsics", {
            {"class_name", "PinholeModel"},
            {"focal_distance", {1000, 1000}},
            {"principal_point", {699, 699}},
            {"skew", {0}},
            {"image_size", {1400, 1400}},
            {"coordinate_convention", "TL0_0"}
        }},
        {"Extrinsics", {
            {"rotation", {1.574922167918299, -0.00031742290161591044, 1.5767922159686159}},
            {"translation", {0.018745173150222103, 0.01101567955196766, -0.033125174590079229}},
            {"class_name", "RealObject"}
        }}
    };

    writeJsonToFile(filePath, valid_json);

    auto model = Pinhole::load(filePath);
    EXPECT_EQ(model->getFocalLength(), std::vector<double>({ 1000, 1000 }));
    EXPECT_EQ(model->getPrincipalPoint(), std::vector<double>({ 699, 699 }));
    EXPECT_EQ(model->getImageSize(), std::vector<int>({ 1400, 1400 }));
}

// Test case for loading invalid JSON resulting in throwing an exception
TEST(PinholeTest, Load_InvalidJson_ThrowException) {
    std::string filePath = "invalid_pinhole.json";
    nlohmann::json invalid_json = {
        {"Intrinsics", {
            {"class_name", "PinholeModel"},
            {"focal_length", {1000}}, // Invalid focal length size
            {"principal_point", {699, 699}},
            {"skew", {0}},
            {"image_size", {1400, 1400}},
            {"coordinate_convention", "TL0_0"}
        }},
        {"Extrinsics", {
            {"rotation", {1.574922167918299, -0.00031742290161591044, 1.5767922159686159}},
            {"translation", {0.018745173150222103, 0.01101567955196766, -0.033125174590079229}},
            {"class_name", "RealObject"}
        }}
    };

    writeJsonToFile(filePath, invalid_json);

    EXPECT_THROW(Pinhole::load(filePath), std::invalid_argument);
}

// Test case for loading valid JSON of a different class resulting in throwing an exception
TEST(PinholeTest, Load_ValidJsonOfDifferentClass_ThrowException) {
    std::string filePath = "different_class.json";
    nlohmann::json different_class_json = {
        {"Intrinsics", {
            {"class_name", "DifferentModel"},
            {"focal_length", {1000, 1000}},
            {"principal_point", {699, 699}},
            {"skew", 0},
            {"image_size", {1400, 1400}},
            {"coordinate_convention", "TL0_0"}
        }},
        { "Extrinsics", {
            { "rotation", {1.574922167918299, -0.00031742290161591044, 1.5767922159686159}},
            { "translation", {0.018745173150222103, 0.01101567955196766, -0.033125174590079229}},
            { "class_name", "RealObject"}
        }}
    };

    writeJsonToFile(filePath, different_class_json);

    EXPECT_THROW(Pinhole::load(filePath), std::invalid_argument); // Assuming load throws an exception on JSON of a different class
}