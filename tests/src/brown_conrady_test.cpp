#include <gtest/gtest.h>
#include <stdexcept>
#include <fstream>
#include <vector>
#include <array>
#include "pixeltraq.h"

// Test case for zero-argument constructor
TEST(BrownConradyTest, ZeroArgConstructor_StandardCall_ValidModel) {
    BrownConrady model;
    EXPECT_EQ(model.getFocalLength(), std::vector<double>({ 1, 1 }));
    EXPECT_EQ(model.getPrincipalPoint(), std::vector<double>({ 0, 0 }));
    EXPECT_EQ(model.getImageSize(), std::vector<int>({0,0}));
    EXPECT_EQ(model.getRadialDistCoeffs(), std::vector<double>({ }));
    EXPECT_EQ(model.getTangentialDistCoeffs(), std::vector<double>({}));
    EXPECT_EQ(model.getTangentialPolynominalDistCoeffs(), std::vector<double>({}));
}

// Test case for many-argument constructor with correct arguments
TEST(BrownConradyTest, ManyArgConstructor_StandardCall_ValidModel) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_distortion = { 0.1 };
    std::vector<double> tangential_distortion = { 0.1, 0.1 };
    std::vector<double> tangential_distortion_polycoeff = { 0.1 };

    BrownConrady model(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);

    EXPECT_EQ(model.getFocalLength(), focal_length);
    EXPECT_EQ(model.getPrincipalPoint(), principal_point);
    EXPECT_EQ(model.getImageSize(), image_size);
    EXPECT_EQ(model.getRadialDistCoeffs(), radial_distortion);
    EXPECT_EQ(model.getTangentialDistCoeffs(), tangential_distortion);
    EXPECT_EQ(model.getTangentialPolynominalDistCoeffs(), tangential_distortion_polycoeff);
}

// Test case for many-argument constructor with incorrect sized arguments
TEST(BrownConradyTest, ManyArgConstructor_IncorrectSizedArgs_ThrowException) {
    std::vector<double> incorrect_focal_length = { 2.0 };
    std::vector<double> correct_principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> correct_radial_distortion = { 0.1 };
    std::vector<double> correct_tangential_distortion = { 0.1, 0.1 };
    std::vector<double> correct_tangential_distortion_polycoeff = { 0.1 };

    EXPECT_THROW(BrownConrady(incorrect_focal_length, correct_principal_point, image_size, correct_radial_distortion, correct_tangential_distortion, correct_tangential_distortion_polycoeff), std::invalid_argument);

    std::vector<double> correct_focal_length = { 2.0, 2.0 };
    std::vector<double> incorrect_principal_point = { 1.0 };

    EXPECT_THROW(BrownConrady(correct_focal_length, incorrect_principal_point, image_size, correct_radial_distortion, correct_tangential_distortion, correct_tangential_distortion_polycoeff), std::invalid_argument);

    std::vector<int> incorrect_image_size = { 480 };

    EXPECT_THROW(BrownConrady(correct_focal_length, correct_principal_point, incorrect_image_size, correct_radial_distortion, correct_tangential_distortion, correct_tangential_distortion_polycoeff), std::invalid_argument);
}

// Test case for copy constructor
TEST(BrownConradyTest, CopyConstructor_Called_AllPropertiesIndependent) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_distortion = { 0.1 };
    std::vector<double> tangential_distortion = { 0.1, 0.1 };
    std::vector<double> tangential_distortion_polycoeff = { 0.1 };

    BrownConrady original(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);
    BrownConrady copy(original);

    EXPECT_EQ(copy.getFocalLength(), focal_length);
    EXPECT_EQ(copy.getPrincipalPoint(), principal_point);
    EXPECT_EQ(copy.getImageSize(), image_size);
    EXPECT_EQ(copy.getRadialDistCoeffs(), radial_distortion);
    EXPECT_EQ(copy.getTangentialDistCoeffs(), tangential_distortion);
    EXPECT_EQ(copy.getTangentialPolynominalDistCoeffs(), tangential_distortion_polycoeff);

    // Modify the original and ensure the copy remains unchanged
    focal_length[0] = 3.0;
    principal_point[0] = 2.0;
    image_size = {1,1};
    radial_distortion = {0.2};
    tangential_distortion = {0.2};
    tangential_distortion_polycoeff = {0.2};
    original.setFocalLength(focal_length);
    original.setPrincipalPoint(principal_point);
    original.setImageSize(image_size);
    original.setRadialDistCoeffs(radial_distortion);
    original.setTangentialDistCoeffs(tangential_distortion);
    original.setTangentialPolynominalDistCoeffs(tangential_distortion_polycoeff);

    EXPECT_EQ(copy.getFocalLength(), std::vector<double>({ 2.0, 2.0 }));
    EXPECT_EQ(copy.getPrincipalPoint(), std::vector<double>({ 1.0, 1.0 }));
    EXPECT_EQ(copy.getImageSize(), std::vector<int>({ 640, 480 }));
    EXPECT_EQ(copy.getRadialDistCoeffs(), std::vector<double>({ 0.1 }));
    EXPECT_EQ(copy.getTangentialDistCoeffs(), std::vector<double>({ 0.1, 0.1 }));
    EXPECT_EQ(copy.getTangentialPolynominalDistCoeffs(), std::vector<double>({ 0.1 }));
}

// Test case for project with normal inputs resulting in normal results
TEST(BrownConradyTest, Project_NormalInputs_NormalResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 0.5, 0.5 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_distortion = { 0 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0 };

    BrownConrady model(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);

    std::array<double, 3> point3D = { 1.0, 1.0, 1.0 };
    std::array<double, 2> expected_projection = { 1.5, 1.5 }; // Sample expected result
    EXPECT_EQ(model.project(point3D), expected_projection);

    radial_distortion = { 0.1 };
    tangential_distortion = { 0.1, 0.1 };
    tangential_distortion_polycoeff = { 0 };
    BrownConrady model2(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);

    std::array<double, 3> point3D2 = { 1.0, 1.0, 1.0 };
    std::array<double, 2> expected_projection2 = { 1.7, 1.7 }; // Sample expected result
    EXPECT_EQ(model2.project(point3D2), expected_projection2);
}

// Test case for project with zero inputs resulting in valid results
TEST(BrownConradyTest, Project_ZerosInputs_ValidResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 0, 0 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_distortion = { 0 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0 };

    BrownConrady model(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);

    std::array<double,3> point3D = { 0.0, 0.0, 1.0 };
    std::array<double, 2> expected_projection = { 0.0, 0.0 }; // Sample expected result

    EXPECT_EQ(model.project(point3D), expected_projection);

    std::array<double, 3> point3D2 = { 0.0, 0.0, 0 };
    std::array<double, 2> expected_projection2 = { 1.0e12, 1.0e12 }; // Sample expected result

    auto proj = model.project(point3D2); 
    EXPECT_EQ(model.project(point3D2), expected_projection2);
}

// Test case for project with zero parameters resulting in valid results
TEST(BrownConradyTest, Project_ZeroParameters_ValidResult) {
    BrownConrady model;

    std::array<double, 3> point3D = { 0.0, 0.0, 1.0 };
    std::array<double, 2> expected_projection = { 0.0, 0.0 }; // Sample expected result

    EXPECT_EQ(model.project(point3D), expected_projection);

    std::vector<double> focal_length = { 0.0, 0.0 };
    std::vector<double> principal_point = { 0, 0 };
    std::vector<int> image_size = { 0, 0 };
    std::vector<double> radial_distortion = { 0 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0 };

    BrownConrady model2(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);

    std::array<double, 3> point3D2 = { 1.0, 1.0, 1.0 };
    std::array<double, 2> expected_projection2 = { 0.0, 0.0 }; // Sample expected result

    EXPECT_EQ(model2.project(point3D2), expected_projection2);
}

// Test case for backproject with normal inputs resulting in normal results
TEST(BrownConradyTest, BackProject_NormalInputs_NormalResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 0.5, 0.5 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_distortion = { 0 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0 };

    BrownConrady model(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);

    std::array<double, 2> point2D = { 1.5, 1.5 };
    std::array<double, 3> expected_backprojection = { 1.0, 1.0, 1.0 }; // Sample expected result
    
    auto backprojection = model.backproject(point2D);
    EXPECT_LT(std::abs(backprojection[0] - expected_backprojection[0]),1e-6);
    EXPECT_LT(std::abs(backprojection[1] - expected_backprojection[1]), 1e-6);

    radial_distortion = { 0.1 };
    tangential_distortion = { 0.1, 0.1 };
    tangential_distortion_polycoeff = { 0 };
    BrownConrady model2(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);

    std::array<double, 2> point2D2 = { 1.7, 1.7 };
    std::array<double, 3> expected_backprojection2 = { 1.0, 1.0, 1.0 }; // Sample expected result

    auto backprojection2 = model2.backproject(point2D2);
    EXPECT_LT(std::abs(backprojection2[0] - expected_backprojection2[0]), 1e-6);
    EXPECT_LT(std::abs(backprojection2[1] - expected_backprojection2[1]), 1e-6);
}

// Test case for backproject with zero inputs resulting in valid results
TEST(BrownConradyTest, BackProject_ZerosInputs_ValidResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 0.5, 0.5 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_distortion = { 0 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0 };

    BrownConrady model(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);

    std::array<double, 2> point2D = { 0.5, 0.5 };
    std::array<double, 3> expected_backprojection = { 0.0, 0.0, 1.0 }; // Sample expected result
    EXPECT_EQ(model.backproject(point2D), expected_backprojection);

    radial_distortion = { 0.1 };
    tangential_distortion = { 0.1, 0.1 };
    tangential_distortion_polycoeff = { 0 };

    BrownConrady model2(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);

    std::array<double, 2> point2D2 = { 0.5, 0.5 };
    std::array<double, 3> expected_backprojection2 = { 0.0, 0.0, 1.0 }; // Sample expected result
    EXPECT_EQ(model2.backproject(point2D2), expected_backprojection2);
}

// Test case for backproject with zero parameters resulting in valid results
TEST(BrownConradyTest, BackProject_ZeroParameters_ValidResult) {
    BrownConrady model;

    std::array<double, 2> point2D = { 0.0, 0.0 };
    std::array<double, 3> expected_backprojection = { 0.0, 0.0, 1.0 }; // Sample expected result

    EXPECT_EQ(model.backproject(point2D), expected_backprojection);

    std::vector<double> focal_length = { 0.0, 0.0 };
    std::vector<double> principal_point = { 0, 0 };
    std::vector<int> image_size = { 0, 0 };
    std::vector<double> radial_distortion = { 0 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0 };

    BrownConrady model2(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);

    std::array<double, 2> point2D2 = { 0.0, 0.0 };
    std::array<double, 3> expected_backprojection2 = { 1.0e12, 1.0e12, 1.0 }; // Sample expected result
    auto backprojection2 = model2.backproject(point2D2);
    EXPECT_GT(std::abs(backprojection2[0]), 1e10);
    EXPECT_GT(std::abs(backprojection2[1]), 1e10);
}

// Test case for getPrincipalPoint using zero-argument constructor
TEST(BrownConradyTest, GetPrincipalPoint_ZeroArgConstructor_ValidResult) {
    BrownConrady model;
    std::vector<double> expected_principal_point = { 0.0, 0.0 };
    EXPECT_EQ(model.getPrincipalPoint(), expected_principal_point);
}

// Test case for getPrincipalPoint using many-argument constructor
TEST(BrownConradyTest, GetPrincipalPoint_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 1, 1 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_distortion = { 0 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0 };

    BrownConrady model(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);
    EXPECT_EQ(model.getPrincipalPoint(), principal_point);
}

// Test case for getFocalLength using zero-argument constructor
TEST(BrownConradyTest, GetFocalLength_ZeroArgConstructor_ValidResult) {
    BrownConrady model;
    std::vector<double> expected_focal_length = { 1.0, 1.0 };
    EXPECT_EQ(model.getFocalLength(), expected_focal_length);
}

// Test case for getFocalLength using many-argument constructor
TEST(BrownConradyTest, GetFocalLength_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 1, 1 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_distortion = { 0 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0 };

    BrownConrady model(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);
    EXPECT_EQ(model.getFocalLength(), focal_length);
}

// Test case for getRadialDistortionCoeff using zero-argument constructor
TEST(BrownConradyTest, GetRadialDistortionCoeff_ZeroArgConstructor_ValidResult) {
    BrownConrady model;
    std::vector<double> expected_coeffs = { };
    EXPECT_EQ(model.getRadialDistCoeffs(), expected_coeffs);
}

// Test case for getRadialDistortionCoeff using many-argument constructor
TEST(BrownConradyTest, GetRadialDistortionCoeff_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 1, 1 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_distortion = { 0.1 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0 };

    BrownConrady model(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);
    EXPECT_EQ(model.getRadialDistCoeffs(), radial_distortion);
}

// Test case for getTangentialDistortionCoeff using zero-argument constructor
TEST(BrownConradyTest, GetTangentialDistortionCoeff_ZeroArgConstructor_ValidResult) {
    BrownConrady model;
    std::vector<double> expected_coeffs = { };
    EXPECT_EQ(model.getTangentialDistCoeffs(), expected_coeffs);
}

// Test case for getTangentialDistortionCoeff using many-argument constructor
TEST(BrownConradyTest, GetTangentialDistortionCoeff_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 1, 1 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_distortion = { 0 };
    std::vector<double> tangential_distortion = { 0.1, 0.1};
    std::vector<double> tangential_distortion_polycoeff = { 0 };

    BrownConrady model(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);
    EXPECT_EQ(model.getTangentialDistCoeffs(), tangential_distortion);
}

// Test case for getTangentialDistortionCoeff using zero-argument constructor
TEST(BrownConradyTest, GetTangentialPolynominalDistCoeffs_ZeroArgConstructor_ValidResult) {
    BrownConrady model;
    std::vector<double> expected_coeffs = { };
    EXPECT_EQ(model.getTangentialPolynominalDistCoeffs(), expected_coeffs);
}

// Test case for getTangentialDistortionCoeff using many-argument constructor
TEST(BrownConradyTest, GetTangentialPolynominalDistCoeffs_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 1, 1 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_distortion = { 0 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0.1 };

    BrownConrady model(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);
    EXPECT_EQ(model.getTangentialPolynominalDistCoeffs(), tangential_distortion_polycoeff);
}

// Test case for getImageSize using zero-argument constructor
TEST(BrownConradyTest, GetImageSize_ZeroArgConstructor_ValidResult) {
    BrownConrady model;
    std::vector<int> expected_image_size = { 0,0 };
    EXPECT_EQ(model.getImageSize(), expected_image_size);
}

// Test case for getImageSize using many-argument constructor
TEST(BrownConradyTest, GetImageSize_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 1, 1 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_distortion = { 0 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0.1 };

    BrownConrady model(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);
    EXPECT_EQ(model.getImageSize(), image_size);
}

// Test case for setPrincipalPoint with valid size inputs
TEST(BrownConradyTest, SetPrincipalPoint_ValidSizeInputs_SetParameters) {
    BrownConrady model;
    std::vector<double> principal_point = { 1.0, 1.0 };
    EXPECT_NO_THROW(model.setPrincipalPoint(principal_point));
    EXPECT_EQ(model.getPrincipalPoint(), principal_point);
}

// Test case for setPrincipalPoint with invalid size inputs
TEST(BrownConradyTest, SetPrincipalPoint_InvalidSizeInputs_ThrowException) {
    BrownConrady model;
    std::vector<double> invalid_principal_point = { 1.0 };
    EXPECT_THROW(model.setPrincipalPoint(invalid_principal_point), std::invalid_argument);
}

// Test case for setFocalLength with valid size inputs
TEST(BrownConradyTest, SetFocalLength_ValidSizeInputs_SetParameters) {
    BrownConrady model;
    std::vector<double> focal_length = { 2.0, 2.0 };
    EXPECT_NO_THROW(model.setFocalLength(focal_length));
    EXPECT_EQ(model.getFocalLength(), focal_length);
}

// Test case for setFocalLength with invalid size inputs
TEST(BrownConradyTest, SetFocalLength_InvalidSizeInputs_ThrowException) {
    BrownConrady model;
    std::vector<double> invalid_focal_length = { 2.0 };
    EXPECT_THROW(model.setFocalLength(invalid_focal_length), std::invalid_argument);
}

// Test case for setRadialDistortion with valid size inputs
TEST(BrownConradyTest, SetRadialDistortion_ValidSizeInputs_SetParameters) {
    BrownConrady model;
    std::vector<double> coeffs = { 0.1, 0.2 };
    EXPECT_NO_THROW(model.setRadialDistCoeffs(coeffs));
    EXPECT_EQ(model.getRadialDistCoeffs(), coeffs);
}

// Test case for setTangentialDistortion with valid size inputs
TEST(BrownConradyTest, SetTangentialDistCoeffs_ValidSizeInputs_SetParameters) {
    BrownConrady model;
    std::vector<double> coeffs = { 0.1, 0.2 };
    EXPECT_NO_THROW(model.setTangentialDistCoeffs(coeffs));
    EXPECT_EQ(model.getTangentialDistCoeffs(), coeffs);
}

// Test case for setTangentialDistortionPoly with valid size inputs
TEST(BrownConradyTest, SetTangentialDistPolyCoeffs_ValidSizeInputs_SetParameters) {
    BrownConrady model;
    std::vector<double> coeffs = { 0.1, 0.2 };
    EXPECT_NO_THROW(model.setTangentialPolynominalDistCoeffs(coeffs));
    EXPECT_EQ(model.getTangentialPolynominalDistCoeffs(), coeffs);
}

// Test case for setImageSize with valid size inputs
TEST(BrownConradyTest, SetImageSize_ValidSizeInputs_SetParameters) {
    BrownConrady model;
    std::vector<int> image_size = { 640, 480 };
    EXPECT_NO_THROW(model.setImageSize(image_size));
    EXPECT_EQ(model.getImageSize(), image_size);
}

// Test case for setImageSize with invalid size inputs
TEST(BrownConradyTest, SetImageSize_InvalidSizeInputs_ThrowException) {
    BrownConrady model;
    std::vector<int> invalid_image_size = { 0 };
    EXPECT_THROW(model.setImageSize(invalid_image_size), std::invalid_argument);
}

// Test case for getPinhole with valid model returning a Pinhole object with matching parameters
TEST(BrownConradyTest, GetPinhole_ValidModel_ReturnsPinholeWithMatchingParams) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 1, 1 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_distortion = { 0 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0 };

    BrownConrady model(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);
    auto pinhole = model.getPinhole();

    EXPECT_EQ(pinhole->getFocalLength(), focal_length);
    EXPECT_EQ(pinhole->getPrincipalPoint(), principal_point);
    EXPECT_EQ(pinhole->getSkew(), 0);
    EXPECT_EQ(pinhole->getImageSize(), image_size); // Assuming getImageSize is a method in Camera class
}

// Test case for getModelName with valid model returning the expected name
TEST(BrownConradyTest, GetModelName_ValidModel_ReturnsExpectedName) {
    BrownConrady model;
    std::string expected_model_name = "Brown Conrady";
    EXPECT_EQ(model.getModelName(), expected_model_name);
}

// Helper function to write JSON to a file
void writeJsonToFile2(const std::string& filePath, const nlohmann::json& j) {
    std::ofstream file(filePath);
    file << j.dump(4); // Pretty print with 4 spaces
}

// Test case for loading valid JSON of the same class resulting in a valid model
TEST(BrownConradyTest, Load_ValidJsonOfSameClass_ValidModel) {
    std::string filePath = "valid_model.json";
    nlohmann::json valid_json = {
        {"Intrinsics", {
            {"class_name", "BrownConradyModel"},
            {"EFL", {800, 800}},
            {"principal_point", {699, 699}},
            {"skew", {0}},
            {"image_size", {1400, 1400}},
            {"coordinate_convention", "TL0_0"},
            {"radial_distortion_coeff", {0.1, 0, 0}},
            {"tangential_distortion_coeff", {0, 0}},
            {"tangential_distortion_poly_coeff", {0, 0, 0}},
            {"projection", "rectilinear"}
          }
        },
        {
          "Extrinsics", {
            {"rotation", {-1.5737876015474659, 0.0014733213448178063, -1.5777673749569177}},
            {"translation", {-0.013763134850525039, -0.018929691821162747, -0.087534582835381888}},
            {"class_name", "RealObject"}
          }
        }
    };

    writeJsonToFile2(filePath, valid_json);

    auto model = BrownConrady::load(filePath);
    EXPECT_EQ(model->getFocalLength(), std::vector<double>({ 800, 800 }));
    EXPECT_EQ(model->getPrincipalPoint(), std::vector<double>({ 699, 699 }));
    EXPECT_EQ(model->getImageSize(), std::vector<int>({ 1400, 1400 }));
    EXPECT_EQ(model->getRadialDistCoeffs(), std::vector<double>({ 0.1, 0, 0 }));
    EXPECT_EQ(model->getTangentialDistCoeffs(), std::vector<double>({ 0, 0 }));
    EXPECT_EQ(model->getTangentialPolynominalDistCoeffs(), std::vector<double>({ 0, 0, 0 }));
}

// Test case for loading invalid JSON resulting in throwing an exception
TEST(BrownConradyTest, Load_InvalidJson_ThrowException) {
    std::string filePath = "invalid_brownConrady.json";
    nlohmann::json invalid_json = {
        {"Intrinsics", {
            {"class_name", "BrownConradyModel"},
            {"EFL", {800}}, // Invalid focal length size
            {"principal_point", {699, 699}},
            {"skew", {0}},
            {"image_size", {1400, 1400}},
            {"coordinate_convention", "TL0_0"},
            {"radial_distortion_coeff", {0.1, 0, 0}},
            {"tangential_distortion_coeff", {0, 0}},
            {"tangential_distortion_poly_coeff", {0, 0, 0}},
            {"projection", "rectilinear"}
          }
        },
        {
          "Extrinsics", {
            {"rotation", {-1.5737876015474659, 0.0014733213448178063, -1.5777673749569177}},
            {"translation", {-0.013763134850525039, -0.018929691821162747, -0.087534582835381888}},
            {"class_name", "RealObject"}
          }
        }
    };

    writeJsonToFile2(filePath, invalid_json);

    EXPECT_THROW(BrownConrady::load(filePath), std::invalid_argument);
}

// Test case for loading valid JSON of a different class resulting in throwing an exception
TEST(BrownConradyTest, Load_ValidJsonOfDifferentClass_ThrowException) {
    std::string filePath = "different_class.json";
    nlohmann::json different_class_json = {
        {"Intrinsics", {
            {"class_name", "InvalidModel"},
            {"EFL", {800, 800}},
            {"principal_point", {699, 699}},
            {"skew", {0}},
            {"image_size", {1400, 1400}},
            {"coordinate_convention", "TL0_0"},
            {"radial_distortion_coeff", {0.1, 0, 0}},
            {"tangential_distortion_coeff", {0, 0}},
            {"tangential_distortion_poly_coeff", {0, 0, 0}},
            {"projection", "rectilinear"}
          }
        },
        {
          "Extrinsics", {
            {"rotation", {-1.5737876015474659, 0.0014733213448178063, -1.5777673749569177}},
            {"translation", {-0.013763134850525039, -0.018929691821162747, -0.087534582835381888}},
            {"class_name", "RealObject"}
          }
        }
    };

    writeJsonToFile2(filePath, different_class_json);

    EXPECT_THROW(BrownConrady::load(filePath), std::invalid_argument); // Assuming load throws an exception on JSON of a different class
}

// Test case for getBackprojectSettings with default values
TEST(BrownConradyTest, getBackprojectSettings_default_returnsexpected) {
    BrownConrady model;
    double expected_threshold = 1e-6;
    int expected_iterations = 20;

    double threshold;
    int iterations;
    model.getBackprojectSettings(threshold, iterations);

    EXPECT_DOUBLE_EQ(threshold, expected_threshold);
    EXPECT_EQ(iterations, expected_iterations);
}

// Test case for setBackprojectSettings and then getBackprojectSettings
TEST(BrownConradyTest, setBackprojectSettings_afterget_returnsexpected) {
    BrownConrady model;
    double set_threshold = 0.5;
    int set_iterations = 10;

    model.setBackprojectSettings(set_threshold, set_iterations);

    double threshold;
    int iterations;
    model.getBackprojectSettings(threshold, iterations);

    EXPECT_DOUBLE_EQ(threshold, set_threshold);
    EXPECT_EQ(iterations, set_iterations);
}

// Test case for setBackprojectSettings with invalid inputs
TEST(BrownConradyTest, setBackprojectSettings_invalidinputs_throwsexception) {
    BrownConrady model;

    double invalid_threshold = -0.5;
    int invalid_iterations = -10;

    EXPECT_THROW(model.setBackprojectSettings(invalid_threshold, invalid_iterations), std::invalid_argument);
}