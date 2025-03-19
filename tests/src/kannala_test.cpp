#include <gtest/gtest.h>
#include <stdexcept>
#include <fstream>
#include <vector>
#include <array>
#include "pixeltraq.h"

// Test case for zero-argument constructor
TEST(KannalaTest, ZeroArgConstructor_StandardCall_ValidModel) {
    Kannala model;
    EXPECT_EQ(model.getFocalLength(), std::vector<double>({ 1, 1 }));
    EXPECT_EQ(model.getPrincipalPoint(), std::vector<double>({ 0, 0 }));
    EXPECT_EQ(model.getImageSize(), std::vector<int>({0,0}));
    EXPECT_EQ(model.getRadialDistSymCoeffs(), std::vector<double>({ }));
    EXPECT_EQ(model.getRadialDistAsymCoeffs(), std::vector<double>({ 0 }));
    EXPECT_EQ(model.getRadialDistFourCoeffs(), std::vector<double>({ 0 , 0 }));
    EXPECT_EQ(model.getTangentialDistAsymCoeffs(), std::vector<double>({ 0 }));
    EXPECT_EQ(model.getTangentialDistFourCoeffs(), std::vector<double>({ 0 , 0 }));
}

// Test case for many-argument constructor with correct arguments
TEST(KannalaTest, ManyArgConstructor_StandardCall_ValidModel) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_dist_sym = { 0.1 };
    std::vector<double> radial_distortion_asym = { 0.1, 0.1 };
    std::vector<double> radial_distortion_four = { 0.1 , 0.1};
    std::vector<double> tangential_distortion_asym = { 0.1 };
    std::vector<double> tangential_distortion_four = { 0.1 , 0.1};

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);

    EXPECT_EQ(model.getFocalLength(), focal_length);
    EXPECT_EQ(model.getPrincipalPoint(), principal_point);
    EXPECT_EQ(model.getImageSize(), image_size);
    EXPECT_EQ(model.getRadialDistSymCoeffs(), radial_dist_sym);
    EXPECT_EQ(model.getRadialDistAsymCoeffs(), radial_distortion_asym);
    EXPECT_EQ(model.getRadialDistFourCoeffs(), radial_distortion_four);
    EXPECT_EQ(model.getTangentialDistAsymCoeffs(), tangential_distortion_asym);
    EXPECT_EQ(model.getTangentialDistFourCoeffs(), tangential_distortion_four);
}

// Test case for many-argument constructor with incorrect sized arguments
TEST(KannalaTest, ManyArgConstructor_IncorrectSizedArgs_ThrowException) {
    std::vector<double> incorrect_focal_length = { 2.0 };
    std::vector<double> correct_principal_point = { 1.0, 1.0 };
    std::vector<int> correct_image_size = { 640, 480 };
    std::vector<double> correct_radial_dist_sym = { 0.1 };
    std::vector<double> correct_radial_distortion_asym = { 0.1, 0.1 };
    std::vector<double> correct_radial_distortion_four = { 0.1 , 0.1};
    std::vector<double> correct_tangential_distortion_asym = { 0.1 };
    std::vector<double> correct_tangential_distortion_four = { 0.1 , 0.1 };

    EXPECT_THROW(Kannala(incorrect_focal_length, correct_principal_point, correct_image_size, correct_radial_dist_sym, correct_radial_distortion_asym, correct_radial_distortion_four, correct_tangential_distortion_asym, correct_tangential_distortion_four), std::invalid_argument);

    std::vector<double> correct_focal_length = { 2.0, 2.0 };
    std::vector<double> incorrect_principal_point = { 1.0 };

    EXPECT_THROW(Kannala(correct_focal_length, incorrect_principal_point, correct_image_size, correct_radial_dist_sym, correct_radial_distortion_asym, correct_radial_distortion_four, correct_tangential_distortion_asym, correct_tangential_distortion_four), std::invalid_argument);

    std::vector<int> incorrect_image_size = { 480 };

    EXPECT_THROW(Kannala(correct_focal_length, correct_principal_point, incorrect_image_size, correct_radial_dist_sym, correct_radial_distortion_asym, correct_radial_distortion_four, correct_tangential_distortion_asym, correct_tangential_distortion_four), std::invalid_argument);
}

// Test case for copy constructor
TEST(KannalaTest, CopyConstructor_Called_AllPropertiesIndependent) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_dist_sym = { 0.1 };
    std::vector<double> radial_distortion_asym = { 0.1, 0.1 };
    std::vector<double> radial_distortion_four = { 0.1 , 0.1 };
    std::vector<double> tangential_distortion_asym = { 0.1 };
    std::vector<double> tangential_distortion_four = { 0.1 , 0.1};

    Kannala original(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);
    Kannala copy(original);

    EXPECT_EQ(copy.getFocalLength(), focal_length);
    EXPECT_EQ(copy.getPrincipalPoint(), principal_point);
    EXPECT_EQ(copy.getImageSize(), image_size);
    EXPECT_EQ(copy.getRadialDistSymCoeffs(), radial_dist_sym);
    EXPECT_EQ(copy.getRadialDistAsymCoeffs(), radial_distortion_asym);
    EXPECT_EQ(copy.getRadialDistFourCoeffs(), radial_distortion_four);
    EXPECT_EQ(copy.getTangentialDistAsymCoeffs(), tangential_distortion_asym);
    EXPECT_EQ(copy.getTangentialDistFourCoeffs(), tangential_distortion_four);

    // Modify the original and ensure the copy remains unchanged
    focal_length[0] = 3.0;
    principal_point[0] = 2.0;
    image_size = {1,1};
    radial_dist_sym = { 0.2 };
    radial_distortion_asym = { 0 };
    radial_distortion_four = { 0 , 0};
    tangential_distortion_asym = { 0 };
    tangential_distortion_four = { 0 , 0};
    original.setFocalLength(focal_length);
    original.setPrincipalPoint(principal_point);
    original.setImageSize(image_size);
    original.setRadialDistSymCoeffs(radial_dist_sym);
    original.setRadialDistAsymCoeffs(radial_distortion_asym);
    original.setRadialDistFourCoeffs(tangential_distortion_four);
    original.setTangentialDistAsymCoeffs(tangential_distortion_asym);
    original.setTangentialDistFourCoeffs(tangential_distortion_four);

    EXPECT_EQ(copy.getFocalLength(), std::vector<double>({ 2.0, 2.0 }));
    EXPECT_EQ(copy.getPrincipalPoint(), std::vector<double>({ 1.0, 1.0 }));
    EXPECT_EQ(copy.getImageSize(), std::vector<int>({ 640, 480 }));
    EXPECT_EQ(copy.getRadialDistSymCoeffs(), std::vector<double>({ 0.1 }));
    EXPECT_EQ(copy.getRadialDistAsymCoeffs(), std::vector<double>({ 0.1, 0.1 }));
    EXPECT_EQ(copy.getRadialDistFourCoeffs(), std::vector<double>({ 0.1 , 0.1 }));
    EXPECT_EQ(copy.getTangentialDistAsymCoeffs(), std::vector<double>({ 0.1 }));
    EXPECT_EQ(copy.getTangentialDistFourCoeffs(), std::vector<double>({ 0.1 , 0.1}));
}

// Test case for project with normal inputs resulting in normal results
TEST(KannalaTest, Project_NormalInputs_NormalResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 0.5, 0.5 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_dist_sym = { 0 };
    std::vector<double> radial_distortion_asym = { 0, 0 };
    std::vector<double> radial_distortion_four = { 0, 0 };
    std::vector<double> tangential_distortion_asym = { 0 };
    std::vector<double> tangential_distortion_four = { 0, 0 };

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);

    std::array<double, 3> point3D = { 1.0, 1.0, 1.0 };
    std::array<double, 2> expected_projection = { 1.1755108, 1.1755108 }; // Sample expected result
    auto projection = model.project(point3D);
    EXPECT_LT(std::abs(projection[0] - expected_projection[0]), 1e-4);
    EXPECT_LT(std::abs(projection[1] - expected_projection[1]), 1e-4);

    radial_dist_sym = { 0.1 };
    radial_distortion_asym = { 0.1 };
    radial_distortion_four = { 0.1, 0.1 };
    tangential_distortion_asym = { 0.1 };
    tangential_distortion_four = { 0.1, 0.1 };

    Kannala model2(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);

    std::array<double, 3> point3D2 = { 1.0, 1.0, 1.0 };
    std::array<double, 2> expected_projection2 = { 1.23715999, 1.25626632 }; // Sample expected result
    auto projection2 = model2.project(point3D2);
    EXPECT_LT(std::abs(projection2[0] - expected_projection2[0]), 1e-4);
    EXPECT_LT(std::abs(projection2[1] - expected_projection2[1]), 1e-4);
}

// Test case for project with zero inputs resulting in valid results
TEST(KannalaTest, Project_ZerosInputs_ValidResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 0.5, 0.5 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_dist_sym = { 0 };
    std::vector<double> radial_distortion_asym = { 0, 0 };
    std::vector<double> radial_distortion_four = { 0 , 0};
    std::vector<double> tangential_distortion_asym = { 0 };
    std::vector<double> tangential_distortion_four = { 0 , 0 };

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);

    std::array<double,3> point3D = { 0.0, 0.0, 1.0 };
    std::array<double, 2> expected_projection = { 0.5, 0.5 }; // Sample expected result

    EXPECT_EQ(model.project(point3D), expected_projection);

    std::array<double, 3> point3D2 = { 0.0, 0.0, 0.0 };
    std::array<double, 2> expected_projection2 = { 1.0e12, 1.0e12 }; // Sample expected result

    auto proj = model.project(point3D2); 
    EXPECT_EQ(model.project(point3D2), expected_projection2);
}

// Test case for project with zero parameters resulting in valid results
TEST(KannalaTest, Project_ZeroParameters_ValidResult) {
    Kannala model;

    std::array<double, 3> point3D = { 0.0, 0.0, 1.0 };
    std::array<double, 2> expected_projection = { 0.0, 0.0 }; // Sample expected result

    EXPECT_EQ(model.project(point3D), expected_projection);

    std::vector<double> focal_length = { 0.0, 0.0 };
    std::vector<double> principal_point = { 0.0, 0.0 };
    std::vector<int> image_size = { 0, 0 };
    std::vector<double> radial_dist_sym = { 0 };
    std::vector<double> radial_distortion_asym = { 0, 0 };
    std::vector<double> radial_distortion_four = { 0 };
    std::vector<double> tangential_distortion_asym = { 0 };
    std::vector<double> tangential_distortion_four = { 0 };

    Kannala model2(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);

    std::array<double, 3> point3D2 = { 1.0, 1.0, 1.0 };
    std::array<double, 2> expected_projection2 = { 0.0, 0.0 }; // Sample expected result

    EXPECT_EQ(model2.project(point3D2), expected_projection2);
}

// Test case for backproject with normal inputs resulting in normal results
TEST(KannalaTest, BackProject_NormalInputs_NormalResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 0.5, 0.5 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_dist_sym = { 0 };
    std::vector<double> radial_distortion_asym = { 0, 0 };
    std::vector<double> radial_distortion_four = { 0 , 0 };
    std::vector<double> tangential_distortion_asym = { 0 };
    std::vector<double> tangential_distortion_four = { 0 , 0 };

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);
    
    std::array<double, 2> point2D = { 1.1755108, 1.1755108 };
    std::array<double, 3> expected_backprojection = { 1.0, 1.0, 1.0 }; // Sample expected result
    
    auto backprojection = model.backproject(point2D);
    EXPECT_LT(std::abs(backprojection[0] - expected_backprojection[0]),1e-6);
    EXPECT_LT(std::abs(backprojection[1] - expected_backprojection[1]), 1e-6);

    radial_dist_sym = { 0.1 };
    radial_distortion_asym = { 0.1 };
    radial_distortion_four = { 0.1 , 0.1 };
    tangential_distortion_asym = { 0.1 };
    tangential_distortion_four = { 0.1 , 0.1 };
    Kannala model2(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);

    std::array<double, 2> point2D2 = { 1.23715999, 1.25626632 };
    std::array<double, 3> expected_backprojection2 = { 1.0, 1.0, 1.0 }; // Sample expected result

    auto backprojection2 = model2.backproject(point2D2);
    EXPECT_LT(std::abs(backprojection2[0] - expected_backprojection2[0]), 1e-6);
    EXPECT_LT(std::abs(backprojection2[1] - expected_backprojection2[1]), 1e-6);
}

// Test case for backproject with zero inputs resulting in valid results
TEST(KannalaTest, BackProject_ZerosInputs_ValidResult) {
    std::vector<double> focal_length = { 1.0, 1.0 };
    std::vector<double> principal_point = { 0.5, 0.5 };
    std::vector<int> image_size = { 2, 2 };
    std::vector<double> radial_dist_sym = { 0 };
    std::vector<double> radial_distortion_asym = { 0, 0 };
    std::vector<double> radial_distortion_four = { 0 , 0 };
    std::vector<double> tangential_distortion_asym = { 0 };
    std::vector<double> tangential_distortion_four = { 0 , 0 };

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);

    std::array<double, 2> point2D = { 0.5, 0.5 };
    std::array<double, 3> expected_backprojection = { 0.0, 0.0, 1.0 }; // Sample expected result
    EXPECT_EQ(model.backproject(point2D), expected_backprojection);

    radial_dist_sym = { 0.1 };
    radial_distortion_asym = { 0.1 };
    radial_distortion_four = { 0.1 , 0.1 };
    tangential_distortion_asym = { 0.1 };
    tangential_distortion_four = { 0.1 , 0.1 };
    Kannala model2(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);

    std::array<double, 2> point2D2 = { 0.5, 0.5 };
    std::array<double, 3> expected_backprojection2 = { 0.0, 0.0, 1.0 }; // Sample expected result
    EXPECT_EQ(model2.backproject(point2D2), expected_backprojection2);
}

// Test case for backproject with zero parameters resulting in valid results
TEST(KannalaTest, BackProject_ZeroParameters_ValidResult) {
    Kannala model;

    std::array<double, 2> point2D = { 0.0, 0.0 };
    std::array<double, 3> expected_backprojection = { 0.0, 0.0, 1.0 }; // Sample expected result

    EXPECT_EQ(model.backproject(point2D), expected_backprojection);

    std::vector<double> focal_length = { 0.0, 0.0 };
    std::vector<double> principal_point = { 0.0, 0.0 };
    std::vector<int> image_size = { 0, 0 };
    std::vector<double> radial_dist_sym = { 0 };
    std::vector<double> radial_distortion_asym = { 0, 0 };
    std::vector<double> radial_distortion_four = { 0 , 0 };
    std::vector<double> tangential_distortion_asym = { 0 };
    std::vector<double> tangential_distortion_four = { 0 , 0 };

    Kannala model2(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);

    std::array<double, 2> point2D2 = { 0.0, 0.0 };
    std::array<double, 3> expected_backprojection2 = { 1.0e12, 1.0e12, 1.0 }; // Sample expected result
    auto backprojection2 = model2.backproject(point2D2);
    EXPECT_GT(std::abs(backprojection2[0]), 1e10);
    EXPECT_GT(std::abs(backprojection2[1]), 1e10);
}

// Test case for getPrincipalPoint using zero-argument constructor
TEST(KannalaTest, GetPrincipalPoint_ZeroArgConstructor_ValidResult) {
    Kannala model;
    std::vector<double> expected_principal_point = { 0.0, 0.0 };
    EXPECT_EQ(model.getPrincipalPoint(), expected_principal_point);
}

// Test case for getPrincipalPoint using many-argument constructor
TEST(KannalaTest, GetPrincipalPoint_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_dist_sym = { 0.1 };
    std::vector<double> radial_distortion_asym = { 0.1, 0.1 };
    std::vector<double> radial_distortion_four = { 0.1 , 0.1};
    std::vector<double> tangential_distortion_asym = { 0.1 };
    std::vector<double> tangential_distortion_four = { 0.1 , 0.1};

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);
    EXPECT_EQ(model.getPrincipalPoint(), principal_point);
}

// Test case for getFocalLength using zero-argument constructor
TEST(KannalaTest, GetFocalLength_ZeroArgConstructor_ValidResult) {
    Kannala model;
    std::vector<double> expected_focal_length = { 1.0, 1.0 };
    EXPECT_EQ(model.getFocalLength(), expected_focal_length);
}

// Test case for getFocalLength using many-argument constructor
TEST(KannalaTest, GetFocalLength_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_dist_sym = { 0.1 };
    std::vector<double> radial_distortion_asym = { 0.1, 0.1 };
    std::vector<double> radial_distortion_four = { 0.1 , 0.1};
    std::vector<double> tangential_distortion_asym = { 0.1 };
    std::vector<double> tangential_distortion_four = { 0.1 , 0.1};

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);
    EXPECT_EQ(model.getFocalLength(), focal_length);
}

// Test case for getRadialDistSymCoeff using zero-argument constructor
TEST(KannalaTest, GetRadialDistortionCoeff_ZeroArgConstructor_ValidResult) {
    Kannala model;
    std::vector<double> expected_coeffs = { };
    EXPECT_EQ(model.getRadialDistSymCoeffs(), expected_coeffs);
}

// Test case for getRadialDistSymCoeff using many-argument constructor
TEST(KannalaTest, GetRadialDistortionCoeff_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_dist_sym = { 0.1 };
    std::vector<double> radial_distortion_asym = { 0.1, 0.1 };
    std::vector<double> radial_distortion_four = { 0.1 , 0.1};
    std::vector<double> tangential_distortion_asym = { 0.1 };
    std::vector<double> tangential_distortion_four = { 0.1 , 0.1};

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);
    EXPECT_EQ(model.getRadialDistSymCoeffs(), radial_dist_sym);
}

// Test case for getRadialDistAsymCoeff using zero-argument constructor
TEST(KannalaTest, GetRadialDistortionAsymCoeff_ZeroArgConstructor_ValidResult) {
    Kannala model;
    std::vector<double> expected_coeffs = { 0 };
    EXPECT_EQ(model.getRadialDistAsymCoeffs(), expected_coeffs);
}

// Test case for getRadialDistortionCoeff using many-argument constructor
TEST(KannalaTest, GetRadialDistortionAsymCoeff_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_dist_sym = { 0.1 };
    std::vector<double> radial_distortion_asym = { 0.1, 0.1 };
    std::vector<double> radial_distortion_four = { 0.1 , 0.1};
    std::vector<double> tangential_distortion_asym = { 0.1 };
    std::vector<double> tangential_distortion_four = { 0.1 , 0.1};

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);
    EXPECT_EQ(model.getRadialDistAsymCoeffs(), radial_distortion_asym);
}

// Test case for GetRadialDistFourCoeffs using zero-argument constructor
TEST(KannalaTest, GetRadialDistFourCoeffs_ZeroArgConstructor_ValidResult) {
    Kannala model;
    std::vector<double> expected_coeffs = { 0 , 0 };
    EXPECT_EQ(model.getRadialDistFourCoeffs(), expected_coeffs);
}

// Test case for GetRadialDistFourCoeffs using many-argument constructor
TEST(KannalaTest, GetRadialDistFourCoeffs_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_dist_sym = { 0.1 };
    std::vector<double> radial_distortion_asym = { 0.1, 0.1 };
    std::vector<double> radial_distortion_four = { 0.2 , 0.1};
    std::vector<double> tangential_distortion_asym = { 0.3 };
    std::vector<double> tangential_distortion_four = { 0.4 , 0.1};

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);
    EXPECT_EQ(model.getRadialDistFourCoeffs(), radial_distortion_four);
}

// Test case for GetTangentialDistAsymCoeffs using zero-argument constructor
TEST(KannalaTest, GetTangentialDistAsymCoeffs_ZeroArgConstructor_ValidResult) {
    Kannala model;
    std::vector<double> expected_coeffs = { 0 };
    EXPECT_EQ(model.getTangentialDistAsymCoeffs(), expected_coeffs);
}

// Test case for GetTangentialDistAsymCoeffs using many-argument constructor
TEST(KannalaTest, GetTangentialDistAsymCoeffs_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_dist_sym = { 0.1 };
    std::vector<double> radial_distortion_asym = { 0.1, 0.1 };
    std::vector<double> radial_distortion_four = { 0.2 , 0.1};
    std::vector<double> tangential_distortion_asym = { 0.3 };
    std::vector<double> tangential_distortion_four = { 0.4 , 0.1};

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);
    EXPECT_EQ(model.getTangentialDistAsymCoeffs(), tangential_distortion_asym);
}

// Test case for GetTangentialDistFourCoeffs using zero-argument constructor
TEST(KannalaTest, GetTangentialDistFourCoeffs_ZeroArgConstructor_ValidResult) {
    Kannala model;
    std::vector<double> expected_coeffs = { 0 , 0};
    EXPECT_EQ(model.getTangentialDistFourCoeffs(), expected_coeffs);
}

// Test case for GetTangentialDistFourCoeffs using many-argument constructor
TEST(KannalaTest, GetTangentialDistFourCoeffs_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_dist_sym = { 0.1 };
    std::vector<double> radial_distortion_asym = { 0.1, 0.1 };
    std::vector<double> radial_distortion_four = { 0.2 , 0.1};
    std::vector<double> tangential_distortion_asym = { 0.3 };
    std::vector<double> tangential_distortion_four = { 0.4 , 0.1};

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);
    EXPECT_EQ(model.getTangentialDistFourCoeffs(), tangential_distortion_four);
}

// Test case for getImageSize using zero-argument constructor
TEST(KannalaTest, GetImageSize_ZeroArgConstructor_ValidResult) {
    Kannala model;
    std::vector<int> expected_image_size = { 0,0 };
    EXPECT_EQ(model.getImageSize(), expected_image_size);
}

// Test case for getImageSize using many-argument constructor
TEST(KannalaTest, GetImageSize_ManyArgConstructor_ValidResult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_dist_sym = { 0.1 };
    std::vector<double> radial_distortion_asym = { 0.1, 0.1 };
    std::vector<double> radial_distortion_four = { 0.1 , 0.1};
    std::vector<double> tangential_distortion_asym = { 0.1 };
    std::vector<double> tangential_distortion_four = { 0.1 , 0.1};

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);
    EXPECT_EQ(model.getImageSize(), image_size);
}

// Test case for setPrincipalPoint with valid size inputs
TEST(KannalaTest, SetPrincipalPoint_ValidSizeInputs_SetParameters) {
    Kannala model;
    std::vector<double> principal_point = { 1.0, 1.0 };
    EXPECT_NO_THROW(model.setPrincipalPoint(principal_point));
    EXPECT_EQ(model.getPrincipalPoint(), principal_point);
}

// Test case for setPrincipalPoint with invalid size inputs
TEST(KannalaTest, SetPrincipalPoint_InvalidSizeInputs_ThrowException) {
    Kannala model;
    std::vector<double> invalid_principal_point = { 1.0 };
    EXPECT_THROW(model.setPrincipalPoint(invalid_principal_point), std::invalid_argument);
}

// Test case for setFocalLength with valid size inputs
TEST(KannalaTest, SetFocalLength_ValidSizeInputs_SetParameters) {
    Kannala model;
    std::vector<double> focal_length = { 2.0, 2.0 };
    EXPECT_NO_THROW(model.setFocalLength(focal_length));
    EXPECT_EQ(model.getFocalLength(), focal_length);
}

// Test case for setFocalLength with invalid size inputs
TEST(KannalaTest, SetFocalLength_InvalidSizeInputs_ThrowException) {
    Kannala model;
    std::vector<double> invalid_focal_length = { 2.0 };
    EXPECT_THROW(model.setFocalLength(invalid_focal_length), std::invalid_argument);
}

// Test case for setRadialDistSymCoeffs with valid size inputs
TEST(KannalaTest, SetRadialDistSymCoeffs_ValidSizeInputs_SetParameters) {
    Kannala model;
    std::vector<double> coeffs = { 0.1, 0.2 };
    EXPECT_NO_THROW(model.setRadialDistSymCoeffs(coeffs));
    EXPECT_EQ(model.getRadialDistSymCoeffs(), coeffs);
}

// Test case for setRadialDistAsymCoeffs with valid size inputs
TEST(KannalaTest, SetRadialDistAsymCoeffs_ValidSizeInputs_SetParameters) {
    Kannala model;
    std::vector<double> coeffs = { 0.1, 0.2 };
    EXPECT_NO_THROW(model.setRadialDistAsymCoeffs(coeffs));
    EXPECT_EQ(model.getRadialDistAsymCoeffs(), coeffs);
}

// Test case for setRadialDistFourCoeffs with valid size inputs
TEST(KannalaTest, SetRadialDistFourCoeffs_ValidSizeInputs_SetParameters) {
    Kannala model;
    std::vector<double> coeffs = { 0.1, 0.2 };
    EXPECT_NO_THROW(model.setRadialDistFourCoeffs(coeffs));
    EXPECT_EQ(model.getRadialDistFourCoeffs(), coeffs);
}

// Test case for setTangentialDistAsymCoeffs with valid size inputs
TEST(KannalaTest, SetTangentialDistAsymCoeffs) {
    Kannala model;
    std::vector<double> coeffs = { 0.1, 0.2 };
    EXPECT_NO_THROW(model.setTangentialDistAsymCoeffs(coeffs));
    EXPECT_EQ(model.getTangentialDistAsymCoeffs(), coeffs);
}

// Test case for setTangentialDistFourCoeffs with valid size inputs
TEST(KannalaTest, SetTangentialDistFourCoeffs) {
    Kannala model;
    std::vector<double> coeffs = { 0.1, 0.2 };
    EXPECT_NO_THROW(model.setTangentialDistFourCoeffs(coeffs));
    EXPECT_EQ(model.getTangentialDistFourCoeffs(), coeffs);
}

// Test case for setImageSize with valid size inputs
TEST(KannalaTest, SetImageSize_ValidSizeInputs_SetParameters) {
    Kannala model;
    std::vector<int> image_size = { 640, 480 };
    EXPECT_NO_THROW(model.setImageSize(image_size));
    EXPECT_EQ(model.getImageSize(), image_size);
}

// Test case for setImageSize with invalid size inputs
TEST(KannalaTest, SetImageSize_InvalidSizeInputs_ThrowException) {
    Kannala model;
    std::vector<int> invalid_image_size = { 0 };
    EXPECT_THROW(model.setImageSize(invalid_image_size), std::invalid_argument);
}

// Test case for getPinhole with valid model returning a Pinhole object with matching parameters
TEST(KannalaTest, GetPinhole_ValidModel_ReturnsPinholeWithMatchingParams) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_dist_sym = { 0.1 };
    std::vector<double> radial_distortion_asym = { 0.1, 0.1 };
    std::vector<double> radial_distortion_four = { 0.1 , 0.1};
    std::vector<double> tangential_distortion_asym = { 0.1 };
    std::vector<double> tangential_distortion_four = { 0.1 , 0.1};

    Kannala model(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);
    auto pinhole = model.getPinhole();

    EXPECT_EQ(pinhole->getFocalLength(), focal_length);
    EXPECT_EQ(pinhole->getPrincipalPoint(), principal_point);
    EXPECT_EQ(pinhole->getSkew(), 0);
    EXPECT_EQ(pinhole->getImageSize(), image_size);
}

// Test case for getModelName with valid model returning the expected name
TEST(KannalaTest, GetModelName_ValidModel_ReturnsExpectedName) {
    Kannala model;
    std::string expected_model_name = "Kannala";
    EXPECT_EQ(model.getModelName(), expected_model_name);
}

// Helper function to write JSON to a file
void writeJsonToFile3(const std::string& filePath, const nlohmann::json& j) {
    std::ofstream file(filePath);
    file << j.dump(4); // Pretty print with 4 spaces
}

// Test case for loading valid JSON of the same class resulting in a valid model
TEST(KannalaTest, Load_ValidJsonOfSameClass_ValidModel) {
    std::string filePath = "valid_model.json";
    nlohmann::json valid_json = {
        {"Intrinsics", {
            {"class_name", "KannalaModel"},
            {"mu_mv", {1009.763547265235, 1010.6711214626042}},
            {"principal_point", {1041.09728683428, 781.8203295124523}},
            {"radial_distortion_coeff", {0.092059675778893874, 0.10958710106229828, -0.051841534464012858, 0.084325680676464759}},
            {"radial_asym_poly", {-5.3981198912737127E-9, -5.5066780054808508E-9, 6.955884050415505E-9}},
            {"radial_asym_fourier", {2510.6745471606873, -52391.96568175267, -67837.372158355109, -41632.047543996}},
            {"tangential_asym_poly", {2.5626552271545331E-8, 1.0298235306784427E-7, -3.172045568820636E-8}},
            {"tangential_asym_fourier", {32403.671282353214, -24930.377251016282, 3073.9739082175288, -5882.2543479979786}},
            {"image_size", {2064, 1544}},
            {"coordinate_convention", "TL0_0"}
          }
        },
        {
          "Extrinsics", {
            {"rotation", {1.5716692559012244, -0.00052164035743438436, 1.5755205754038231}},
            {"translation", {0.018195322184966764, 0.010179320484197167, -0.032138752484333559}},
            {"class_name", "RealObject"}
          }
        }
    };

    writeJsonToFile3(filePath, valid_json);

    auto model = Kannala::load(filePath);
    EXPECT_EQ(model->getFocalLength(), std::vector<double>({ 1009.763547265235, 1010.6711214626042 }));
    EXPECT_EQ(model->getPrincipalPoint(), std::vector<double>({ 1041.09728683428, 781.8203295124523 }));
    EXPECT_EQ(model->getImageSize(), std::vector<int>({ 2064, 1544 }));
    EXPECT_EQ(model->getRadialDistSymCoeffs(), std::vector<double>({ 0.092059675778893874, 0.10958710106229828, -0.051841534464012858, 0.084325680676464759 }));
    EXPECT_EQ(model->getRadialDistAsymCoeffs(), std::vector<double>({ -5.3981198912737127E-9, -5.5066780054808508E-9, 6.955884050415505E-9 }));
    EXPECT_EQ(model->getRadialDistFourCoeffs(), std::vector<double>({ 2510.6745471606873, -52391.96568175267, -67837.372158355109, -41632.047543996 }));
    EXPECT_EQ(model->getTangentialDistAsymCoeffs(), std::vector<double>({ 2.5626552271545331E-8, 1.0298235306784427E-7, -3.172045568820636E-8 }));
    EXPECT_EQ(model->getTangentialDistFourCoeffs(), std::vector<double>({ 32403.671282353214, -24930.377251016282, 3073.9739082175288, -5882.2543479979786 }));
}

// Test case for loading invalid JSON resulting in throwing an exception
TEST(KannalaTest, Load_InvalidJson_ThrowException) {
    std::string filePath = "invalid_brownConrady.json";
    nlohmann::json invalid_json = {
        {"Intrinsics", {
            {"class_name", "KannalaModel"},
            {"mu_mv", {1009.763547265235}},
            {"principal_point", {1041.09728683428, 781.8203295124523}},
            {"radial_distortion_coeff", {0.092059675778893874, 0.10958710106229828, -0.051841534464012858, 0.084325680676464759}},
            {"radial_asym_poly", {-5.3981198912737127E-9, -5.5066780054808508E-9, 6.955884050415505E-9}},
            {"radial_asym_fourier", {2510.6745471606873, -52391.96568175267, -67837.372158355109, -41632.047543996}},
            {"tangential_asym_poly", {2.5626552271545331E-8, 1.0298235306784427E-7, -3.172045568820636E-8}},
            {"tangential_asym_fourier", {32403.671282353214, -24930.377251016282, 3073.9739082175288, -5882.2543479979786}},
            {"image_size", {2064, 1544}},
            {"coordinate_convention", "TL0_0"}
          }
        },
        {
          "Extrinsics", {
            {"rotation", {1.5716692559012244, -0.00052164035743438436, 1.5755205754038231}},
            {"translation", {0.018195322184966764, 0.010179320484197167, -0.032138752484333559}},
            {"class_name", "RealObject"}
          }
        }
    };

    writeJsonToFile3(filePath, invalid_json);

    EXPECT_THROW(Kannala::load(filePath), std::invalid_argument);
}

// Test case for loading valid JSON of a different class resulting in throwing an exception
TEST(KannalaTest, Load_ValidJsonOfDifferentClass_ThrowException) {
    std::string filePath = "different_class.json";
    nlohmann::json different_class_json = {
        {"Intrinsics", {
            {"class_name", "InvalidModel"},
            {"mu_mv", {1009.763547265235, 1010.6711214626042}},
            {"principal_point", {1041.09728683428, 781.8203295124523}},
            {"radial_distortion_coeff", {0.092059675778893874, 0.10958710106229828, -0.051841534464012858, 0.084325680676464759}},
            {"radial_asym_poly", {-5.3981198912737127E-9, -5.5066780054808508E-9, 6.955884050415505E-9}},
            {"radial_asym_fourier", {2510.6745471606873, -52391.96568175267, -67837.372158355109, -41632.047543996}},
            {"tangential_asym_poly", {2.5626552271545331E-8, 1.0298235306784427E-7, -3.172045568820636E-8}},
            {"tangential_asym_fourier", {32403.671282353214, -24930.377251016282, 3073.9739082175288, -5882.2543479979786}},
            {"image_size", {2064, 1544}},
            {"coordinate_convention", "TL0_0"}
          }
        },
        {
          "Extrinsics", {
            {"rotation", {1.5716692559012244, -0.00052164035743438436, 1.5755205754038231}},
            {"translation", {0.018195322184966764, 0.010179320484197167, -0.032138752484333559}},
            {"class_name", "RealObject"}
          }
        }
    };

    writeJsonToFile3(filePath, different_class_json);

    EXPECT_THROW(Kannala::load(filePath), std::invalid_argument); // Assuming load throws an exception on JSON of a different class
}

// Test case for getBackprojectSettings with default values
TEST(KannalaTest, getBackprojectSettings_default_returnsexpected) {
    Kannala model;
    double expected_threshold = 1e-6;
    int expected_iterations = 20;

    double threshold;
    int iterations;
    model.getBackprojectSettings(threshold, iterations);

    EXPECT_DOUBLE_EQ(threshold, expected_threshold);
    EXPECT_EQ(iterations, expected_iterations);
}

// Test case for setBackprojectSettings and then getBackprojectSettings
TEST(KannalaTest, setBackprojectSettings_afterget_returnsexpected) {
    Kannala model;
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
TEST(KannalaTest, setBackprojectSettings_invalidinputs_throwsexception) {
    Kannala model;

    double invalid_threshold = -0.5;
    int invalid_iterations = -10;

    EXPECT_THROW(model.setBackprojectSettings(invalid_threshold, invalid_iterations), std::invalid_argument);
}