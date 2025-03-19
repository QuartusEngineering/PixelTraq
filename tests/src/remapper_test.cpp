#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include "pixeltraq.h"

// Helper function to create a dummy image
std::vector<std::vector<std::vector<double>>> createDummyImage(int width, int height, int channels) {
    std::vector<std::vector<std::vector<double>>> image(channels, std::vector<std::vector<double>>(height, std::vector<double>(width, 0)));
    for (int c = 0; c < channels; ++c) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                image[c][y][x] = static_cast<double>(x + y + c);
            }
        }
    }
    return image;
}

std::vector<std::vector<std::vector<double>>> createZeroValuedImage(int width, int height, int channels) {
    return std::vector<std::vector<std::vector<double>>>(channels, std::vector<std::vector<double>>(height, std::vector<double>(width, 0.0)));
}

std::vector<std::vector<std::vector<double>>> createNaNValuedImage(int width, int height, int channels) {
    return std::vector<std::vector<std::vector<double>>>(channels, std::vector<std::vector<double>>(height, std::vector<double>(width, std::numeric_limits<double>::quiet_NaN())));
}

// Test cases
TEST(RemapperTest, singleargconstructor_validinputs_validresult) {
    auto cam_source_1 = std::make_shared<Pinhole>();
    EXPECT_NO_THROW(Remapper remapper(cam_source_1));

    auto cam_source_2 = std::make_shared<BrownConrady>();
    EXPECT_NO_THROW(Remapper remapper(cam_source_2));

    auto cam_source_3 = std::make_shared<Kannala>();
    EXPECT_NO_THROW(Remapper remapper(cam_source_3));

    auto cam_source_4 = std::make_shared<GenFTanTheta>();
    EXPECT_NO_THROW(Remapper remapper(cam_source_4));

    auto cam_source_5 = std::make_shared<GenFTheta>();
    EXPECT_NO_THROW(Remapper remapper(cam_source_5));
}

TEST(RemapperTest, twoargconstructor_validinputs_validresult) {
    auto cam_source_1 = std::make_shared<Pinhole>();
    auto cam_target_1 = std::make_shared<Pinhole>();
    EXPECT_NO_THROW(Remapper remapper(cam_source_1, cam_target_1));

    auto cam_source_2 = std::make_shared<BrownConrady>();
    auto cam_target_2 = std::make_shared<BrownConrady>();
    EXPECT_NO_THROW(Remapper remapper(cam_source_2, cam_target_2));

    auto cam_source_3 = std::make_shared<Kannala>();
    auto cam_target_3 = std::make_shared<Kannala>();
    EXPECT_NO_THROW(Remapper remapper(cam_source_3, cam_target_3));

    auto cam_source_4 = std::make_shared<GenFTanTheta>();
    auto cam_target_4 = std::make_shared<GenFTanTheta>();
    EXPECT_NO_THROW(Remapper remapper(cam_source_4, cam_target_4));

    auto cam_source_5 = std::make_shared<GenFTheta>();
    auto cam_target_5 = std::make_shared<GenFTheta>();
    EXPECT_NO_THROW(Remapper remapper(cam_source_5, cam_target_5));
}

TEST(RemapperTest, threeargconstructor_validinputs_validresult) {
    auto cam_source = std::make_shared<Pinhole>();
    auto cam_target = std::make_shared<Pinhole>();
    Matrix3x3 rotation_matrix = { { {1.0, 0, 0}, {0, 1.0, 0}, {0, 0, 1.0} } };
    EXPECT_NO_THROW(Remapper remapper(cam_source, cam_target, rotation_matrix));
}

TEST(RemapperTest, distort_validimage_validresult) {
    std::vector<double> focal_length_source = { 2.0, 2.0 };
    std::vector<double> principal_point_source = { 1.0, 1.0 };
    double skew_source = 0.5;
    std::vector<int> image_size_source = { 640, 480 };
    auto cam_source = std::make_shared<Pinhole>(focal_length_source, principal_point_source, skew_source, image_size_source);
    std::vector<double> focal_length_target = { 2.0, 2.0 };
    std::vector<double> principal_point_target = { 1.0, 1.0 };
    double skew_target = 0.5;
    std::vector<int> image_size_target = { 800, 600 };
    auto cam_target = std::make_shared<Pinhole>(focal_length_target, principal_point_target, skew_target, image_size_target);
    Remapper remapper(cam_source, cam_target);

    // Create a dummy image
    auto image = createDummyImage(800,600,3);

    // Distort the image
    auto distorted_image = remapper.distort(image);

    // Validate the result (using some example checks, adjust as needed)
    EXPECT_EQ(distorted_image.size(), 3);
    EXPECT_EQ(distorted_image[0].size(), 480);
    EXPECT_EQ(distorted_image[0][0].size(), 640);
}

TEST(RemapperTest, distort_emptyimage_emptyresult) {
    std::vector<double> focal_length_source = { 2.0, 2.0 };
    std::vector<double> principal_point_source = { 1.0, 1.0 };
    double skew_source = 0.5;
    std::vector<int> image_size_source = { 640, 480 };
    auto cam_source = std::make_shared<Pinhole>(focal_length_source, principal_point_source, skew_source, image_size_source);
    std::vector<double> focal_length_target = { 2.0, 2.0 };
    std::vector<double> principal_point_target = { 1.0, 1.0 };
    double skew_target = 0.5;
    std::vector<int> image_size_target = { 800, 600 };
    auto cam_target = std::make_shared<Pinhole>(focal_length_target, principal_point_target, skew_target, image_size_target);
    Remapper remapper(cam_source, cam_target);

    // Create an empty image
    std::vector<std::vector<std::vector<double>>> empty_image = {};

    // Distort the image
    auto distorted_image = remapper.distort(empty_image);

    // Validate the result
    EXPECT_TRUE(distorted_image.empty());
}

TEST(RemapperTest, distort_zerovaluedimage_validresult) {
    std::vector<double> focal_length_source = { 2.0, 2.0 };
    std::vector<double> principal_point_source = { 1.0, 1.0 };
    double skew_source = 0.5;
    std::vector<int> image_size_source = { 640, 480 };
    auto cam_source = std::make_shared<Pinhole>(focal_length_source, principal_point_source, skew_source, image_size_source);
    std::vector<double> focal_length_target = { 2.0, 2.0 };
    std::vector<double> principal_point_target = { 1.0, 1.0 };
    double skew_target = 0.5;
    std::vector<int> image_size_target = { 800, 600 };
    auto cam_target = std::make_shared<Pinhole>(focal_length_target, principal_point_target, skew_target, image_size_target);
    Remapper remapper(cam_source, cam_target);

    // Create a zero-valued image
    auto zero_valued_image = createZeroValuedImage(800, 600, 3);

    // Distort the image
    auto distorted_image = remapper.distort(zero_valued_image);

    // Validate the result (using some example checks, adjust as needed)
    EXPECT_EQ(distorted_image.size(), 3);
    EXPECT_EQ(distorted_image[0].size(), 480);
    EXPECT_EQ(distorted_image[0][0].size(), 640);

    // Check that the distorted image still contains zero values
    for (const auto& channel : distorted_image) {
        for (const auto& column : channel) {
            for (const auto& pixel : column) {
                EXPECT_EQ(pixel, 0.0);
            }
        }
    }
}

TEST(RemapperTest, undistort_validimage_validresult) {
    std::vector<double> focal_length_source = { 2.0, 2.0 };
    std::vector<double> principal_point_source = { 1.0, 1.0 };
    double skew_source = 0.5;
    std::vector<int> image_size_source = { 640, 480 };
    auto cam_source = std::make_shared<Pinhole>(focal_length_source, principal_point_source, skew_source, image_size_source);
    std::vector<double> focal_length_target = { 2.0, 2.0 };
    std::vector<double> principal_point_target = { 1.0, 1.0 };
    double skew_target = 0.5;
    std::vector<int> image_size_target = { 800, 600 };
    auto cam_target = std::make_shared<Pinhole>(focal_length_target, principal_point_target, skew_target, image_size_target);
    Remapper remapper(cam_source, cam_target);

    // Create a dummy image
    auto image = createDummyImage(640, 480, 3);

    // Undistort the image
    auto undistorted_image = remapper.undistort(image);

    // Validate the result (using some example checks, adjust as needed)
    EXPECT_EQ(undistorted_image.size(), 3);
    EXPECT_EQ(undistorted_image[0].size(), 600);
    EXPECT_EQ(undistorted_image[0][0].size(), 800);
    EXPECT_EQ(undistorted_image[0][0][0], image[0][0][0]);  // Example check
}

TEST(RemapperTest, undistort_emptyimage_emptyresult) {
    std::vector<double> focal_length_source = { 2.0, 2.0 };
    std::vector<double> principal_point_source = { 1.0, 1.0 };
    double skew_source = 0.5;
    std::vector<int> image_size_source = { 640, 480 };
    auto cam_source = std::make_shared<Pinhole>(focal_length_source, principal_point_source, skew_source, image_size_source);
    std::vector<double> focal_length_target = { 2.0, 2.0 };
    std::vector<double> principal_point_target = { 1.0, 1.0 };
    double skew_target = 0.5;
    std::vector<int> image_size_target = { 800, 600 };
    auto cam_target = std::make_shared<Pinhole>(focal_length_target, principal_point_target, skew_target, image_size_target);
    Remapper remapper(cam_source, cam_target);

    // Create an empty image
    std::vector<std::vector<std::vector<double>>> empty_image = {};

    // Undistort the image
    auto undistorted_image = remapper.undistort(empty_image);

    // Validate the result
    EXPECT_TRUE(undistorted_image.empty());
}

TEST(RemapperTest, undistort_zerovaluedimage_validresult) {
    std::vector<double> focal_length_source = { 2.0, 2.0 };
    std::vector<double> principal_point_source = { 1.0, 1.0 };
    double skew_source = 0.5;
    std::vector<int> image_size_source = { 640, 480 };
    auto cam_source = std::make_shared<Pinhole>(focal_length_source, principal_point_source, skew_source, image_size_source);
    std::vector<double> focal_length_target = { 2.0, 2.0 };
    std::vector<double> principal_point_target = { 1.0, 1.0 };
    double skew_target = 0.5;
    std::vector<int> image_size_target = { 800, 600 };
    auto cam_target = std::make_shared<Pinhole>(focal_length_target, principal_point_target, skew_target, image_size_target);
    Remapper remapper(cam_source, cam_target);

    // Create a zero-valued image
    auto zero_valued_image = createZeroValuedImage(640, 480, 3);

    // Undistort the image
    auto undistorted_image = remapper.undistort(zero_valued_image);

    // Validate the result (using some example checks, adjust as needed)
    EXPECT_EQ(undistorted_image.size(), 3);
    EXPECT_EQ(undistorted_image[0].size(), 600);
    EXPECT_EQ(undistorted_image[0][0].size(), 800);

    // Check that the distorted image still contains zero values
    for (const auto& channel : undistorted_image) {
        for (const auto& column : channel) {
            for (const auto& pixel : column) {
                EXPECT_EQ(pixel, 0.0);
            }
        }
    }
}

TEST(RemapperTest, DistortUndistortRoundtripReturnsOriginal) {

    std::vector<double> focal_length_source = { 600.0, 600.0 };
    std::vector<double> principal_point_source = { 400.0, 300.0 };
    double skew_source = 0.0;
    std::vector<int> image_size_source = { 800, 600 };
    auto cam_source = std::make_shared<Pinhole>(focal_length_source, principal_point_source, skew_source, image_size_source);
    std::vector<double> focal_length_target = { 600.0, 600.0 };
    std::vector<double> principal_point_target = { 400.0, 300.0 };
    double skew_target = 0.0;
    std::vector<int> image_size_target = { 800, 600 };
    auto cam_target = std::make_shared<Pinhole>(focal_length_target, principal_point_target, skew_target, image_size_target);
    Remapper remapper(cam_source, cam_target);

    // Create a dummy image
    auto original_image = createDummyImage(800, 600, 3);
    //Utils::saveAsBMP(original_image, "original.bmp");

    // Undistort and then distort the image
    auto distorted_image = remapper.distort(original_image);
    //Utils::saveAsBMP(undistorted_image, "distorted.bmp");
    auto roundtrip_image = remapper.undistort(distorted_image);
    //Utils::saveAsBMP(roundtrip_image, "roundtrip.bmp");

    // Check if the round-trip image is close to the original
    int offset = 1; //ignore edge effects
    for (size_t i = 0; i < original_image.size(); ++i) {
        for (size_t j = offset; j < original_image[0].size() - offset; ++j) {
            for (size_t k = offset; k < original_image[0][0].size() - offset; ++k) {
                EXPECT_NEAR(original_image[i][j][k], roundtrip_image[i][j][k], 1e-3) << "Value mismatch at (" << i << ", " << j << ", " << k << ")";
            }
        }
    }

    std::vector<double> focal_length = { 600.0, 600.0 };
    std::vector<double> principal_point = { 320, 240 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_distortion = { 0.1 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0 };
    auto cam_source_2 = std::make_shared<BrownConrady>(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);
    Remapper remapper2(cam_source_2, cam_target);

    // Create a dummy image
    original_image = createDummyImage(800, 600, 3);
    //Utils::saveAsBMP(original_image, "original.bmp");

    // Undistort and then distort the image
    distorted_image = remapper2.distort(original_image);
    //Utils::saveAsBMP(distorted_image, "distorted.bmp");
    roundtrip_image = remapper2.undistort(distorted_image);
    //Utils::saveAsBMP(roundtrip_image, "roundtrip.bmp");

    // Check if the round-trip image is close to the original
    offset = 100; //ignore edge effects
    for (size_t i = 0; i < original_image.size(); ++i) {
        for (size_t j = offset; j < original_image[0].size() - offset; ++j) {
            for (size_t k = offset; k < original_image[0][0].size() - offset; ++k) {
                EXPECT_NEAR(original_image[i][j][k], roundtrip_image[i][j][k], 1e-3) << "Value mismatch at (" << i << ", " << j << ", " << k << ")";
            }
        }
    }

    std::vector<double> radial_dist_sym = { 0 };
    std::vector<double> radial_distortion_asym = { 0, 0 };
    std::vector<double> radial_distortion_four = { 0, 0 };
    std::vector<double> tangential_distortion_asym = { 0 };
    std::vector<double> tangential_distortion_four = { 0, 0 };

    auto cam_source_3 = std::make_shared<Kannala>(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);
    Remapper remapper3(cam_source_3, cam_target);

    // Create a dummy image
    original_image = createDummyImage(800, 600, 3);
    //Utils::saveAsBMP(original_image, "original.bmp");

    // Undistort and then distort the image
    distorted_image = remapper3.distort(original_image);
    //Utils::saveAsBMP(distorted_image, "distorted.bmp");
    roundtrip_image = remapper3.undistort(distorted_image);
    //Utils::saveAsBMP(roundtrip_image, "roundtrip.bmp");

    // Check if the round-trip image is close to the original
    offset = 60; //ignore edge effects
    for (size_t i = 0; i < original_image.size(); ++i) {
        for (size_t j = offset; j < original_image[0].size() - offset; ++j) {
            for (size_t k = offset; k < original_image[0][0].size() - offset; ++k) {
                EXPECT_NEAR(original_image[i][j][k], roundtrip_image[i][j][k], 1e-3) << "Value mismatch at (" << i << ", " << j << ", " << k << ")";
            }
        }
    }
}

TEST(RemapperTest, UndistortDistortRoundtripReturnsOriginal) {

    std::vector<double> focal_length_source = { 600.0, 600.0 };
    std::vector<double> principal_point_source = { 400.0, 300.0 };
    double skew_source = 0.0;
    std::vector<int> image_size_source = { 800, 600 };
    auto cam_source = std::make_shared<Pinhole>(focal_length_source, principal_point_source, skew_source, image_size_source);
    std::vector<double> focal_length_target = { 600.0, 600.0 };
    std::vector<double> principal_point_target = { 400.0, 300.0 };
    double skew_target = 0.0;
    std::vector<int> image_size_target = { 800, 600 };
    auto cam_target = std::make_shared<Pinhole>(focal_length_target, principal_point_target, skew_target, image_size_target);
    Remapper remapper(cam_source, cam_target);

    // Create a dummy image
    auto original_image = createDummyImage(800, 600, 3);
    //Utils::saveAsBMP(original_image, "original.bmp");

    // Undistort and then distort the image
    auto undistorted_image = remapper.undistort(original_image);
    //Utils::saveAsBMP(undistorted_image, "undistorted.bmp");
    auto roundtrip_image = remapper.distort(undistorted_image);
    //Utils::saveAsBMP(roundtrip_image, "roundtrip.bmp");

    // Check if the round-trip image is close to the original
    int offset = 1; //ignore edge effects
    for (size_t i = 0; i < original_image.size(); ++i) {
        for (size_t j = offset; j < original_image[0].size() - offset; ++j) {
            for (size_t k = offset; k < original_image[0][0].size()- offset; ++k) {
                EXPECT_NEAR(original_image[i][j][k], roundtrip_image[i][j][k], 1e-3) << "Value mismatch at (" << i << ", " << j << ", " << k << ")";
            }
        }
    }

    std::vector<double> focal_length = { 600.0, 600.0 };
    std::vector<double> principal_point = { 320, 240 };
    std::vector<int> image_size = { 640, 480 };
    std::vector<double> radial_distortion = { 0.1 };
    std::vector<double> tangential_distortion = { 0, 0 };
    std::vector<double> tangential_distortion_polycoeff = { 0 };
    auto cam_source_2 = std::make_shared<BrownConrady>(focal_length, principal_point, image_size, radial_distortion, tangential_distortion, tangential_distortion_polycoeff);
    Remapper remapper2(cam_source_2, cam_target);

    // Create a dummy image
    original_image = createDummyImage(640, 480, 3);
    //Utils::saveAsBMP(original_image, "original.bmp");

    // Undistort and then distort the image
    undistorted_image = remapper2.undistort(original_image);
    //Utils::saveAsBMP(undistorted_image, "undistorted.bmp");
    roundtrip_image = remapper2.distort(undistorted_image);
    //Utils::saveAsBMP(roundtrip_image, "roundtrip.bmp");

    // Check if the round-trip image is close to the original
    offset = 2; //ignore edge effects
    for (size_t i = 0; i < original_image.size(); ++i) {
        for (size_t j = offset; j < original_image[0].size() - offset; ++j) {
            for (size_t k = offset; k < original_image[0][0].size() - offset; ++k) {
                EXPECT_NEAR(original_image[i][j][k], roundtrip_image[i][j][k], 1e-3) << "Value mismatch at (" << i << ", " << j << ", " << k << ")";
            }
        }
    }

    std::vector<double> radial_dist_sym = { 0 };
    std::vector<double> radial_distortion_asym = { 0, 0 };
    std::vector<double> radial_distortion_four = { 0, 0 };
    std::vector<double> tangential_distortion_asym = { 0 };
    std::vector<double> tangential_distortion_four = { 0, 0 };

    auto cam_source_3 = std::make_shared<Kannala>(focal_length, principal_point, image_size, radial_dist_sym, radial_distortion_asym, radial_distortion_four, tangential_distortion_asym, tangential_distortion_four);
    Remapper remapper3(cam_source_3, cam_target);

    // Create a dummy image
    original_image = createDummyImage(640, 480, 3);
    //Utils::saveAsBMP(original_image, "original.bmp");

    // Undistort and then distort the image
    undistorted_image = remapper3.undistort(original_image);
    //Utils::saveAsBMP(undistorted_image, "undistorted.bmp");
    roundtrip_image = remapper3.distort(undistorted_image);
    //Utils::saveAsBMP(roundtrip_image, "roundtrip.bmp");

    // Check if the round-trip image is close to the original
    offset = 2; //ignore edge effects
    for (size_t i = 0; i < original_image.size(); ++i) {
        for (size_t j = offset; j < original_image[0].size() - offset; ++j) {
            for (size_t k = offset; k < original_image[0][0].size() - offset; ++k) {
                EXPECT_NEAR(original_image[i][j][k], roundtrip_image[i][j][k], 1e-3) << "Value mismatch at (" << i << ", " << j << ", " << k << ")";
            }
        }
    }
}