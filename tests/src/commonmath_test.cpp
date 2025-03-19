#include "pixeltraq.h"
#include <gtest/gtest.h>
#include <cmath>

// Define M_PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Test evaluatePolynomial with normal inputs
TEST(CommonMathTest, EvaluatePolynomial_NormalInputs_ReturnExpected) {
    std::vector<double> coeffs = { 1, 0, -2 }; // Represents 1 - 2 * x^2
    double x = 3.0;
    EXPECT_NEAR(CommonMath::evaluatePolynomial(coeffs, x), -17.0, 1e-6);
}

// Test evaluatePolynomial with zero inputs
TEST(CommonMathTest, EvaluatePolynomial_ZeroInputs_ReturnExpected) {
    std::vector<double> coeffs = { 0, 0, 0 }; // Represents 0
    double x = 3.0;
    EXPECT_DOUBLE_EQ(CommonMath::evaluatePolynomial(coeffs, x), 0.0);
}

// Test evaluatePolynomial with NaN inputs
TEST(CommonMathTest, EvaluatePolynomial_NaNInputs_ReturnNaN) {
    std::vector<double> coeffs = { NAN, NAN, NAN };
    double x = 3.0;
    EXPECT_TRUE(std::isnan(CommonMath::evaluatePolynomial(coeffs, x)));
}

// Test isZero with a zero vector
TEST(CommonMathTest, IsZero_ZeroVector_ReturnTrue) {
    std::vector<double> v = { 0.0, 0.0, 0.0 };
    EXPECT_TRUE(CommonMath::isZero(v));
}

// Test isZero with a non-zero vector
TEST(CommonMathTest, IsZero_NonZeroVector_ReturnFalse) {
    std::vector<double> v = { 0.0, 0.0, 1.0 };
    EXPECT_FALSE(CommonMath::isZero(v));
}

// Test evaluateFourier with normal inputs
TEST(CommonMathTest, EvaluateFourier_NormalInputs_ReturnExpected) {
    std::vector<double> coeffs = { 1.0, 2.0};
    
    double phi = M_PI / 4.0; // 45 degrees
    double expected = 1.0 * cos(phi) + 2.0 * sin(phi);
    EXPECT_NEAR(CommonMath::evaluateFourier(coeffs, phi), expected, 1e-6);

    coeffs = { 1.0, 2.0, 3.0, 4.0};
    phi = M_PI / 4.0; // 45 degrees
    expected = 1.0 * cos(phi) + 2.0 * sin(phi) +
        3.0 * cos(2 * phi) + 4.0 * sin(2 * phi);
    EXPECT_NEAR(CommonMath::evaluateFourier(coeffs, phi), expected, 1e-6);
    
    coeffs = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    phi = M_PI / 4.0; // 45 degrees
    expected = 1.0 * cos(phi) + 2.0 * sin(phi) +
        3.0 * cos(2 * phi) + 4.0 * sin(2 * phi) +
        5.0 * cos(3 * phi) + 6.0 * sin(3 * phi);
    EXPECT_NEAR(CommonMath::evaluateFourier(coeffs, phi), expected, 1e-6);

    coeffs = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0 };
    phi = M_PI / 4.0; // 45 degrees
    expected = 1.0 * cos(phi) + 2.0 * sin(phi) +
        3.0 * cos(2 * phi) + 4.0 * sin(2 * phi) +
        5.0 * cos(3 * phi) + 6.0 * sin(3 * phi) +
        7.0 * cos(4 * phi) + 8.0 * sin(4 * phi);
    EXPECT_NEAR(CommonMath::evaluateFourier(coeffs, phi), expected, 1e-6);
}

// Test evaluateFourier with zero inputs
TEST(CommonMathTest, EvaluateFourier_ZeroInputs_ReturnExpected) {
    std::vector<double> coeffs = { 0.0, 0.0, 0.0, 0.0 };
    double phi = M_PI / 4.0; // 45 degrees
    EXPECT_DOUBLE_EQ(CommonMath::evaluateFourier(coeffs, phi), 0.0);
}

// Test evaluateFourier with NaN inputs
TEST(CommonMathTest, EvaluateFourier_NaNInputs_ReturnNaN) {
    std::vector<double> coeffs = { std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN() };
    double phi = M_PI / 4.0; // 45 degrees
    EXPECT_TRUE(std::isnan(CommonMath::evaluateFourier(coeffs, phi)));
}

// Test interp2 (overload 1) with normal inputs
TEST(CommonMathTest, Interp2Overload1_NormalInputs_ReturnExpected) {
    std::vector<std::vector<double>> img = { {1, 2}, {3, 4} };
    std::vector<std::vector<double>> Xd = { {0, 1}, {0, 1} };
    std::vector<std::vector<double>> Yd = { {0, 0}, {1, 1} };
    std::vector<std::vector<double>> expected = { {1, 2}, {3, 4} };

    EXPECT_EQ(CommonMath::interp2(img, Xd, Yd), expected);

    img = { {1, 2, 3}, {4, 5, 6}, {7, 8, 9} };
    Xd = { {0, 1}, {0, 1} };
    Yd = { {0, 0}, {1, 1} };
    expected = { {1, 2}, {4, 5} };

    EXPECT_EQ(CommonMath::interp2(img, Xd, Yd), expected);
}

// Test interp2 (overload 1) with zero inputs
TEST(CommonMathTest, Interp2Overload1_ZeroInputs_ReturnExpected) {
    std::vector<std::vector<double>> img = { {0, 0}, {0, 0} };
    std::vector<std::vector<double>> Xd = { {0, 1}, {0, 1} };
    std::vector<std::vector<double>> Yd = { {0, 0}, {1, 1} };
    std::vector<std::vector<double>> expected = { {0, 0}, {0, 0} };

    EXPECT_EQ(CommonMath::interp2(img, Xd, Yd), expected);
}

// Test interp2 (overload 1) with empty inputs
TEST(CommonMathTest, Interp2Overload1_EmptyInputs_ReturnEmpty) {
    std::vector<std::vector<double>> img = {};
    std::vector<std::vector<double>> Xd = {};
    std::vector<std::vector<double>> Yd = {};

    EXPECT_EQ(CommonMath::interp2(img, Xd, Yd), img);
}

// Test interp2 (overload 2) with normal inputs
TEST(CommonMathTest, Interp2Overload2_NormalInputs_ReturnExpected) {
    std::vector<std::vector<std::vector<double>>> img = { {{1, 2}, {3, 4}}, {{5, 6}, {7, 8}} };
    std::vector<std::vector<double>> Xd = { {0, 1}, {0, 1} };
    std::vector<std::vector<double>> Yd = { {0, 0}, {1, 1} };
    std::vector<std::vector<std::vector<double>>> expected = { {{1, 2}, {3, 4}}, {{5, 6}, {7, 8}} };

    EXPECT_EQ(CommonMath::interp2(img, Xd, Yd), expected);
}

// Test interp2 (overload 2) with zero inputs
TEST(CommonMathTest, Interp2Overload2_ZeroInputs_ReturnExpected) {
    std::vector<std::vector<std::vector<double>>> img = { {{0, 0}, {0, 0}}, {{0, 0}, {0, 0}} };
    std::vector<std::vector<double>> Xd = { {0, 1}, {0, 1} };
    std::vector<std::vector<double>> Yd = { {0, 0}, {1, 1} };
    std::vector<std::vector<std::vector<double>>> expected = { {{0, 0}, {0, 0}}, {{0, 0}, {0, 0}} };

    EXPECT_EQ(CommonMath::interp2(img, Xd, Yd), expected);
}

// Test interp2 (overload 2) with empty inputs
TEST(CommonMathTest, Interp2Overload2_EmptyInputs_ReturnEmpty) {
    std::vector<std::vector<std::vector<double>>> img = {};
    std::vector<std::vector<double>> Xd = {};
    std::vector<std::vector<double>> Yd = {};

    EXPECT_EQ(CommonMath::interp2(img, Xd, Yd), img);
}

// Test bilinearInterpolate with normal inputs
TEST(CommonMathTest, BilinearInterpolate_NormalInputs_ReturnExpected) {
    std::vector<std::vector<double>> img = { {1, 2}, {3, 4} };
    double x = 0.5;
    double y = 0.5;
    double expected = 2.5; // Interpolated value at (0.5, 0.5)
    EXPECT_DOUBLE_EQ(CommonMath::bilinearInterpolate(img, x, y), expected);
}

// Test bilinearInterpolate with zero inputs
TEST(CommonMathTest, BilinearInterpolate_ZeroInputs_ReturnExpected) {
    std::vector<std::vector<double>> img = { {0, 0}, {0, 0} };
    double x = 0.5;
    double y = 0.5;
    double expected = 0.0; // Interpolated value at (0.5, 0.5) should be 0
    EXPECT_DOUBLE_EQ(CommonMath::bilinearInterpolate(img, x, y), expected);
}

// Test bilinearInterpolate with empty inputs
TEST(CommonMathTest, BilinearInterpolate_EmptyInputs_ReturnEmpty) {
    std::vector<std::vector<double>> img = {};
    double x = 0.5;
    double y = 0.5;
    double expected = 0.0; // Interpolated value with empty image should be 0
    EXPECT_DOUBLE_EQ(CommonMath::bilinearInterpolate(img, x, y), expected);
}

// Test rotatePoint with normal inputs
TEST(CommonMathTest, RotatePoint_NormalInputs_ReturnExpected) {
    Point3 point = { 1.0, 2.0, 3.0 };
    Matrix3x3 rotation_matrix = {{
        {0, -1, 0},
        {1, 0, 0},
        {0, 0, 1}
    }};
    Point3 expected = { -2.0, 1.0, 3.0 };
    EXPECT_EQ(CommonMath::rotatePoint(point, rotation_matrix), expected);
}

// Test rotatePoint with zero inputs
TEST(CommonMathTest, RotatePoint_ZeroInputs_ReturnExpected) {
    Point3 point = { 0.0, 0.0, 0.0 };
    Matrix3x3 rotation_matrix = { {
        {0, -1, 0},
        {1, 0, 0},
        {0, 0, 1}
    } };
    Point3 expected = { 0.0, 0.0, 0.0 };
    EXPECT_EQ(CommonMath::rotatePoint(point, rotation_matrix), expected);
}

// Test rotatePoints with normal inputs
TEST(CommonMathTest, RotatePoints_NormalInputs_ReturnExpected) {
    std::vector<Point3> points = {
        {1.0, 2.0, 3.0},
        {4.0, 5.0, 6.0}
    };
    Matrix3x3 rotation_matrix = { {
        {0, -1, 0},
        {1, 0, 0},
        {0, 0, 1}
    } };
    std::vector<Point3> expected = {
        {-2.0, 1.0, 3.0},
        {-5.0, 4.0, 6.0}
    };
    EXPECT_EQ(CommonMath::rotatePoints(points, rotation_matrix), expected);
}

// Test rotatePoints with zero inputs
TEST(CommonMathTest, RotatePoints_ZeroInputs_ReturnExpected) {
    std::vector<Point3> points = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };
    Matrix3x3 rotation_matrix = { {
        {0, -1, 0},
        {1, 0, 0},
        {0, 0, 1}
    } };
    std::vector<Point3> expected = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };
    EXPECT_EQ(CommonMath::rotatePoints(points, rotation_matrix), expected);
}

// Test transformPoint with normal inputs
TEST(CommonMathTest, TransformPoint_NormalInputs_ReturnExpected) {
    Point3 point = { 1.0, 2.0, 3.0 };

    Matrix3x3 rotation_matrix = { {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    } };

    Point3 translation = { 1.0, 1.0, 1.0 };

    Point3 expected = { 2.0, 3.0, 4.0 };

    Point3 result = CommonMath::transformPoint(point, rotation_matrix, translation);

    rotation_matrix = { {
        {0, -1, 0},
        {1, 0, 0},
        {0, 0, 1}
    } };
    expected = { -1.0, 2.0, 4.0 };

    result = CommonMath::transformPoint(point, rotation_matrix, translation);
    EXPECT_EQ(result, expected);
}

// Test transformPoint with zero inputs
TEST(CommonMathTest, TransformPoint_ZeroInputs_ReturnExpected) {
    Point3 point = { 0.0, 0.0, 0.0 };

    Matrix3x3 rotation_matrix = { {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    } };

    Point3 translation = { 0.0, 0.0, 0.0 };

    Point3 expected = { 0.0, 0.0, 0.0 };

    Point3 result = CommonMath::transformPoint(point, rotation_matrix, translation);
    EXPECT_EQ(result, expected);
}

// Test transformPoints with normal inputs
TEST(CommonMathTest, TransformPoints_NormalInputs_ReturnExpected) {
    std::vector<Point3> points = {
        {1.0, 2.0, 3.0},
        {4.0, 5.0, 6.0},
        {7.0, 8.0, 9.0}
    };

    Matrix3x3 rotation_matrix = { {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    } };

    Point3 translation = { 1.0, 1.0, 1.0 };

    std::vector<Point3> expected = {
        {2.0, 3.0, 4.0},
        {5.0, 6.0, 7.0},
        {8.0, 9.0, 10.0}
    };

    std::vector<Point3> result = CommonMath::transformPoints(points, rotation_matrix, translation);
    EXPECT_EQ(result, expected);
}

// Test transformPoints with zero inputs
TEST(CommonMathTest, TransformPoints_ZeroInputs_ReturnExpected) {
    std::vector<Point3> points = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    Matrix3x3 rotation_matrix = { {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    } };

    Point3 translation = { 0.0, 0.0, 0.0 };

    std::vector<Point3> expected = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    std::vector<Point3> result = CommonMath::transformPoints(points, rotation_matrix, translation);
    EXPECT_EQ(result, expected);
}

// Test for rotationInverse with normal inputs
TEST(CommonMathTest, rotationInverse_NormalInputs_ReturnExpected) {
    Matrix3x3 rotation_matrix = { {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    } };
    Matrix3x3 expected_inverse = { {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    } };
    Matrix3x3 result = CommonMath::rotationInverse(rotation_matrix);
    EXPECT_EQ(result, expected_inverse);

    rotation_matrix = { {
        {0, -1, 0},
        {1, 0, 0},
        {0, 0, 1}
    } };
    expected_inverse = { {
        {0, 1, 0},
        {-1, 0, 0},
        {0, 0, 1}
    } };
    result = CommonMath::rotationInverse(rotation_matrix);
    EXPECT_EQ(result, expected_inverse);
}

// Test for transposeMatrix with normal inputs
TEST(CommonMathTest, transposeMatrix_NormalInputs_ReturnExpected) {
    Matrix3x3 matrix = { {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    } };
    Matrix3x3 expected_transpose = { {
        {1, 4, 7},
        {2, 5, 8},
        {3, 6, 9}
    } };
    Matrix3x3 result = CommonMath::transposeMatrix(matrix);
    EXPECT_EQ(result, expected_transpose);
}