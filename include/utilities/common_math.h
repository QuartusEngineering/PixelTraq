#ifndef COMMONMATH_H
#define COMMONMATH_H

#include <cmath>
#include <vector>
#include <iostream>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <algorithm>

class Camera; // Forward declaration
class Pinhole; // Forward declaration

const double EPS = std::numeric_limits<double>::epsilon();
const double DNAN = std::numeric_limits<double>::quiet_NaN();

using Matrix3x3 = std::array<std::array<double, 3>, 3>;
using Point3 = std::array<double, 3>;
using Point2 = std::array<double, 2>;

class CommonMath {
public:
    // camera model specific math
	static double evaluatePolynomial(const std::vector<double>& coeffs, double x);
	static bool isZero(const std::vector<double>& v);
    static double evaluateFourier(const std::vector<double>& fourier_coeff,double phi);

    // extrinsics math
    static std::vector<Point3> transformPoints(const std::vector<Point3>& points, const Matrix3x3& rotation_matrix, const Point3& translation);
    static Point3 transformPoint(const Point3& point, const Matrix3x3& rotation_matrix, const Point3& translation);
    static Point3 rotatePoint(const Point3& point, const Matrix3x3& rotation_matrix);
    static std::vector<Point3> rotatePoints(const std::vector<Point3>& points, const Matrix3x3& rotation_matrix);
    static Matrix3x3 rotationInverse(const Matrix3x3& rotation_matrix);
    static std::vector<std::vector<double>> transposeMatrix(const std::vector<std::vector<double>>& matrix);

    // specialized operations
    static std::vector<std::array<double, 3>> intersectRays(std::shared_ptr<Camera> cameraL, std::shared_ptr<Camera> cameraR, const std::vector<Point3>& raysL, const std::vector<Point3>& raysR);
    static int lineLineIntersect(Point3 p1, Point3 p2, Point3 p3, Point3 p4, Point3& pa, Point3& pb, double& mua, double& mub);
    static void stereoRectify(std::shared_ptr<Pinhole> cameraL, std::shared_ptr<Pinhole> cameraR, Matrix3x3& Rl, Matrix3x3& Rr);

    // 3d vector math
    static double norm(const Point3& vec);
    static Point3 cross(const Point3& lhs, const Point3& rhs);
    static Matrix3x3 matrixLog3(const Matrix3x3& R);
    static Matrix3x3 matrixExp3(const Matrix3x3& so3mat);
    static Matrix3x3 vecToso3(const Point3& omg);
    static Point3 so3ToVec(const Matrix3x3& so3mat);
    static void axisAng3(const Point3& omgtheta, Point3& omghat, double& theta);
    static bool nearZero(double val);

    static Matrix3x3 eulerToRot(const Point3& rotation);

    // interpolation
    static std::vector<std::vector<double>> interp2(const std::vector<std::vector<double>>& img, const std::vector<std::vector<double>>& Xd, const std::vector<std::vector<double>>& Yd);
    static std::vector<std::vector<std::vector<double>>> interp2(const std::vector<std::vector<std::vector<double>>>& img, const std::vector<std::vector<double>>& Xd, const std::vector<std::vector<double>>& Y);
    static double bilinearInterpolate(const std::vector<std::vector<double>>& img, double x, double y);

    // operator overload
    friend Point3 operator+(const Point3& lhs, const Point3& rhs);
    friend Point3 operator-(const Point3& lhs, const Point3& rhs);
    friend Point3 operator*(const Point3& lhs, const double& rhs);
    friend Point3 operator*(const double& lhs, const Point3& rhs);
    friend Point3 operator*(const Point3& lhs, const Point3& rhs);
    friend Point3 operator/(const Point3& lhs, const double& rhs);
    friend Point3 operator/(const double& lhs, const Point3& rhs);

    friend Point2 operator+(const Point2& lhs, const Point2& rhs);
    friend Point2 operator-(const Point2& lhs, const Point2& rhs);
    friend Point2 operator*(const Point2& lhs, const double& rhs);
    friend Point2 operator*(const double& lhs, const Point2& rhs);
    friend Point2 operator*(const Point2& lhs, const Point2& rhs);
    friend Point2 operator/(const Point2& lhs, const double& rhs);
    friend Point2 operator/(const double& lhs, const Point2& rhs);

    template<typename T>
    static T clamp(T val, T mn, T mx) {
        return std::max(std::min(val, mx), mn);
    }

    template <typename T, std::size_t N>
    static T dot(const std::array<T, N>& lhs, const std::array<T, N>& rhs) {
        T result = 0;
        for (std::size_t i = 0; i < N; ++i) {
            result += lhs[i] * rhs[i];
        }
        return result;
    }

    template <typename T, std::size_t N>
    static std::vector<T> arrayToVector(const std::array<T, N>& arr) {
        return std::vector<T>(arr.begin(), arr.end());
    }

    template <typename T, std::size_t N>
    static std::vector<std::vector<T>> arrayToVector(const std::array<std::array<T, N>, N>& arr) {
        std::vector<std::vector<double>> vec(N, std::vector<T>(N));
        for (std::size_t i = 0; i < N; ++i) {
            std::copy(arr[i].begin(), arr[i].end(), vec[i].begin());
        }

        return vec;
    }

    template <typename T, std::size_t N>
    static std::array<T, N> vectorToArray(const std::vector<T>& vec) {
        if (vec.size() != N) {
            throw std::runtime_error("Vector size does not match array size");
        }

        std::array<T, N> arr;
        std::copy(vec.begin(), vec.end(), arr.begin());
        return arr;
    }

    template <typename T, std::size_t N>
    static std::array<std::array<T, N>, N> vectorToArray(const std::vector<std::vector<T>>& vec) {
        if (vec.size() != N || std::any_of(vec.begin(), vec.end(), [](const std::vector<T>& row) { return row.size() != N; })) {
            throw std::runtime_error("Vector size does not match array size");
        }

        std::array<std::array<T, N>, N> arr;
        for (std::size_t i = 0; i < N; ++i) {
            std::copy(vec[i].begin(), vec[i].end(), arr[i].begin());
        }

        return arr;
    }

    template <std::size_t N>
    static std::array<double, N> normalize(const std::array<double, N>& vec) {
        double magnitude = 0.0;
        for (const auto& val : vec) {
            magnitude += val * val;
        }
        magnitude = std::sqrt(magnitude);
        if (magnitude == 0) {
            throw std::invalid_argument("Cannot normalize a zero vector");
        }

        std::array<double, N> normalizedVec;
        for (std::size_t i = 0; i < N; ++i) {
            normalizedVec[i] = vec[i] / magnitude;
        }
        return normalizedVec;
    }

    template <typename T, std::size_t N>
    static std::array<std::array<T, N>, N> transposeMatrix(const std::array<std::array<T, N>, N>& matrix) {
        std::array<std::array<T, N>, N> transposedMatrix {};

        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                transposedMatrix[j][i] = matrix[i][j];
            }
        }

        return transposedMatrix;
    }

    template <std::size_t N>
    static std::array<std::array<double, N>, N> matrixMultiply(const std::array<std::array<double, N>, N>& lhs, const std::array<std::array<double, N>, N>& rhs) {
        // Initialize the result matrix with zeros
        std::array<std::array<double, N>, N> result = {};

        // Perform matrix multiplication
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = 0; j < N; ++j) {
                for (std::size_t k = 0; k < N; ++k) {
                    result[i][j] += lhs[i][k] * rhs[k][j];
                }
            }
        }

        return result;
    }
};

// operator overloads
inline
Point3 operator+(const Point3& lhs, const Point3& rhs) {
    return { lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2] };
}

inline
Point3 operator-(const Point3& lhs, const Point3& rhs) {
    return { lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2] };
}

inline
Point3 operator-(const Point3& lhs) {
    return { -lhs[0], -lhs[1], -lhs[2] };
}

inline
Point3 operator*(const Point3& lhs, const double& rhs) {
    return { lhs[0] * rhs, lhs[1] * rhs, lhs[2] * rhs };
}

inline
Point3 operator*(const double& lhs, const Point3& rhs) {
    return { lhs * rhs[0], lhs * rhs[1], lhs * rhs[2] };
}

inline
Point3 operator*(const Point3& lhs, const Point3& rhs) {
    return { lhs[0] * rhs[0], lhs[1] * rhs[1], lhs[2] * rhs[2] };
}

inline
Point3 operator/(const Point3& lhs, const double& rhs) {
    return { lhs[0] / rhs, lhs[1] / rhs, lhs[2] / rhs };
}

inline
Point3 operator/(const double& lhs, const Point3& rhs) {
    return { lhs / rhs[0], lhs / rhs[1], lhs / rhs[2] };
}

inline
Point2 operator+(const Point2& lhs, const Point2& rhs) {
    return { lhs[0] + rhs[0], lhs[1] + rhs[1] };
}

inline
Point2 operator-(const Point2& lhs, const Point2& rhs) {
    return { lhs[0] - rhs[0], lhs[1] - rhs[1] };
}

inline
Point2 operator-(const Point2& lhs) {
    return { -lhs[0], -lhs[1] };
}

inline
Point2 operator*(const Point2& lhs, const double& rhs) {
    return { lhs[0] * rhs, lhs[1] * rhs };
}

inline
Point2 operator*(const double& lhs, const Point2& rhs) {
    return { lhs * rhs[0], lhs * rhs[1] };
}

inline
Point2 operator*(const Point2& lhs, const Point2& rhs) {
    return { lhs[0] * rhs[0], lhs[1] * rhs[1] };
}

inline
Point2 operator/(const Point2& lhs, const double& rhs) {
    return { lhs[0] / rhs, lhs[1] / rhs };
}

inline
Point2 operator/(const double& lhs, const Point2& rhs) {
    return { lhs / rhs[0], lhs / rhs[1] };
}

#endif // COMMONMATH_H
