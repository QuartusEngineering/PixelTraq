#include "utilities/common_math.h"
#include "camera/camera.h"
#include "camera/pinhole.h"
#include <algorithm>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Evaluates a polynomial at a given value x.
 *
 * @param coeffs The coefficients of the polynomial, where coeffs[i] is the coefficient for x^i.
 * @param x The value at which to evaluate the polynomial.
 * @return The result of the polynomial evaluation.
 */
double CommonMath::evaluatePolynomial(const std::vector<double>& coeffs, double x) {

    double result = 0.0;
    for (int i = coeffs.size()-1; i >= 0; --i) {
        result = result * x + coeffs[i];
    }
    return result;
}

/**
 * @brief Checks if all elements in a vector are zero.
 *
 * @param v The vector to check.
 * @return True if all elements are zero, false otherwise.
 */
bool CommonMath::isZero(const std::vector<double>& v)
{
    return std::all_of(v.begin(), v.end(), [](double i) { return i == 0; });
}

/**
 * @brief Evaluates a Fourier series at a given value phi.
 *
 * @param fourier_coeff The Fourier coefficients.
 * @param phi The value at which to evaluate the Fourier series.
 * @return The result of the Fourier series evaluation.
 */
double CommonMath::evaluateFourier(const std::vector<double>& fourier_coeff, double phi)
{
    int n_coeff = fourier_coeff.size();
    double cos_phi = std::cos(phi);
    double sin_phi = std::sin(phi);

    double sum = fourier_coeff[0] * cos_phi + fourier_coeff[1] * sin_phi;

    if (n_coeff > 2)
    {
        sum += fourier_coeff[2] * (2 * cos_phi * cos_phi - 1) + fourier_coeff[3] * (2 * sin_phi * cos_phi);
    }

    // higher order terms
    if (n_coeff > 4)
    {
        for (int i = 3; i < 3 + (n_coeff/2-2); ++i)
        {
            sum += fourier_coeff[2 * i - 2] * std::cos(phi * i) + fourier_coeff[2 * i - 1] * std::sin(phi * i);
        }
    }

    return sum;
}

/**
 * @brief Performs bilinear interpolation on a 3D image at given coordinates.
 *
 * @param img The source image.
 * @param Xd The x-coordinates for interpolation.
 * @param Yd The y-coordinates for interpolation.
 * @return The interpolated image.
 */
std::vector<std::vector<std::vector<double>>> CommonMath::interp2(
    const std::vector<std::vector<std::vector<double>>>& img,
    const std::vector<std::vector<double>>& Xd,
    const std::vector<std::vector<double>>& Yd
) {

    if (img.size() == 0 || Xd.size() == 0 || Yd.size() == 0)
    {
        return {};
    }

    int numChannels = img.size();
    int height = Xd.size();
    int width = Xd[0].size();

    // Create an output image with the same number of channels
    std::vector<std::vector<std::vector<double>>> outputImg(
        numChannels,
        std::vector<std::vector<double>>(height, std::vector<double>(width, 0))
    );

    // Loop over each channel
    for (int c = 0; c < numChannels; ++c) {

        outputImg[c] = interp2(img[c], Xd, Yd);
    }

    return outputImg;
}

/**
 * @brief Performs bilinear interpolation on a 2D image at given coordinates.
 *
 * @param img The source image.
 * @param Xd The x-coordinates for interpolation.
 * @param Yd The y-coordinates for interpolation.
 * @return The interpolated image.
 */
std::vector<std::vector<double>>  CommonMath::interp2(
    const std::vector<std::vector<double>>& img,
    const std::vector<std::vector<double>>& Xd,
    const std::vector<std::vector<double>>& Yd
) {

    if (img.size() == 0 || Xd.size() == 0 || Yd.size() == 0 )
    {
        return {};
    }

    // Create an output image of the same size as Xd/Yd
    int height = Xd.size();
    int width = Xd[0].size();
    std::vector<std::vector<double>> outputImg(height, std::vector<double>(width, 0));

    // Loop over each point in the destination grid
    #pragma omp parallel for
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Interpolate pixel value from img at (Xd[y][x], Yd[y][x])
            outputImg[y][x] = bilinearInterpolate(img, Xd[y][x], Yd[y][x]);
        }
    }

    return outputImg;
}

/**
 * @brief Performs bilinear interpolation on a single point in a 2D image.
 *
 * @param img The source image.
 * @param x The x-coordinate for interpolation.
 * @param y The y-coordinate for interpolation.
 * @return The interpolated value.
 */
double CommonMath::bilinearInterpolate(const std::vector<std::vector<double>>& img, double x, double y) {
    // Get 4 pixel inds surrounding x and y
    // Ensure indices are within bounds
    int height = img.size();
    if (height == 0)
    {
        return {};
    }
    int width = img[0].size();
    if (width == 0)
    {
        return {};
    }

    
    if (x < 0 || y < 0 || x > width || y > height) {
        return 0;
    }
    int x1 = static_cast<int>(std::floor(x));
    int y1 = static_cast<int>(std::floor(y));
    int x2 = x1 + 1;
    int y2 = y1 + 1;

    // Get fractional value of intermediate pixels
    double x_frac = x - x1;
    double y_frac = y - y1;

    // Clamp indices and get 4 corner pixel values
    x1 = CommonMath::clamp(x1, 0, width - 1);
    x2 = CommonMath::clamp(x2, 0, width - 1);
    y1 = CommonMath::clamp(y1, 0, height - 1);
    y2 = CommonMath::clamp(y2, 0, height - 1);
    double Q11 = img[y1][x1];
    double Q12 = img[y2][x1];
    double Q21 = img[y1][x2];
    double Q22 = img[y2][x2];

    // Perform bilinear interpolation
    double R1 = (1 - x_frac) * Q11 + x_frac * Q21;
    double R2 = (1 - x_frac) * Q12 + x_frac * Q22;
    double q = (1 - y_frac) * R1 + y_frac * R2;

    return q;
}

/**
 * @brief Rotates a point using a given rotation matrix.
 *
 * @param point The point to rotate.
 * @param rotation_matrix The rotation matrix.
 * @return The rotated point.
 */
Point3 CommonMath::rotatePoint(const Point3& point, const Matrix3x3& rotation_matrix) {

    Point3 rotatedPoint = {
       rotation_matrix[0][0] * point[0] + rotation_matrix[0][1] * point[1] + rotation_matrix[0][2] * point[2],
       rotation_matrix[1][0] * point[0] + rotation_matrix[1][1] * point[1] + rotation_matrix[1][2] * point[2],
       rotation_matrix[2][0] * point[0] + rotation_matrix[2][1] * point[1] + rotation_matrix[2][2] * point[2]
    };
    return rotatedPoint;
}

/**
 * @brief Rotates a list of points using a given rotation matrix.
 *
 * @param points The list of points to rotate.
 * @param rotation_matrix The rotation matrix.
 * @return The list of rotated points.
 */
std::vector<Point3> CommonMath::rotatePoints(const std::vector<Point3>& points, const Matrix3x3& rotation_matrix) {
    std::vector<Point3> rotatedPoints;

    int N = points.size();
    rotatedPoints.reserve(N);
    rotatedPoints.resize(N);

    #pragma omp parallel for
    for (size_t i = 0; i < N; ++i) {
        rotatedPoints[i] = rotatePoint(points[i], rotation_matrix);
    }

    return rotatedPoints;
}

/**
 * @brief Transforms a point using a given rotation matrix and translation vector.
 *
 * @param point The point to transform.
 * @param rotation_matrix The rotation matrix.
 * @param translation The translation vector.
 * @return The transformed point.
 */
Point3 CommonMath::transformPoint(const Point3& point, const Matrix3x3& rotation_matrix, const Point3& translation) {

    Point3 transformedPoint;

    transformedPoint = rotatePoint(point, rotation_matrix);
    transformedPoint = {
        transformedPoint[0] + translation[0],
        transformedPoint[1] + translation[1],
        transformedPoint[2] + translation[2]
    };

    return transformedPoint;
}

/**
 * @brief Transforms a list of points using a given rotation matrix and translation vector.
 *
 * @param points The list of points to transform.
 * @param rotation_matrix The rotation matrix.
 * @param translation The translation vector.
 * @return The list of transformed points.
 */
std::vector<Point3> CommonMath::transformPoints(const std::vector<Point3>& points, const Matrix3x3& rotation_matrix, const Point3& translation) {
    std::vector<Point3> transformedPoints;

    int N = points.size();
    transformedPoints.reserve(N);
    transformedPoints.resize(N);

    #pragma omp parallel for
    for (size_t i = 0; i < N; ++i) {
        transformedPoints[i] = transformPoint(points[i], rotation_matrix, translation);
    }

    return transformedPoints;
}

/**
 * @brief Computes the inverse of a rotation matrix.
 *
 * @param rotation_matrix The rotation matrix.
 * @return The inverse rotation matrix.
 */
Matrix3x3 CommonMath::rotationInverse(const Matrix3x3& rotation_matrix)
{
    return CommonMath::transposeMatrix(rotation_matrix);
}

/**
 * @brief Converts Euler angles to a rotation matrix.
 *
 * @param rotation The Euler angles (rx, ry, rz).
 * @return The rotation matrix.
 */
Matrix3x3 CommonMath::eulerToRot(const Point3& rotation) {
    double rx = rotation[0];
    double ry = rotation[1];
    double rz = rotation[2];

    Matrix3x3 rotationMatrix{};

    rotationMatrix[0][0] = cos(ry) * cos(rz);
    rotationMatrix[0][1] = -cos(ry) * sin(rz);
    rotationMatrix[0][2] = sin(ry);

    rotationMatrix[1][0] = cos(rx) * sin(rz) + sin(rx) * sin(ry) * cos(rz);
    rotationMatrix[1][1] = cos(rx) * cos(rz) - sin(rx) * sin(ry) * sin(rz);
    rotationMatrix[1][2] = -sin(rx) * cos(ry);

    rotationMatrix[2][0] = sin(rx) * sin(rz) - cos(rx) * sin(ry) * cos(rz);
    rotationMatrix[2][1] = sin(rx) * cos(rz) + cos(rx) * sin(ry) * sin(rz);
    rotationMatrix[2][2] = cos(rx) * cos(ry);

    return rotationMatrix;
}

/**
 * @brief Transposes a given matrix.
 *
 * @param matrix The matrix to transpose.
 * @return The transposed matrix.
 * @throws std::invalid_argument if the matrix rows are of inconsistent size.
 */
std::vector<std::vector<double>> CommonMath::transposeMatrix(const std::vector<std::vector<double>>& matrix) {

    size_t n = matrix.size();
    size_t m = matrix[0].size();

    std::vector<std::vector<double>> transposedMatrix(m, std::vector<double>(n));

    for (size_t i = 0; i < n; ++i) {
        if (matrix[i].size() != m)
        {
            throw std::invalid_argument("Matrix row size is inconsistent");
        }
        for (size_t j = 0; j < m; ++j) {
            transposedMatrix[j][i] = matrix[i][j];
        }
    }

    return transposedMatrix;
}

/**
 * @brief Computes the intersection of rays from two cameras.
 *
 * @param cameraL The left camera.
 * @param cameraR The right camera.
 * @param raysL The rays from the left camera.
 * @param raysR The rays from the right camera.
 * @return The 3D intersection points.
 */
std::vector<Point3> CommonMath::intersectRays(std::shared_ptr<Camera> cameraL, std::shared_ptr<Camera> cameraR, const std::vector<Point3>& raysL, const std::vector<Point3>& raysR)
{
    auto centerL = cameraL->getInvTranslation();
    auto centerR = cameraR->getInvTranslation();

    // transform rays to object frame
    auto raysLObject = CommonMath::rotatePoints(raysL, cameraL->getInvRotationMatrix());
    auto raysRObject = CommonMath::rotatePoints(raysR, cameraR->getInvRotationMatrix());

    Point3 p1 = centerL;
    Point3 p3 = centerR;

    std::vector<Point3> points3D;
    points3D.reserve(raysL.size());
    points3D.resize(raysL.size());

    #pragma omp parallel for
    for (int i = 0; i < raysL.size(); i++)
    {
        Point3 p2{}, p4{}, pa{}, pb{};
        double mua, mub;
        p2 = centerL + raysLObject[i];
        p4 = centerR + raysRObject[i];
        if (CommonMath::lineLineIntersect(p1, p2, p3, p4, pa, pb, mua, mub))
        {
            Point3 result = (pa + pb)/2;
            points3D[i] = result;
        }
        else
        {
            points3D[i] = { DNAN,DNAN,DNAN };
        }
    }

    return points3D;
}

/**
 * @brief Computes the intersection of two lines.
 *
 * @param p1 The first point of the first line.
 * @param p2 The second point of the first line.
 * @param p3 The first point of the second line.
 * @param p4 The second point of the second line.
 * @param pa The intersection point on the first line.
 * @param pb The intersection point on the second line.
 * @param mua The parameter of the intersection point on the first line.
 * @param mub The parameter of the intersection point on the second line.
 * @return True if there is an intersection, false otherwise.
 */
int CommonMath::lineLineIntersect(Point3 p1, Point3 p2, Point3 p3, Point3 p4, Point3& pa, Point3& pb, double& mua, double& mub)
{
    /*
       Calculate the line segment PaPb that is the shortest route between
       two lines P1P2 and P3P4. Calculate also the values of mua and mub where
          Pa = P1 + mua (P2 - P1)
          Pb = P3 + mub (P4 - P3)
       Return FALSE if no solution exists.
       Credit: Paul Bourke
    */

    double d1343, d4321, d1321, d4343, d2121;
    double numer, denom;

    Point3 p13 = p1 - p3;
    Point3 p43 = p4 - p3;

    if (std::abs(p43[0]) < EPS && std::abs(p43[1]) < EPS && std::abs(p43[2]) < EPS)
        return(false);
    Point3 p21 = p2 - p1;

    if (std::abs(p21[0]) < EPS && std::abs(p21[1]) < EPS && std::abs(p21[2]) < EPS)
        return(false);

    d1343 = dot(p13, p43);
    d4321 = dot(p43, p21);
    d1321 = dot(p13, p21);
    d4343 = dot(p43, p43);
    d2121 = dot(p21, p21);

    denom = d2121 * d4343 - d4321 * d4321;
    if (std::abs(denom) < EPS)
        return(false);
    numer = d1343 * d4321 - d1321 * d4343;

    mua = numer / denom;
    mub = (d1343 + d4321 * (mua)) / d4343;

    pa = p1 + mua * p21;
    pb = p3 + mua * p43;

    return(true);
}

/**
 * @brief Performs stereo rectification for two pinhole cameras.
 *
 * @param cameraL The left camera.
 * @param cameraR The right camera.
 * @param Rl The output rotation matrix for the left camera.
 * @param Rr The output rotation matrix for the right camera.
 */
void CommonMath::stereoRectify(std::shared_ptr<Pinhole> cameraL, std::shared_ptr<Pinhole> cameraR, Matrix3x3& Rl, Matrix3x3& Rr) {

    // normalize to standard pinhole for both cameras
    auto focalLengthL = cameraL->getFocalLength();
    auto focalLengthR = cameraR->getFocalLength();
    auto focalLengthNew = { (focalLengthL[0] + focalLengthR[0]) / 2,(focalLengthL[1] + focalLengthR[1]) / 2 };
    cameraL->setFocalLength(focalLengthNew);
    cameraR->setFocalLength(focalLengthNew);

    auto imageSize = cameraL->getImageSize();
    std::vector<double> principalPointNew = { imageSize[0] / 2.0 - 1 , imageSize[1] / 2.0 - 1 };
    cameraL->setPrincipalPoint(principalPointNew);
    cameraR->setPrincipalPoint(principalPointNew);

    // Compute 1/2 rotation matrix
    auto R_left = cameraL->getRotationMatrix();
    auto t_left = cameraL->getTranslation();
    auto R_inv_right = cameraR->getInvRotationMatrix();
    auto t_inv_right = cameraR->getInvTranslation();
    auto t_inv_right_in_left = rotatePoint(t_inv_right, R_left);
    Point3 t = t_inv_right_in_left + t_left;
    auto R = CommonMath::matrixMultiply(R_left, R_inv_right);

    auto R_se3 = matrixLog3(R);
    for (auto& row : R_se3) {
        for (auto& element : row) {
            element /= 2.0;
        }
    }
    auto rl = CommonMath::transposeMatrix(matrixExp3(R_se3));

    auto rr = CommonMath::transposeMatrix(rl);
    t = rotatePoint(t, rl);

    auto e1 = normalize(t);
    auto n1 = norm({ t[0], t[1] });
    std::array<double,3> e2 { -t[1] / n1, t[0] / n1, 0 };
    auto e3 = cross(e1, e2);

    Matrix3x3 Rrect = { e1, e2, e3 };

    Rl = CommonMath::matrixMultiply<3>(Rrect,rl);
    Rr = CommonMath::matrixMultiply<3>(Rrect,rr);
}

/**
 * @brief Computes the Euclidean norm of a 3D vector.
 *
 * @param vec The vector.
 * @return The norm of the vector.
 */
double CommonMath::norm(const Point3& vec) {
    return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

/**
 * @brief Computes the cross product of two 3D vectors.
 *
 * @param lhs The left-hand side vector.
 * @param rhs The right-hand side vector.
 * @return The cross product vector.
 */
Point3 CommonMath::cross(const Point3& lhs, const Point3& rhs) {
    return {
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0]
    };
}

/**
 * @brief Checks if a value is near zero.
 *
 * @param val The value to check.
 * @return True if the value is near zero, false otherwise.
 */
bool CommonMath::nearZero(double val) {
    return std::abs(val) < EPS;
}

/**
 * @brief Converts an angular velocity vector to a unit vector and angle.
 *
 * @param omgtheta The angular velocity vector.
 * @param omghat The unit vector.
 * @param theta The angle.
 */
void CommonMath::axisAng3(const Point3& omgtheta, Point3& omghat, double& theta) {
    theta = norm(omgtheta);
    omghat = { omgtheta[0] / theta, omgtheta[1] / theta, omgtheta[2] / theta };
}

/**
 * @brief Computes the matrix logarithm of a rotation matrix.
 *
 * @param R The rotation matrix.
 * @return The matrix logarithm.
 */
std::array<Point3, 3> CommonMath::matrixLog3(const Matrix3x3& R) {
    double acosinput = (R[0][0] + R[1][1] + R[2][2] - 1) / 2.0;
    Matrix3x3 so3mat{};

    if (acosinput >= 1) {
        // acosinput == 1 -> theta == 0. Rotation matrix is the identity matrix
        return so3mat; // Return zero matrix
    }
    else if (acosinput <= -1) {
        std::array<double, 3> omg{};
        if (!nearZero(1 + R[2][2])) {
            omg = {
                (1 / std::sqrt(2 * (1 + R[2][2]))) * R[0][2],
                (1 / std::sqrt(2 * (1 + R[2][2]))) * R[1][2],
                (1 / std::sqrt(2 * (1 + R[2][2]))) * (1 + R[2][2])
            };
        }
        else if (!nearZero(1 + R[1][1])) {
            omg = {
                (1 / std::sqrt(2 * (1 + R[1][1]))) * R[0][1],
                (1 / std::sqrt(2 * (1 + R[1][1]))) * (1 + R[1][1]),
                (1 / std::sqrt(2 * (1 + R[1][1]))) * R[2][1]
            };
        }
        else {
            omg = {
                (1 / std::sqrt(2 * (1 + R[0][0]))) * (1 + R[0][0]),
                (1 / std::sqrt(2 * (1 + R[0][0]))) * R[1][0],
                (1 / std::sqrt(2 * (1 + R[0][0]))) * R[2][0]
            };
        }
        so3mat = CommonMath::vecToso3({ M_PI * omg[0], M_PI * omg[1], M_PI * omg[2] });
    }
    else {
        double theta = std::acos(acosinput);
        double multiplier = theta / (2 * std::sin(theta));
        so3mat = {
            {
                { 0, multiplier * (R[0][1] - R[1][0]), multiplier * (R[0][2] - R[2][0]) },
                { multiplier * (R[1][0] - R[0][1]), 0, multiplier * (R[1][2] - R[2][1]) },
                { multiplier * (R[2][0] - R[0][2]), multiplier * (R[2][1] - R[1][2]), 0 }
            }
        };
    }

    return so3mat;
}

/**
 * @brief Converts a 3D rotation vector to a skew-symmetric matrix (so(3) matrix).
 *
 * @param omg The 3D rotation vector.
 * @return The skew-symmetric matrix.
 */
Matrix3x3 CommonMath::vecToso3(const Point3& omg) {
    return {
        { { 0, -omg[2], omg[1] },
          { omg[2], 0, -omg[0] },
          { -omg[1], omg[0], 0 } }
    };
}

/**
 * @brief Converts a skew-symmetric matrix (so(3) matrix) to a 3D rotation vector.
 *
 * @param so3mat The skew-symmetric matrix.
 * @return The 3D rotation vector.
 */
Point3 CommonMath::so3ToVec(const Matrix3x3& so3mat) {
    return { so3mat[2][1], so3mat[0][2], so3mat[1][0] };
}

/**
 * @brief Computes the matrix exponential of a skew-symmetric matrix (so(3) matrix).
 *
 * @param so3mat The skew-symmetric matrix.
 * @return The rotation matrix.
 */
Matrix3x3 CommonMath::matrixExp3(const Matrix3x3& so3mat) {
    Point3 omgtheta = CommonMath::so3ToVec(so3mat);
    double theta = CommonMath::norm(omgtheta);

    Matrix3x3 R = { {
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 }
    } };

    if (nearZero(theta)) {
        return R; // Return identity matrix
    }
    else {
        Point3 omghat;
        axisAng3(omgtheta, omghat, theta);

        Matrix3x3 omgmat = { {
            { 0, -omghat[2], omghat[1] },
            { omghat[2], 0, -omghat[0] },
            { -omghat[1], omghat[0], 0 }
        } };

        Matrix3x3 omgmat2 = { {
            { omgmat[0][0] * omgmat[0][0] + omgmat[0][1] * omgmat[1][0] + omgmat[0][2] * omgmat[2][0],
              omgmat[0][0] * omgmat[0][1] + omgmat[0][1] * omgmat[1][1] + omgmat[0][2] * omgmat[2][1],
              omgmat[0][0] * omgmat[0][2] + omgmat[0][1] * omgmat[1][2] + omgmat[0][2] * omgmat[2][2] },

            { omgmat[1][0] * omgmat[0][0] + omgmat[1][1] * omgmat[1][0] + omgmat[1][2] * omgmat[2][0],
              omgmat[1][0] * omgmat[0][1] + omgmat[1][1] * omgmat[1][1] + omgmat[1][2] * omgmat[2][1],
              omgmat[1][0] * omgmat[0][2] + omgmat[1][1] * omgmat[1][2] + omgmat[1][2] * omgmat[2][2] },

            { omgmat[2][0] * omgmat[0][0] + omgmat[2][1] * omgmat[1][0] + omgmat[2][2] * omgmat[2][0],
              omgmat[2][0] * omgmat[0][1] + omgmat[2][1] * omgmat[1][1] + omgmat[2][2] * omgmat[2][1],
              omgmat[2][0] * omgmat[0][2] + omgmat[2][1] * omgmat[1][2] + omgmat[2][2] * omgmat[2][2] }
        } };

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R[i][j] = R[i][j] + sin(theta) * omgmat[i][j] + (1 - cos(theta)) * omgmat2[i][j];
            }
        }
    }

    return R;
}