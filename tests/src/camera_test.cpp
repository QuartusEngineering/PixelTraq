#include <gtest/gtest.h>
#include "pixeltraq.h"

class CameraTest {
public:
    static void CompareMatrix(Matrix3x3 matrix, Matrix3x3 matrixExp, double tol = 1e-6)
    {
        for (size_t i = 0; i < matrix.size(); ++i) {
            for (size_t j = 0; j < matrix[0].size(); ++j) {
                EXPECT_LT(std::abs(matrix[i][j] - matrixExp[i][j]), tol);
            }
        }
    }

    static void CompareVector(std::vector<double> vector, std::vector<double> vectorExp, double tol = 1e-6)
    {
        for (size_t i = 0; i < vector.size(); ++i) {
            EXPECT_LT(std::abs(vector[i] - vectorExp[i]), tol);
        }
    }

    static void CompareArray(std::array<double,3> array, std::array<double, 3> arrayExp, double tol = 1e-6)
    {
        for (size_t i = 0; i < array.size(); ++i) {
            EXPECT_LT(std::abs(array[i] - arrayExp[i]), tol);
        }
    }
};

// Test for display method with Pinhole input
TEST(CameraTest, display_PinholeInput_Displaysvalid) {
    Pinhole pinhole;
    std::stringstream buffer;
    std::streambuf* old = std::cout.rdbuf(buffer.rdbuf()); // Redirect cout

    pinhole.display();

    std::cout.rdbuf(old); // Restore cout
    std::string output = buffer.str();
    EXPECT_NE(output.find("Pinhole Camera Model"), std::string::npos);
}

// Test for display method with BrownConrady input
TEST(CameraTest, display_BrownConradyInput_Displaysvalid) {
    BrownConrady brownConrady;
    std::stringstream buffer;
    std::streambuf* old = std::cout.rdbuf(buffer.rdbuf()); // Redirect cout

    brownConrady.display();

    std::cout.rdbuf(old); // Restore cout
    std::string output = buffer.str();
    EXPECT_NE(output.find("Brown Conrady Camera Model"), std::string::npos);
}

// Test for display method with Kannala input
TEST(CameraTest, display_KannalaInput_Displaysvalid) {
    Kannala kannala;
    std::stringstream buffer;
    std::streambuf* old = std::cout.rdbuf(buffer.rdbuf()); // Redirect cout

    kannala.display();

    std::cout.rdbuf(old); // Restore cout
    std::string output = buffer.str();
    EXPECT_NE(output.find("Kannala Camera Model"), std::string::npos);
}

// Test for display method with General FTheta input
TEST(CameraTest, display_GenFThetaInput_Displaysvalid) {
    GenFTheta genFTheta;
    std::stringstream buffer;
    std::streambuf* old = std::cout.rdbuf(buffer.rdbuf()); // Redirect cout

    genFTheta.display();

    std::cout.rdbuf(old); // Restore cout
    std::string output = buffer.str();
    EXPECT_NE(output.find("General FTheta Camera Model"), std::string::npos);
}

// Test for display method with General FTanTheta input
TEST(CameraTest, display_GenFTanTheta_Displaysvalid) {
    GenFTanTheta genFTanTheta;
    std::stringstream buffer;
    std::streambuf* old = std::cout.rdbuf(buffer.rdbuf()); // Redirect cout

    genFTanTheta.display();

    std::cout.rdbuf(old); // Restore cout
    std::string output = buffer.str();
    EXPECT_NE(output.find("General FTan Theta Camera Model"), std::string::npos);
}

// Test for getImageSize with zero argument constructor
TEST(CameraTest, getImageSize_ZeroArgConstructor_validresult) {
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>();
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    std::vector<int> imageSize = camera->getImageSize();
    EXPECT_EQ(imageSize.size(), 2);
    EXPECT_EQ(imageSize[0], 0);
    EXPECT_EQ(imageSize[1], 0);
}

// Test for getImageSize with multiple arguments constructor
TEST(CameraTest, getImageSize_MultiArgConstructor_validresult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>(focal_length, principal_point, skew, image_size);
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    std::vector<int> result = camera->getImageSize();
    EXPECT_EQ(result.size(), 2);
    EXPECT_EQ(result[0], 640);
    EXPECT_EQ(result[1], 480);
}

// Test for setImageSize with valid size inputs
TEST(CameraTest, setImageSize_ValidSizeInputs_setparameter) {
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>();
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    std::vector<int> imageSize = { 800, 600 };
    camera->setImageSize(imageSize);
    std::vector<int> result = camera->getImageSize();
    EXPECT_EQ(result.size(), 2);
    EXPECT_EQ(result[0], 800);
    EXPECT_EQ(result[1], 600);
}

// Test for setImageSize with invalid size input (should throw exception)
TEST(CameraTest, setImageSize_InValidSizeInput_ThrowException) {
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>();
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    std::vector<int> invalidSize = { 800 }; // Only one element, should be two
    EXPECT_THROW(camera->setImageSize(invalidSize), std::invalid_argument);
}

// Test for get_translation with zero argument constructor
TEST(CameraTest, gettranslation_ZeroArgConstructor_validresult) {
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>();
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    Point3 translation = camera->getTranslation();
    EXPECT_EQ(translation.size(), 3);
    EXPECT_EQ(translation[0], 0.0);
    EXPECT_EQ(translation[1], 0.0);
    EXPECT_EQ(translation[2], 0.0);
}

// Test for get_translation with multiple arguments constructor
TEST(CameraTest, gettranslation_MultiArgConstructor_validresult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };
    Point3 rotation = { 0.0, 0.0, 0.0 };
    Point3 translation = { 1.0, 2.0, 3.0 };
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>(focal_length, principal_point, skew, image_size, rotation, translation);
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    Point3 result = camera->getTranslation();
    EXPECT_EQ(result.size(), 3);
    EXPECT_EQ(result[0], 1.0);
    EXPECT_EQ(result[1], 2.0);
    EXPECT_EQ(result[2], 3.0);
}

// Test for setTranslation with valid size inputs
TEST(CameraTest, settranslation_ValidSizeInputs_setparameter) {
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>();
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    Point3 translation = { 4.0, 5.0, 6.0 };
    camera->setTranslation(translation);
    Point3 result = camera->getTranslation();
    EXPECT_EQ(result.size(), 3);
    EXPECT_EQ(result[0], 4.0);
    EXPECT_EQ(result[1], 5.0);
    EXPECT_EQ(result[2], 6.0);
}

// Test for getRotation with zero argument constructor
TEST(CameraTest, getrotation_ZeroArgConstructor_validresult) {
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>();
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    Point3 rotation = camera->getRotation();
    EXPECT_EQ(rotation.size(), 3);
    EXPECT_EQ(rotation[0], 0.0);
    EXPECT_EQ(rotation[1], 0.0);
    EXPECT_EQ(rotation[2], 0.0);
}

// Test for getRotation with multiple arguments constructor
TEST(CameraTest, getrotation_MultiArgConstructor_validresult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };
    Point3 rotation = { 0.1, 0.2, 0.3 };
    Point3 translation = { 0.0, 0.0, 0.0 };
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>(focal_length, principal_point, skew, image_size, rotation, translation);
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    Point3 result = camera->getRotation();
    EXPECT_EQ(result.size(), 3);
    EXPECT_EQ(result[0], 0.1);
    EXPECT_EQ(result[1], 0.2);
    EXPECT_EQ(result[2], 0.3);
}

// Test for set_rotation with valid size inputs
TEST(CameraTest, setrotation_ValidSizeInputs_setparameter) {
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>();
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    Point3 translation = { 1, 2, 3 };
    camera->setTranslation(translation);
    Point3 rotation = { 0.4, 0.5, 0.6 };
    camera->setRotation(rotation);
    Point3 result = camera->getRotation();
    EXPECT_EQ(result.size(), 3);
    EXPECT_EQ(result[0], 0.4);
    EXPECT_EQ(result[1], 0.5);
    EXPECT_EQ(result[2], 0.6);

    Matrix3x3 rotationMatrix = camera->getRotationMatrix();
    EXPECT_EQ(rotationMatrix.size(), 3);
    EXPECT_EQ(rotationMatrix[0].size(), 3);
    EXPECT_EQ(rotationMatrix[1].size(), 3);
    EXPECT_EQ(rotationMatrix[2].size(), 3);

    Matrix3x3 rotationMatrixExp = 
    { {
        {0.724300143351802, -0.495520388354132, 0.479425538604203 },
        { 0.674157922396867, 0.654767330379896, -0.341746746490328 },
        { -0.144569699488170, 0.570735742522365, 0.808307066774345 }
    } };

    CameraTest::CompareMatrix(rotationMatrix, rotationMatrixExp, 1e-6);

    Matrix3x3 invRotationMatrix = camera->getInvRotationMatrix();
    EXPECT_EQ(invRotationMatrix.size(), 3);
    EXPECT_EQ(invRotationMatrix[0].size(), 3);
    EXPECT_EQ(invRotationMatrix[1].size(), 3);
    EXPECT_EQ(invRotationMatrix[2].size(), 3);

    Matrix3x3 rotationMatrixInvExp =
    { {
        {0.724300143351802, 0.674157922396867, -0.144569699488170},
        {-0.495520388354132, 0.654767330379896, 0.570735742522365},
        {0.479425538604203, -0.341746746490328, 0.808307066774345}
    } };

    CameraTest::CompareMatrix(invRotationMatrix, rotationMatrixInvExp, 1e-6);

    Point3 invTranslation = camera->getInvTranslation();
    EXPECT_EQ(invTranslation.size(), 3);

    Point3 invTranslationExp = 
    { 
        -1.638906889681027,
        -2.526221499972754,
        -2.220853245946583
    };

    CameraTest::CompareArray(invTranslation, invTranslationExp, 1e-6);

}

// Test for getInvTranslation with zero argument constructor
TEST(CameraTest, getinvtranslation_ZeroArgConstructor_validresult) {
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>();
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    Point3 invTranslation = camera->getInvTranslation();
    EXPECT_EQ(invTranslation.size(), 3);

    CameraTest::CompareArray(invTranslation, {0.0,0.0,0.0}, 1e-6);
}

// Test for getInvTranslation with multiple arguments constructor
TEST(CameraTest, getinvtranslation_MultiArgConstructor_validresult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };
    Point3 rotation = { 0.4, 0.5, 0.6 };
    Point3 translation = { 1.0, 2.0, 3.0 };
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>(focal_length, principal_point, skew, image_size, rotation, translation);
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    Point3 invTranslation = camera->getInvTranslation();
    EXPECT_EQ(invTranslation.size(), 3);

    Point3 invTranslationExp =
    {
        -1.638906889681027,
        -2.526221499972754,
        -2.220853245946583
    };

    CameraTest::CompareArray(invTranslation, invTranslationExp, 1e-6);
}

// Test for getRotationMatrix with zero argument constructor
TEST(CameraTest, getrotationmatrix_ZeroArgConstructor_validresult) {
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>();
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    Matrix3x3 rotationMatrix = camera->getRotationMatrix();
    EXPECT_EQ(rotationMatrix.size(), 3);
    EXPECT_EQ(rotationMatrix[0].size(), 3);
    EXPECT_EQ(rotationMatrix[1].size(), 3);
    EXPECT_EQ(rotationMatrix[2].size(), 3);

    Matrix3x3 rotationMatrixExp =
    { {
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 }
    } };

    CameraTest::CompareMatrix(rotationMatrix, rotationMatrixExp, 1e-6);

    camera->setRotation({ 0.4, 0.5, 0.6 });
    rotationMatrix = camera->getRotationMatrix();

    rotationMatrixExp =
    { {
        {0.724300143351802, -0.495520388354132, 0.479425538604203 },
        { 0.674157922396867, 0.654767330379896, -0.341746746490328 },
        { -0.144569699488170, 0.570735742522365, 0.808307066774345 }
    } };

    CameraTest::CompareMatrix(rotationMatrix, rotationMatrixExp, 1e-6);
}

// Test for getRotationMatrix with multiple arguments constructor
TEST(CameraTest, getrotationmatrix_MultiArgConstructor_validresult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };
    Point3 rotation = { 0.4, 0.5, 0.6 };
    Point3 translation = { 0.0, 0.0, 0.0 };
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>(focal_length, principal_point, skew, image_size, rotation, translation);
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    Matrix3x3 rotationMatrix = camera->getRotationMatrix();
    EXPECT_EQ(rotationMatrix.size(), 3);
    EXPECT_EQ(rotationMatrix[0].size(), 3);
    EXPECT_EQ(rotationMatrix[1].size(), 3);
    EXPECT_EQ(rotationMatrix[2].size(), 3);

    Matrix3x3 rotationMatrixExp =
    { {
        {0.724300143351802, -0.495520388354132, 0.479425538604203 },
        { 0.674157922396867, 0.654767330379896, -0.341746746490328 },
        { -0.144569699488170, 0.570735742522365, 0.808307066774345 }
    } };
    CameraTest::CompareMatrix(rotationMatrix, rotationMatrixExp, 1e-6);

    camera->setRotation({ 0.0, 0.0, 0.0 });
    rotationMatrix = camera->getRotationMatrix();

    rotationMatrixExp =
    { {
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 }
    } };

    CameraTest::CompareMatrix(rotationMatrix, rotationMatrixExp, 1e-6);
}

// Test for getInvRotationMatrix with zero argument constructor
TEST(CameraTest, getinvrotationmatrix_ZeroArgConstructor_validresult) {
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>();
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    Matrix3x3 invRotationMatrix = camera->getInvRotationMatrix();
    EXPECT_EQ(invRotationMatrix.size(), 3);
    EXPECT_EQ(invRotationMatrix[0].size(), 3);
    EXPECT_EQ(invRotationMatrix[1].size(), 3);
    EXPECT_EQ(invRotationMatrix[2].size(), 3);

    Matrix3x3 rotationMatrixExp =
    { {
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 }
    } };

    CameraTest::CompareMatrix(invRotationMatrix, rotationMatrixExp, 1e-6);

    camera->setRotation({ 0.4, 0.5, 0.6 });
    invRotationMatrix = camera->getInvRotationMatrix();

    rotationMatrixExp =
    { {
        {0.724300143351802, 0.674157922396867, -0.144569699488170},
        {-0.495520388354132, 0.654767330379896, 0.570735742522365},
        {0.479425538604203, -0.341746746490328, 0.808307066774345}
    } };

    CameraTest::CompareMatrix(invRotationMatrix, rotationMatrixExp, 1e-6);
}

// Test for getInvRotationMatrix with multiple arguments constructor
TEST(CameraTest, getinvrotationmatrix_MultiArgConstructor_validresult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };
    Point3 rotation = { 0.4, 0.5, 0.6 };
    Point3 translation = { 0.0, 0.0, 0.0 };
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>(focal_length, principal_point, skew, image_size, rotation, translation);
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);
    Matrix3x3 invRotationMatrix = camera->getInvRotationMatrix();
    EXPECT_EQ(invRotationMatrix.size(), 3);
    EXPECT_EQ(invRotationMatrix[0].size(), 3);
    EXPECT_EQ(invRotationMatrix[1].size(), 3);
    EXPECT_EQ(invRotationMatrix[2].size(), 3);

    Matrix3x3 rotationMatrixExp =
    { {
        { 0.724300143351802, 0.674157922396867, -0.144569699488170 },
        {-0.495520388354132, 0.654767330379896, 0.570735742522365},
        {0.479425538604203, -0.341746746490328, 0.808307066774345}
    } };

    CameraTest::CompareMatrix(invRotationMatrix, rotationMatrixExp, 1e-6);

    camera->setRotation({ 0.0, 0.0, 0.0 });
    invRotationMatrix = camera->getInvRotationMatrix();

    rotationMatrixExp =
    { {
        { 1, 0, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 }
    } };

    CameraTest::CompareMatrix(invRotationMatrix, rotationMatrixExp, 1e-6);
}

// Test for worldToCameraPnts with normal inputs
TEST(CameraTest, worldToCameraPnts_NormalInputs_validresult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };
    Point3 rotation = { 0.4, 0.5, 0.6 };
    Point3 translation = { 1.0, 2.0, 3.0 };
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>(focal_length, principal_point, skew, image_size, rotation, translation);
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);

    std::vector<std::array<double, 3>> worldPoints = { {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}} };
    std::vector<std::array<double, 3>> cameraPoints = camera->worldToCameraPnts(worldPoints);

    std::vector<std::array<double, 3>> arrays =
    {
        {2.171535982456147, 2.958452343685676, 6.421822985879595},
        {4.296151863261766, 5.919987862544982, 10.125242315305215}
    };

    CameraTest::CompareArray(cameraPoints[0], arrays[0], 1e-6);
    CameraTest::CompareArray(cameraPoints[1], arrays[1], 1e-6);
}

// Test for worldToCameraPnts with zero inputs
TEST(CameraTest, worldToCameraPnts_ZeroInputs_validresult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };
    Point3 rotation = { 0.4, 0.5, 0.6 };
    Point3 translation = { 1.0, 2.0, 3.0 };
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>(focal_length, principal_point, skew, image_size, rotation, translation);
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);

    std::vector<std::array<double, 3>> worldPoints = { {0.0,0.0,0.0} };
    std::vector<std::array<double, 3>> cameraPoints = camera->worldToCameraPnts(worldPoints);

    CameraTest::CompareArray(cameraPoints[0], { 1.0, 2.0, 3.0 }, 1e-6);
}

// Test for cameraToWorldPnts with normal inputs
TEST(CameraTest, cameraToWorldPnts_NormalInputs_validresult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };
    Point3 rotation = { 0.4, 0.5, 0.6 };
    Point3 translation = { 1.0, 2.0, 3.0 };
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>(focal_length, principal_point, skew, image_size, rotation, translation);
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);

    std::vector<std::array<double, 3>> cameraPoints = {
        {2.171535982456147, 2.958452343685676, 6.421822985879595},
        {4.296151863261766, 5.919987862544982, 10.125242315305215}
    };
    std::vector<std::array<double, 3>> worldPoints = camera->cameraToWorldPnts(cameraPoints);

    std::vector<std::array<double, 3>> arrays = { {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}} };

    CameraTest::CompareArray(worldPoints[0], arrays[0], 1e-6);
    CameraTest::CompareArray(worldPoints[1], arrays[1], 1e-6);
}

// Test for cameraToWorldPnts with zero inputs
TEST(CameraTest, cameraToWorldPnts_ZeroInputs_validresult) {
    std::vector<double> focal_length = { 2.0, 2.0 };
    std::vector<double> principal_point = { 1.0, 1.0 };
    double skew = 0.5;
    std::vector<int> image_size = { 640, 480 };
    Point3 rotation = { 0.4, 0.5, 0.6 };
    Point3 translation = { 1.0, 2.0, 3.0 };
    std::shared_ptr<Pinhole> pinhole = std::make_shared<Pinhole>(focal_length, principal_point, skew, image_size, rotation, translation);
    std::shared_ptr<Camera> camera = std::static_pointer_cast<Camera>(pinhole);

    std::vector<std::array<double, 3>> cameraPoints = { {{0.0, 0.0, 0.0}} };
    std::vector<std::array<double, 3>> worldPoints = camera->cameraToWorldPnts(cameraPoints);

    CameraTest::CompareArray(worldPoints[0],
        {
        -1.638906889681027,
        -2.526221499972754,
        -2.220853245946583
        }, 1e-6);
}