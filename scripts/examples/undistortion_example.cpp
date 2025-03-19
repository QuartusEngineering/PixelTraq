#include <iostream>
#include <stdexcept>
#include <cmath>
#include "pixeltraq.h"

int main() 
{
    try {
		// Load in the two cameras for the stereo module
		auto model = Kannala::load(std::string(DATA_DIR) + "/H2395912_CameraModel.json");

		// Display the models
		std::cout << "Camera Model:" << std::endl;
		model->display();

		// load in the image
		auto distorted_image = Utils::loadImage(std::string(DATA_DIR) + "/H2395912.png");

		// create remapper with 1 argument constructor
		// this maps to a pinhole camera of the same size image and focal length
		auto remapper1 = std::make_shared<Remapper>(model);

		// undistort image (wrap this section in a loop for streaming applications)
		auto undistorted_image = remapper1->undistort(distorted_image);

		// save out undistorted image
		Utils::saveImage(undistorted_image, "undistorted_image_1.png");

		// get the Pinhole model
		auto pinholeModel = model->getPinhole();

		// Rescale the pinhole model to get the full field of view
		auto current_principal_point = pinholeModel->getPrincipalPoint();
		double scale_factor = 1.3;
		current_principal_point = { scale_factor * current_principal_point[0], scale_factor * current_principal_point[1] };
		pinholeModel->setPrincipalPoint(current_principal_point);
		auto current_image_size = pinholeModel->getImageSize();
		current_image_size = { (int)std::round( scale_factor * current_image_size[0]), (int)std::round( scale_factor * current_image_size[1]) };
		pinholeModel->setImageSize(current_image_size);

		// create remapper with 2 argument constructor
		// this maps to a pinhole camera with rescaled image size and focal length
		auto remapper2 = std::make_shared<Remapper>(model,pinholeModel);

		// undistort image (wrap this section in a loop for streaming applications)
		undistorted_image = remapper2->undistort(distorted_image);

		// save out undistorted image
		Utils::saveImage(undistorted_image, "undistorted_image_2.png");

		// create a rotation matrix from making a rotated remapper
		auto R = CommonMath::eulerToRot({0.1,0.1,0.1});

		// create remapper with 3 argument constructor
		// this maps to a pinhole camera with rescaled image size and focal length and allows for a rotation
		auto remapper3 = std::make_shared<Remapper>(model, pinholeModel,R);

		// undistort image (wrap this section in a loop for streaming applications)
		undistorted_image = remapper3->undistort(distorted_image);

		// save out undistorted image
		Utils::saveImage(undistorted_image, "undistorted_image_3.png");
    }
    catch (const std::exception& e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}