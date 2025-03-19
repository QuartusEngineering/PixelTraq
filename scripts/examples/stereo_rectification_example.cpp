#include <iostream>
#include <stdexcept>
#include "pixeltraq.h"

int main() 
{
	try
	{
		// Load in the two cameras for the stereo module
		auto modelL = Kannala::load(std::string(DATA_DIR) + "/H2387815_CameraModel.json");
		auto modelR = Kannala::load(std::string(DATA_DIR) + "/H2395912_CameraModel.json");

		// Display the models
		std::cout << "Left Camera Model:" << std::endl;
		modelL->display();
		std::cout << std::endl << "Right Camera Model:" << std::endl;
		modelR->display();

		// load in the images
		auto imageL = Utils::loadImage(std::string(DATA_DIR) + "/H2387815.png");
		auto imageR = Utils::loadImage(std::string(DATA_DIR) + "/H2395912.png");

		// compute rectification rotation
		Matrix3x3 RrectL{ {{1,0,0}, { 0,1,0 }, { 0,0,1 }} };
		Matrix3x3 RrectR{ {{1,0,0}, { 0,1,0 }, { 0,0,1 }} };

		// get the nominal pinhole models to send into the rectify method
		auto pinholeL = modelL->getPinhole();
		auto pinholeR = modelR->getPinhole();

		// this method computes the normalized pinhole models and the rotations for rectification
		CommonMath::stereoRectify(pinholeL, pinholeR, RrectL, RrectR);

		// create remappers with 3 argument constructor for rectification matrix
		auto remapperL = std::make_shared<Remapper>(modelL, pinholeL, RrectL);
		auto remapperR = std::make_shared<Remapper>(modelR, pinholeR, RrectR);

		// undistort images (wrap this section in a loop for streaming applications)
		auto imageL_rect = remapperL->undistort(imageL);
		auto imageR_rect = remapperR->undistort(imageR);

		// save out rectified images
		Utils::saveImage(imageL_rect, "imageLRect.png");
		Utils::saveImage(imageR_rect, "imageRRect.png");
	}
	catch (const std::exception& e) {
		std::cerr << "An error occurred: " << e.what() << std::endl;
		return 1;
	}
}