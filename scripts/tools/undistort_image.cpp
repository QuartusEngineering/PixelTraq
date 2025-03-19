#include <iostream>
#include <stdexcept>
#include "pixeltraq.h"

int main(int argc, char* argv[]) {

    // Check if the number of command-line arguments is correct
    if (argc < 4 || argc > 5) {
        std::cerr << "Usage: " << argv[0] << " <input_image> <output_image> <input_model> [output_model]" << std::endl;
        return 1;
    }

    try {
        // Required arguments
        std::string input_image_path = argv[1];
        std::string output_image_path = argv[2];
        std::string input_model_path = argv[3];

        // Optional argument
        std::string output_model_path;
        if (argc == 5) {
            output_model_path = argv[4];
        }

        // Print the received inputs
        std::cout << "Input Image: " << input_image_path << std::endl;
        std::cout << "Output Image: " << output_image_path << std::endl;
        std::cout << "Input Model: " << input_model_path << std::endl;
        std::shared_ptr<Camera> input_model;
        std::shared_ptr<Camera> output_model;
        if (!output_model_path.empty()) {
            std::cout << "Output Model: " << output_model_path << std::endl << std::endl;
            input_model = Camera::load(input_model_path);
            output_model = Camera::load(output_model_path);
            std::cout << "Input Model:" << std::endl;
            input_model->display();
            std::cout << std::endl << "Output Model:" << std::endl;
            output_model->display();
            std::cout << std::endl;
        }
        else {
            std::cout << "Output Model: (not provided)" << std::endl << std::endl;
            input_model = Camera::load(input_model_path);
            std::cout << "Input Model:" << std::endl;
            input_model->display();
            std::cout << std::endl;
        }

        auto input_image = Utils::loadImage(input_image_path);

        // Core functionality
        std::shared_ptr<Remapper> remapper;
        if (!output_model_path.empty()) {
            // In this case, we are creating a two argument remapper mapping from the input to output model
            remapper = std::make_shared<Remapper>(input_model,output_model);
        }
        else {
            // In this case, we are creating a one argument remapper mapping from the input model to a Pinhole of the same focal length
            remapper = std::make_shared<Remapper>(input_model);
        };

        auto output_image = remapper->undistort(input_image);

        Utils::saveImage(output_image, output_image_path);
    }
    catch (const std::exception& e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}