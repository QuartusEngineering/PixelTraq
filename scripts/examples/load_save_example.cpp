#include <iostream>
#include <stdexcept>
#include "pixeltraq.h"

int main() {

    try {
        
        // models can be loaded directly from their class
        auto model1 = Kannala::load(std::string(DATA_DIR) + "/H2395912_CameraModel.json");
        //auto model1 = Kannala::load("output.json");

        // or from the camera class
        auto model2 = Camera::load(std::string(DATA_DIR) + "/H2395912_CameraModel.json");

        // to covert the generic class to the Kannala class
        auto model3 = std::static_pointer_cast<Kannala>(model2);

        // to save the model call save. The output type is determined by the file type of the string
        model1->save("output.json");
    }
    catch (const std::exception& e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}