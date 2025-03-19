#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "external/stb_image/stb_image.h"
#include "external/stb_image/stb_image_write.h"

#include "utilities/common_math.h"
#include "utilities/utils.h"
#include <vector>
#include <string>
#include <array> 
#include <algorithm>
#include <iostream>
#include <sys/stat.h>

/**
 * @brief Clamps a value between a minimum and a maximum value.
 *
 * @tparam T The type of the value.
 * @param val The value to be clamped.
 * @param mn The minimum value.
 * @param mx The maximum value.
 * @return The clamped value.
 */
template <typename T>
T clamp(T val, T mn, T mx) {
    return std::max(mn, std::min(val, mx));
}

/**
 * @brief Loads an image from a file and returns it as a 3D vector.
 *
 * @param filename The path to the image file.
 * @param desiredChannels The number of desired channels in the output image.
 * @return A 3D vector representing the image.
 * @throws std::runtime_error if the image fails to load.
 */
std::vector<std::vector<std::vector<double>>> Utils::loadImage(const std::string& filename, int desiredChannels) {
    int width, height, channels;

    // Load the image data into an unsigned char array with the specified number of channels
    uint8_t* imgData = stbi_load(filename.c_str(), &width, &height, &channels, desiredChannels);
    if (imgData == nullptr) {
        throw std::runtime_error("Failed to load image.");
    }

    // Initialize a 3D vector to hold the image data
    // If grayscale (desiredChannels = 1), the outer vector will have a single channel
    // If RGB (desiredChannels = 3), the outer vector will have three channels
    std::vector<std::vector<std::vector<double>>> imageVector(desiredChannels, std::vector<std::vector<double>>(height, std::vector<double>(width)));

    // Transfer pixel data to the 3D vector
    for (int c = 0; c < desiredChannels; ++c) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // Calculate the index for the desired channel
                imageVector[c][y][x] = static_cast<double>(imgData[(y * width + x) * desiredChannels + c]);
            }
        }
    }

    // Free the image data memory
    stbi_image_free(imgData);

    return imageVector;
}

/**
 * @brief Saves an image to a file in a specified format.
 *
 * @param data The 3D vector representing the image.
 * @param filename The path to the output image file.
 * @param format The format to save the image in (e.g., "bmp", "png", "jpg").
 * @throws std::invalid_argument if the input data has inconsistent dimensions or an unsupported format.
 */
void Utils::saveImage(const std::vector<std::vector<std::vector<double>>>& data, const std::string& filename, const std::string& format) {
    int channels = (int)data.size(); // Infer the number of channels
    if (channels == 0) {
        throw std::invalid_argument("Input data has no channels.");
    }

    int height = (int)data[0].size();
    int width = (int)data[0][0].size();

    for (int c = 0; c < channels; ++c) {
        if (data[c].size() != height || data[c][0].size() != width) {
            throw std::invalid_argument("All channels must have the same dimensions.");
        }
    }

    std::vector<unsigned char> imageBuffer(height * width * channels);
    for (int c = 0; c < channels; ++c) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                imageBuffer[(y * width + x) * channels + c] =
                    static_cast<unsigned char>(clamp(data[c][y][x], 0.0, 255.0));
            }
        }
    }

    // Determine the format and save the image
    bool success = false;
    if (format == "bmp") {
        success = stbi_write_bmp(filename.c_str(), width, height, channels, imageBuffer.data());
    }
    else if (format == "png") {
        int stride_in_bytes = width * channels;
        success = stbi_write_png(filename.c_str(), width, height, channels, imageBuffer.data(), stride_in_bytes);
    }
    else if (format == "jpg" || format == "jpeg") {
        int quality = 100;
        success = stbi_write_jpg(filename.c_str(), width, height, channels, imageBuffer.data(), quality);
    }
    else {
        throw std::invalid_argument("Unsupported image format.");
    }

    if (success) {
        std::cout << "Image saved as " << filename << "\n";
    }
    else {
        std::cerr << "Failed to save image.\n";
    }
}

/**
 * @brief Saves an image to a file in a specified format based on the extension in the filename.
 *
 * @param data The 3D vector representing the image.
 * @param filename The path to the output image file including the extension.
 * @throws std::invalid_argument if the input data has inconsistent dimensions or an unsupported format.
 */
void Utils::saveImage(const std::vector<std::vector<std::vector<double>>>& data, const std::string& filename) {
    std::string::size_type idx = filename.rfind('.');
    if (idx == std::string::npos) {
        throw std::invalid_argument("Filename does not contain an extension.");
    }

    std::string extension = filename.substr(idx + 1);
    if (extension.empty()) {
        throw std::invalid_argument("Filename extension is empty.");
    }

    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

    saveImage(data, filename, extension);
}

/**
 * @brief Checks if a file exists.
 *
 * @param name The path to the file.
 * @return true if the file exists, false otherwise.
 */
bool Utils::exists(const std::string& name) {

    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}
