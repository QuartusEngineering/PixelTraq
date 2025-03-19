#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <stdexcept>
#include <string>

class Utils {
public:
    static std::vector<std::vector<std::vector<double>>> loadImage(const std::string& filename, int desiredChannels = 1);
    static void saveImage(const std::vector<std::vector<std::vector<double>>>& data, const std::string& filename, const std::string& format);
    static void saveImage(const std::vector<std::vector<std::vector<double>>>& data, const std::string& filename);
    static bool exists(const std::string& name);
};

#endif // UTILS_H