#include <vector>
#include <string>



bool isClose(double a, double b, double epsilon);

// helper functions 
template <size_t N> //allows for world and image pnt size (2 or 3)
bool compareArrays(const std::vector<std::array<double, N>>& array1,
    const std::vector<std::array<double, N>>& array2) {

    // should be within 10 um for 3d coords and 0.2 pixel for pixel coords
    double epsilon = (N == 3) ? 1e-4 : 0.1;



    return std::equal(array1.begin(), array1.end(), array2.begin(), array2.end(),
        [epsilon](const std::array<double, N>& a, const std::array<double, N>& b) {

            for (size_t i = 0; i < N; ++i) {
                if (!isClose(a[i], b[i], epsilon)) {
                    std::cout << "difference of " << a[i] - b[i] << " found at index " << i << " larger than: " << epsilon << std::endl;
                    return false;

                }
            }
            return true;
        });
}


bool extractJsonArrays(
    const std::string& jsonFilePath,
    std::vector<std::array<double, 2>>& projected_array,
    std::vector<std::array<double, 3>>& backprojected_array,
    std::vector<std::array<double, 2>>& input_array,
    std::vector<std::array<double, 3>>* transformed_array = nullptr // Optional parameter
);
