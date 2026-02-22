#include "Logger.h"
#include "timing.h"
#include <iostream>
#include <fstream>
#include "QuadModel.h"
#include "Estimator.h"


uint64_t g_current_time;
uint64_t getCurrentTimeUs()
{
    return g_current_time;
}

std::vector<float> parseLineToFloats(const std::string& line) {
    std::vector<float> row;
    std::stringstream ss(line);
    std::string value;

    // Use comma as the delimiter
    while (std::getline(ss, value, ',')) {
        try {
            // Remove potential leading/trailing whitespace
            value.erase(0, value.find_first_not_of(" \t\r\n"));
            value.erase(value.find_last_not_of(" \t\r\n") + 1);

            if (!value.empty()) {
                row.push_back(std::stof(value));
            }
        } catch (const std::invalid_argument& e) {
            // Skips cells that aren't numbers (like "N/A" or headers)
            std::cerr << "Warning: Could not convert '" << value << "' to float." << std::endl;
        }
    }
    return row;
}

#include <cmath> // Required for std::round

int roundToNearest100(int n) {
    // 1. Divide by 100.0 to perform floating-point division
    double divided = static_cast<double>(n) / 100.0;
    
    // 2. Round the result to the nearest integer using std::round
    // Example: 123.45 becomes 123.0, 123.50 becomes 124.0
    double rounded = std::round(divided); //
    
    // 3. Multiply by 100 to get the final rounded value
    int result = static_cast<int>(rounded * 100.0);
    
    return result;
}

int main(int argc, char *argv[])
{
    QuadcopterModel model;
    Estimator m_estimator(model);

    std::ifstream omega("omega.csv");
    std::ifstream imu_acc("imu_acc.csv");
    std::ifstream imu_gyro("imu_gyro.csv");
    std::ifstream control("control.csv");
    std::string line;
    std::cout << "Starting Replay..." << std::endl;
    while (std::getline(omega, line)){
        std::vector<float> om_in = parseLineToFloats(line);
        g_current_time = roundToNearest100((uint64_t)(om_in[0]* 1e6));
        if (g_current_time==30000){
            std::cout << "time..." << std::endl;
        }
        Vector4f omega_in;
        omega_in<< om_in[1], om_in[2], om_in[3], om_in[4];
        model.omega_sig.update(omega_in, g_current_time);
        std::getline(imu_acc, line);
        std::vector<float> acc_in = parseLineToFloats(line);
        std::getline(imu_gyro, line);
        std::vector<float> gyro_in = parseLineToFloats(line);
        std::getline(control, line);
        std::vector<float> control_in = parseLineToFloats(line);

        Eigen::Matrix<float, 6, 1> imu_vec;
        imu_vec << acc_in[1], acc_in[2], acc_in[3], gyro_in[1], gyro_in[2], gyro_in[3];
        model.imu_sig.update(imu_vec, g_current_time);
        
        Vector4f control_vec;
        control_vec << control_in[1], control_in[2], control_in[3], control_in[4];
        model.control_sig.update(control_vec, g_current_time);

        m_estimator.run();
    }



}
