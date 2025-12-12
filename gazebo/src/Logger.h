#pragma once
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <mutex>
#include <Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>

class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void startNewSession() {
        std::lock_guard<std::mutex> lock(mtx);
        // Close existing files
        for (auto& pair : files) {
            if (pair.second.is_open()) pair.second.close();
        }
        files.clear();

        // Create new directory based on time
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << "logs/" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
        session_dir = oss.str();
        
        std::filesystem::create_directories(session_dir);
        std::cout << "[Logger] Started new session: " << session_dir << std::endl;
    }

    template <typename Derived>
    void log(const std::string& name, const Eigen::MatrixBase<Derived>& data, uint64_t timestamp) {
        std::lock_guard<std::mutex> lock(mtx);
        ensureFileOpen(name);
        files[name] << timestamp;
        
        if (data.size() > 0) {
            static const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",");
            files[name] << "," << data.format(CSVFormat);
        }
        files[name] << "\n";
    }

private:
    Logger() {
        startNewSession();
    }
    
    ~Logger() {
        for (auto& pair : files) {
            if (pair.second.is_open()) pair.second.close();
        }
    }

    void ensureFileOpen(const std::string& name) {
        if (files.find(name) == files.end()) {
            std::string path = session_dir  +"/"+  name  +".csv";
            files[name].open(path);
            if (!files[name].is_open()) {
                std::cerr << "[Logger] Failed to open file: " << path << std::endl;
            }
        }
    }

    std::string session_dir;
    std::map<std::string, std::ofstream> files;
    std::mutex mtx;
};
