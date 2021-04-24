#pragma once
#include "../model/Position.hpp"
#include "Trajectory.hpp"
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

struct CameraInfo {
    std::string name = "";
    int width;
    int height;
    std::vector<double> D;
    std::vector<double> K;
    std::vector<double> R;
    std::vector<double> P;
};

struct Config {
    Trajectory trajectory;
    Position position;
    std::filesystem::path outputDir = "output";
    std::optional<CameraInfo> cameraInfo;
};

Config argParse(int argc, char* argv[]);
Trajectory getTrajectory(std::string const&);
CameraInfo getCameraInfo(std::string const&);