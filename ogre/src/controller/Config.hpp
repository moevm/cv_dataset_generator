#pragma once
#include "../model/Position.hpp"
#include "Trajectory.hpp"
#include <filesystem>
#include <optional>
#include <sensor_msgs/CameraInfo.h>
#include <string>
#include <vector>

struct Config {
    Trajectory trajectory;
    Position position;
    std::filesystem::path outputDir = "output";
    std::optional<sensor_msgs::CameraInfo> cameraInfo;
};

Config argParse(int argc, char* argv[]);
Trajectory getTrajectory(std::string const&);