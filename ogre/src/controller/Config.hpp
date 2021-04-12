#pragma once
#include "../model/Position.hpp"
#include "Trajectory.hpp"
#include <filesystem>
#include <string>
#include <vector>

struct Config {
    Trajectory trajectory;
    Position position;
    std::filesystem::path outputDir = "output";
};

Config argParse(int argc, char* argv[]);
Trajectory getTrajectory(std::string const&);