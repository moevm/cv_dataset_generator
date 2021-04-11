#pragma once
#include "../model/Position.hpp"
#include <filesystem>
#include <string>
#include <vector>

struct Config {
    std::vector<Position> trajectory;
    Position position;
    std::filesystem::path outputDir = "output";
};

Config argParse(int argc, char* argv[]);
std::vector<Position> getTrajectory(std::string const&);