#include "Config.hpp"
#include "../../ext/clipp/clipp.h"
#include <camera_calibration_parsers/parse.h>
#include <exception>
#include <fstream>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>

Config argParse(int argc, char* argv[]) {
    using namespace clipp;
    enum class mode { trajectory, position, help };

    mode selectedMode = mode::help;
    Config config;
    std::string trajectoryFile = "";
    std::string outputDir = "";
    std::string cameraFile = "";
    std::vector<double> positionValues;

    auto trajectory = (option("-t").set(selectedMode, mode::trajectory) &
                       value("trajectory", trajectoryFile)) %
                      "camera trajectory";
    auto position = (option("-p").set(selectedMode, mode::position) &
                     values("position", positionValues)) %
                    "initial camera position";
    auto output =
        (option("-o") & value("output", outputDir)) % "output directory";
    auto camera = (option("-c") & value("camera", cameraFile)) % "camera info";
    auto help = option("-h").set(selectedMode, mode::help) % "show help";

    auto cli = ((trajectory | position, output, camera) | help);

    if (!parse(argc, argv, cli)) {
        std::cout << make_man_page(cli, argv[0]);
        exit(1);
    }

    switch (selectedMode) {
    case mode::help:
        std::cout << make_man_page(cli, argv[0]);
        exit(0);
    case mode::trajectory:
        config.trajectory = getTrajectory(trajectoryFile);
        if (config.trajectory.empty())
            throw std::invalid_argument("Empty trajectory");
        config.position = config.trajectory.front();
        break;
    case mode::position:
        config.position = positionValues;
        config.trajectory = {config.position};
        break;
    }

    if (!outputDir.empty())
        config.outputDir = outputDir;

    if (!cameraFile.empty()) {
        config.cameraInfo = sensor_msgs::CameraInfo{};
        std::string cameraName;
        camera_calibration_parsers::readCalibration(cameraFile, cameraName,
                                                    *config.cameraInfo);
    }

    return config;
}

Trajectory getTrajectory(std::string const& filename) {
    std::ifstream fin(filename);
    Trajectory trajectory;
    Position position;
    while (fin >> position) {
        trajectory.push_back(position);
    }
    return trajectory;
}