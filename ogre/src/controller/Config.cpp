#include "Config.hpp"
#include "../../ext/clipp/clipp.h"
#include <exception>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>

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

    if (!cameraFile.empty())
        config.cameraInfo = getCameraInfo(cameraFile);

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

CameraInfo getCameraInfo(std::string const& filename) {
    YAML::Node cameraYAML = YAML::LoadFile(filename);
    CameraInfo cameraInfo;

    if (cameraYAML["height"])
        cameraInfo.height = cameraYAML["height"].as<int>();
    else if (cameraYAML["image_height"])
        cameraInfo.height = cameraYAML["image_height"].as<int>();

    if (cameraYAML["width"])
        cameraInfo.width = cameraYAML["width"].as<int>();
    else if (cameraYAML["image_width"])
        cameraInfo.width = cameraYAML["image_width"].as<int>();

    if (cameraYAML["camera_name"])
        cameraInfo.name = cameraYAML["camera_name"].as<std::string>();

    if (cameraYAML["D"])
        cameraInfo.D = cameraYAML["D"].as<std::vector<double>>();
    else if (cameraYAML["distortion_coefficients"])
        cameraInfo.D =
            cameraYAML["distortion_coefficients"]["data"].as<std::vector<double>>();

    if (cameraYAML["K"])
        cameraInfo.K = cameraYAML["K"].as<std::vector<double>>();
    else if (cameraYAML["camera_matrix"])
        cameraInfo.K = cameraYAML["camera_matrix"]["data"].as<std::vector<double>>();

    if (cameraYAML["R"])
        cameraInfo.R = cameraYAML["R"].as<std::vector<double>>();
    else if (cameraYAML["rectification_matrix"])
        cameraInfo.R =
            cameraYAML["rectification_matrix"]["data"].as<std::vector<double>>();

    if (cameraYAML["P"])
        cameraInfo.P = cameraYAML["P"].as<std::vector<double>>();
    else if (cameraYAML["projection_matrix"])
        cameraInfo.P =
            cameraYAML["projection_matrix"]["data"].as<std::vector<double>>();

    return cameraInfo;
}