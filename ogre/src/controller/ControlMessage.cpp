#include "ControlMessage.hpp"
#include <sstream>
#include <yaml-cpp/yaml.h>

ControlMessage ControlMessage::fromYAML(std::string const& string) {
    YAML::Node yaml = YAML::Load(string);
    ControlMessage controlMessage;
    if (yaml["position"])
        controlMessage.position = yaml["position"].as<std::vector<double>>();
    if (yaml["cameraName"])
        controlMessage.cameraName = yaml["cameraName"].as<std::string>();
    return controlMessage;
}

std::string ControlMessage::toYAML() const {
    YAML::Node yaml;
    yaml["position"] = position.toVector();
    yaml["cameraName"] = cameraName;

    std::stringstream ss;
    ss << yaml;
    return ss.str();
}