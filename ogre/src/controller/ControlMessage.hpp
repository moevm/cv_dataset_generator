#pragma once
#include "../model/Position.hpp"
#include <string>

struct ControlMessage {
    Position position;
    std::string cameraName = "";

    static ControlMessage fromYAML(std::string const&);
    std::string toYAML() const;
};
