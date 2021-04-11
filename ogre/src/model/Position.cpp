#include "Position.hpp"
#include <exception>

Position::Position(double x, double y, double z, double pitch, double yaw, double roll)
    : x(x), y(y), z(z), pitch(pitch), yaw(yaw), roll(roll) {
}

Position::Position(std::vector<double> const& values) {
    if (values.size() != 6)
        throw std::invalid_argument(
            "Position can only be initialized with 6 values");
    x = values[0];
    y = values[1];
    z = values[2];
    pitch = values[3];
    yaw = values[4];
    roll = values[5];
}

std::istream& operator>>(std::istream& is, Position& position) {
    is >> position.x >> position.y >> position.z;
    double pitch, yaw, roll;
    is >> pitch >> yaw >> roll;
    position.pitch = pitch, position.yaw = yaw, position.roll = roll;
    return is;
}