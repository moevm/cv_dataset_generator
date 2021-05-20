#include "Position.hpp"
#include <exception>
#include <iomanip>
#include <sstream>

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

std::string Position::toString() const {
    std::stringstream ss;
    ss << std::setprecision(2) << std::fixed;
    ss << std::fixed << x << '_' << y << '_' << z << '_' << pitch.valueDegrees()
       << '_' << yaw.valueDegrees() << '_' << roll.valueDegrees();
    return ss.str();
}

std::string Position::displayString() const {
    std::stringstream ss;
    ss << std::setprecision(1) << std::fixed;
    ss << "XYZ   " << x << ' ' << y << ' ' << z << std::endl;
    ss << "Pitch " << std::fixed << pitch.valueDegrees() << std::endl;
    ss << "Yaw   " << std::fixed << yaw.valueDegrees() << std::endl;
    ss << "Roll  " << std::fixed << roll.valueDegrees();
    return ss.str();
}