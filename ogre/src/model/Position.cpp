#include "Position.hpp"

Position::Position(double x, double y, double z, double pitch, double yaw, double roll)
    : x(x), y(y), z(z), pitch(pitch), yaw(yaw), roll(roll) {
}

std::istream& operator>>(std::istream& is, Position& position) {
    is >> position.x >> position.y >> position.z;
    double pitch, yaw, roll;
    is >> pitch >> yaw >> roll;
    position.pitch = pitch, position.yaw = yaw, position.roll = roll;
    return is;
}