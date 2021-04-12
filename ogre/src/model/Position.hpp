#pragma once
#include "Ogre.h"
#include <istream>
#include <string>
#include <vector>

enum class Direction { FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN };

struct Position {
    Ogre::Real x;
    Ogre::Real y;
    Ogre::Real z;

    Ogre::Degree pitch;
    Ogre::Degree yaw;
    Ogre::Degree roll;

    Position() = default;
    Position(double, double, double, double, double, double);
    Position(std::vector<double> const&);
    friend std::istream& operator>>(std::istream&, Position&);
    std::string toString() const;
};