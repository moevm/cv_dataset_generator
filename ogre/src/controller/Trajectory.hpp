#pragma once
#include "../model/Position.hpp"
#include <vector>

class Trajectory : public std::vector<Position> {
public:
    using vector = std::vector<Position>;
    using vector::vector;

    void next();
    void prev();
    void advance(int);

    Position& get();
    Position const& get() const;

private:
    std::size_t index = 0;
};