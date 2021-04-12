#include "Trajectory.hpp"

void Trajectory::next() {
    advance(1);
}

void Trajectory::prev() {
    advance(-1);
}

void Trajectory::advance(int delta) {
    const int mod = size();
    index = ((static_cast<int>(index) + delta) % mod + mod) % mod;
}

Position& Trajectory::get() {
    return at(index);
}

Position const& Trajectory::get() const {
    return at(index);
}