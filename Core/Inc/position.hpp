#pragma once

#include "util/adrc.hpp"

class PositionADRC
{
public:
    double position, velocity;

private:
    control::ADRC2d adrc;

public:
    PositionADRC(decltype(position) position, decltype(velocity) velocity, double convergence = 1.) noexcept;
    auto update(double target, double velocity, double dt) noexcept -> double;
};
