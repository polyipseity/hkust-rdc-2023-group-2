#pragma once

#include "util/adrc.hpp"

class PositionADRC
{
public:
    double m_position, m_velocity, m_gain;

private:
    control::ADRC2d m_control;

public:
    PositionADRC(decltype(m_position) position, decltype(m_velocity) velocity, decltype(m_gain) gain = 1., double convergence = 16.) noexcept;
    auto update(double target, double velocity, double dt) noexcept -> double;
};
