#pragma once

#include <tuple>

#include "util/adrc.hpp"
#include "util/matrix.hpp"

class PositionADRC
{
public:
    double m_position, m_velocity;

private:
    double m_gain;
    control::ADRC2d m_control;

public:
    PositionADRC(decltype(m_position) position, decltype(m_velocity) velocity, decltype(m_gain) gain = 1., double convergence = 16.) noexcept;
    auto update(decltype(m_position) target, decltype(m_velocity) velocity, double dt) noexcept -> decltype(m_velocity);
};

class AutoRobotADRC
{
public:
    math::Vector<double, 2> m_position;
    math::SquareMatrix<double, 2> m_rotation;
    std::tuple<double, double> m_velocities;

private:
    double m_gain;
    control::ADRC2d m_position_control, m_rotation_control;

public:
    AutoRobotADRC(decltype(m_position) position, double rotation, decltype(m_velocities) velocities, decltype(m_gain) gain = 1., double convergence = 16.) noexcept;
    auto update(decltype(m_position) const &target, decltype(m_velocities) const &velocities, double dt) noexcept -> decltype(m_velocities);
};
