#include "position.hpp"

PositionADRC::PositionADRC(decltype(m_position) position, decltype(m_velocity) velocity, decltype(m_gain) gain, double convergence) noexcept
    : m_position{position}, m_velocity{velocity}, m_gain{gain}, m_control{gain, convergence, {position}}
{
}

auto PositionADRC::update(double target, double velocity, double dt) noexcept -> double
{
    m_position += m_gain * m_velocity * dt;
    m_velocity = velocity;
    auto const input{m_control.update(target, m_velocity, m_position, dt)};
    return input;
}
