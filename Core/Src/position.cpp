#include "position.hpp"

PositionADRC::PositionADRC(decltype(position) position, decltype(velocity) velocity, double convergence) noexcept
    : position{position}, velocity{velocity}, adrc{1., convergence, {position}}
{
}

auto PositionADRC::update(double target, double velocity, double dt) noexcept -> double
{
    position += velocity * dt;
    auto const input{adrc.update(target, this->velocity, position, dt)};
    this->velocity = velocity;
    return input;
}
