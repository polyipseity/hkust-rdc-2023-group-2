#pragma once

#include <tuple>

#include "util/adrc.hpp"
#include "util/matrix.hpp"

/**
 * @brief An ADRC controller to control position for testing purpose
 */
class PositionADRC
{
public:
    /**
     * @brief Current position in meters
     */
    double m_position;

    /**
     * @brief Current velocity reported by the motor
     */
    double m_velocity;

private:
    /**
     * @brief Circumference of the wheel in meters
     */
    double m_gain;

    /**
     * @brief Controller for the motor
     */
    control::ADRC2d m_control;

public:
    /**
     * @brief Construct a new `PositionADRC` object
     *
     * @param position current position in meters
     * @param velocity current velocity reported by the motor
     * @param gain circumference of the wheel in meters
     * @param convergence control reactiveness
     */
    PositionADRC(decltype(m_position) position, decltype(m_velocity) velocity, decltype(m_gain) gain = 1., double convergence = 16.) noexcept;

    /**
     * @brief Update the current state and recommend a velocity for the motor
     *
     * @param target desired position
     * @param velocity current velocity reported by the motor
     * @param dt time since last update
     * @return a recommended motor velocity
     */
    auto update(decltype(m_position) target, decltype(m_velocity) velocity, double dt) noexcept -> decltype(m_velocity);
};

/**
 * @brief An ADRC controller to control auto robot
 */
class AutoRobotADRC
{
public:
    /**
     * @brief Current position represented by (x, y) in meters
     */
    math::Vector<double, 2> m_position;

    /**
     * @brief Current rotation represented by a 2D rotation matrix
     */
    math::SquareMatrix<double, 2> m_rotation;

    /**
     * @brief Current velocities reported by the two motors
     */
    std::tuple<double, double> m_velocities;

private:
    /**
     * @brief Circumference of the wheels in meters
     */
    double m_gain;

    /**
     * @brief Controller for position
     */
    control::ADRC2d m_position_control;

    /**
     * @brief Controller for rotation
     */
    control::ADRC2d m_rotation_control;

public:
    /**
     * @brief Construct a new `AutoRobotADRC` object
     *
     * @param position current position in meters
     * @param rotation current rotation in radians
     * @param velocities current velocities reported by the two motors
     * @param gain circumference of the wheels in meters
     * @param convergence control reactiveness
     */
    AutoRobotADRC(decltype(m_position) position, double rotation, decltype(m_velocities) velocities, decltype(m_gain) gain = 1., double convergence = 16.) noexcept;

    /**
     * @brief Update the current state and recommend velocities for the two motors
     *
     * @param target desired position
     * @param velocities current velocities reported by the two motors
     * @param dt time since last update
     * @return recommended motor velocities for the two motors
     */
    auto update(decltype(m_position) const &target, decltype(m_velocities) const &velocities, double dt) noexcept -> decltype(m_velocities);
};
