#pragma once

#include <optional>
#include <tuple>

#include "util/adrc.hpp"
#include "util/matrix.hpp"

/**
 * @brief Ratio of convergence rate of rotation over translation
 */
constexpr auto const adrc_rotation_convergence_factor{4.};

/**
 * @brief An ADRC controller to control position
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

    /**
     * @brief Position ber bearing turn
     */
    double m_gain;

private:
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
     * @param gain position per bearing turn
     * @param convergence control sensitivity
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
 * @brief An ADRC controller to control the auto robot
 */
class AutoRobotADRC
{
public:
    /**
     * @brief Current position, useful for relative calculations only
     */
    double m_position;

    /**
     * @brief Current rotation represented by a 2D rotation matrix, useful for relative calculations only
     */
    math::SquareMatrix<double, 2> m_rotation;

    /**
     * @brief Current velocities reported by the two motors
     */
    std::tuple<double, double> m_velocities;

    /**
     * @brief Meters per bearing turn
     */
    double m_gain;

private:
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
     * @param gain meters per bearing turn
     * @param convergence control sensitivity
     */
    AutoRobotADRC(decltype(m_position) position, double rotation, decltype(m_velocities) velocities, decltype(m_gain) gain = .068 * math::pi * 23. / 30. / 25., double convergence = 16.) noexcept;

    /**
     * @brief Update the current state and recommend velocities for the two motors
     *
     * @param target desired position
     * @param target_rot desired rotation
     * @param velocities current velocities reported by the two motors
     * @param dt time since last update
     * @param max_velocities max linear and angular velocity
     * @return recommended motor velocities for the two motors
     */
    auto update(decltype(m_position) target, double target_rot, decltype(m_velocities) const &velocities, double dt, std::tuple<double, double> const &max_velocities) noexcept -> decltype(m_velocities);
};

/**
 * @brief A testing ADRC controller to control auto robot
 */
class AutoRobotTestADRC
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

    /**
     * @brief Meters per bearing turn
     */
    double m_gain;

private:
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
     * @brief Construct a new `AutoRobotTestADRC` object
     *
     * @param position current position in meters
     * @param rotation current rotation in radians
     * @param velocities current velocities reported by the two motors
     * @param gain meters per bearing turn
     * @param convergence control sensitivity
     */
    AutoRobotTestADRC(decltype(m_position) position, double rotation, decltype(m_velocities) velocities, decltype(m_gain) gain = .068 * math::pi * 23. / 30. / 25., double convergence = 16.) noexcept;

    /**
     * @brief Update the current state and recommend velocities for the two motors
     *
     * @param target desired position
     * @param target_rot desired rotation
     * @param velocities current velocities reported by the two motors
     * @param dt time since last update
     * @param max_velocities max linear and angular velocity
     * @return recommended motor velocities for the two motors
     */
    auto update(decltype(m_position) const &target, std::optional<double> target_rot, decltype(m_velocities) const &velocities, double dt, std::tuple<double, double> const &max_velocities) noexcept -> decltype(m_velocities);
};

/**
 * @brief An ADRC controller to control the task robot
 */
class TaskRobotADRC
{
public:
    /**
     * @brief Current position represented by (x, y) in meters, useful for relative calculations only
     */
    math::Vector<double, 2> m_position;

    /**
     * @brief Current rotation represented by a 2D rotation matrix, useful for relative calculations only
     */
    math::SquareMatrix<double, 2> m_rotation;

    /**
     * @brief Current velocities reported by the four motors
     */
    math::Vector<double, 4> m_velocities;

    /**
     * @brief Meters per bearing turn
     */
    double m_gain;

private:
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
     * @brief Construct a new `TaskRobotADRC` object
     *
     * @param position current position in meters
     * @param rotation current rotation in radians
     * @param velocities current velocities reported by the four motors
     * @param gain meters per bearing turn
     * @param convergence control sensitivity
     */
    TaskRobotADRC(decltype(m_position) position, double rotation, decltype(m_velocities) velocities, decltype(m_gain) gain = .15 * math::pi * 16.5 / 30. / 10., double convergence = 1.) noexcept;

    /**
     * @brief Update the current state and recommend velocities for the four motors
     *
     * @param target desired position
     * @param target_rot desired rotation
     * @param velocities current velocities reported by the two motors
     * @param dt time since last update
     * @return recommended motor velocities for the two motors
     */
    auto update(decltype(m_position) const &target, double target_rot, decltype(m_velocities) const &velocities, double dt) noexcept -> decltype(m_velocities);
};
