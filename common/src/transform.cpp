#include "transform.hpp"

#include "util/matrix.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>

namespace
{
    constexpr auto const auto_robot_axle_radius{.175};
    constexpr auto const auto_robot_rotate_velocity_diff_min{auto_robot_axle_radius / 4.};
    constexpr auto const auto_robot_self_rotate_radius_min{auto_robot_axle_radius / 4.};
    constexpr auto const auto_robot_max_velocity{1.8};
    constexpr auto const auto_robot_max_angular_velocity{math::tau};
    constexpr auto const auto_robot_position_tolerance{1. / 8.};
    constexpr auto calc_linear_angular_velocities [[nodiscard]] (double left_v, double right_v) noexcept -> std::tuple<double, double, double>
    {
        // https://math.stackexchange.com/a/3680738, https://stackoverflow.com/a/55810955

        auto const v_diff{right_v - left_v}, vv{left_v + v_diff / 2.};
        if (std::abs(v_diff) < auto_robot_rotate_velocity_diff_min)
        {
            return {vv, 0., NAN};
        }
        auto const rad_to_left{(left_v + right_v) * auto_robot_axle_radius / v_diff};
        if (std::abs(rad_to_left) < auto_robot_self_rotate_radius_min)
        {
            return {vv, v_diff / 2. / auto_robot_axle_radius, 0.};
        }
        return {vv, vv / rad_to_left, rad_to_left};
    }
    constexpr auto calc_motor_velocities [[nodiscard]] (double target_lin_v, double target_ang_v) noexcept -> std::tuple<double, double>
    {
        target_ang_v = std::clamp(target_ang_v, -auto_robot_max_angular_velocity, auto_robot_max_angular_velocity);
        auto const extra_v{target_ang_v * auto_robot_axle_radius};
        target_lin_v = std::clamp(target_lin_v, -auto_robot_max_velocity + extra_v, auto_robot_max_velocity - extra_v);
        return {target_lin_v - extra_v, target_lin_v + extra_v};
    }
}

PositionADRC::PositionADRC(decltype(m_position) position, decltype(m_velocity) velocity, decltype(m_gain) gain, double convergence) noexcept
    : m_position{position}, m_velocity{velocity}, m_gain{gain}, m_control{gain, convergence, {position}}
{
}

auto PositionADRC::update(decltype(m_position) target, decltype(m_velocity) velocity, double dt) noexcept -> decltype(m_velocity)
{
    m_position += m_gain * m_velocity * dt;
    m_velocity = velocity;
    auto const input{m_control.update(target, m_velocity, m_position, dt)};
    return input;
}

AutoRobotADRC::AutoRobotADRC(decltype(m_position) position, double rotation, decltype(m_velocities) velocities, decltype(m_gain) gain, double convergence) noexcept
    : m_position{position},
      m_rotation{math::rotation_matrix2(rotation)},
      m_velocities{std::move(velocities)},
      m_gain{gain},
      m_position_control{1., convergence, {0.}},
      m_rotation_control{1., adrc_rotation_convergence_factor * convergence, {0.}}
{
}

auto AutoRobotADRC::update(decltype(m_position) target, double target_rot, decltype(m_velocities) const &velocities, double dt) noexcept -> decltype(m_velocities)
{
    auto [left_v, right_v]{m_velocities};
    left_v *= m_gain;
    right_v *= m_gain;

    auto [lin_v, ang_v, rad]{calc_linear_angular_velocities(left_v, right_v)};
    m_position += lin_v * dt;
    if (!std::isnan(rad))
    {
        auto const rot_mat{math::rotation_matrix2(ang_v * dt)};
        m_rotation = math::orthogonalize_rotation_matrix2(rot_mat * m_rotation);
    }

    m_velocities = velocities;

    target_rot = std::fmod(target_rot + math::tau / 4., math::tau);
    if (target_rot < 0.)
    {
        target_rot += math::tau;
    }

    auto const forward_unit{m_rotation * math::Vector<double, 2>{0., 1.}};
    auto ang_diff{target_rot - std::atan2(forward_unit(1), forward_unit(0))};
    if (ang_diff > math::pi)
    {
        ang_diff -= math::tau;
    }
    else if (ang_diff < -math::pi)
    {
        ang_diff += math::tau;
    }

    lin_v = m_position_control.update(0., lin_v, m_position - target, dt);
    ang_v = m_rotation_control.update(0., ang_v, std::isnan(ang_diff) ? 0. : -ang_diff, dt);

    std::tie(left_v, right_v) = calc_motor_velocities(lin_v, ang_v);
    return {left_v / m_gain, right_v / m_gain};
}

AutoRobotTestADRC::AutoRobotTestADRC(decltype(m_position) position, double rotation, decltype(m_velocities) velocities, decltype(m_gain) gain, double convergence) noexcept
    : m_position{std::move(position)},
      m_rotation{math::rotation_matrix2(rotation)},
      m_velocities{std::move(velocities)},
      m_gain{gain},
      m_position_control{1., convergence, {0.}},
      m_rotation_control{1., adrc_rotation_convergence_factor * convergence, {0.}}
{
}

auto AutoRobotTestADRC::update(decltype(m_position) const &target, std::optional<double> target_rot, decltype(m_velocities) const &velocities, double dt) noexcept -> decltype(m_velocities)
{
    auto [left_v, right_v]{m_velocities};
    left_v *= m_gain;
    right_v *= m_gain;

    auto [lin_v, ang_v, rad]{calc_linear_angular_velocities(left_v, right_v)};
    if (std::isnan(rad))
    {
        m_position += m_rotation * decltype(m_position){0., lin_v * dt};
    }
    else
    {
        auto const rot_mat{math::rotation_matrix2(ang_v * dt)};
        m_position += (rot_mat - math::identity<double, 2>)*m_rotation * decltype(m_position){rad, 0.};
        m_rotation = math::orthogonalize_rotation_matrix2(rot_mat * m_rotation);
    }

    m_velocities = velocities;

    auto const forward_unit{m_rotation * decltype(m_position){0., 1.}};
    auto const pos_diff{target - m_position};
    auto const cur_ang{std::atan2(forward_unit(1), forward_unit(0))};
    auto ang_diff{std::atan2(pos_diff(1), pos_diff(0)) - cur_ang};
    if (math::magnitude(pos_diff) <= auto_robot_position_tolerance)
    {
        if (target_rot)
        {
            auto tar_ang{std::fmod(*target_rot + math::tau / 4., math::tau)};
            if (tar_ang < 0.)
            {
                tar_ang += math::tau;
            }
            ang_diff = tar_ang - cur_ang;
        }
        else
        {
            ang_diff = 0.;
        }
    }
    if (ang_diff > math::pi)
    {
        ang_diff -= math::tau;
    }
    else if (ang_diff < -math::pi)
    {
        ang_diff += math::tau;
    }

    lin_v = m_position_control.update(0., lin_v, -math::dot_product(forward_unit, pos_diff) * std::max(0., std::cos(ang_diff)), dt);
    ang_v = m_rotation_control.update(0., ang_v, std::isnan(ang_diff) ? 0. : -ang_diff, dt);

    std::tie(left_v, right_v) = calc_motor_velocities(lin_v, ang_v);
    return {left_v / m_gain, right_v / m_gain};
}
