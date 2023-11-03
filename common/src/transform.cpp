#include "transform.hpp"

#include "util/matrix.hpp"

#include <cmath>
#include <tuple>
#include <utility>

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

namespace
{
    constexpr auto const auto_robot_axle_radius{1.};                                 // todo: use real value
    constexpr auto const auto_robot_rotate_velocity_diff_min{.01};                   // todo: find a good value
    constexpr auto const auto_robot_rotate_radius_min{auto_robot_axle_radius / 10.}; // todo: find a good value
    constexpr auto calc_linear_angular_velocities [[nodiscard]] (double left_v, double right_v) noexcept -> std::tuple<double, double, double>
    {
        // https://math.stackexchange.com/a/3680738, https://stackoverflow.com/a/55810955

        auto const v_diff{right_v - left_v}, vv{left_v + v_diff / 2.};
        if (std::abs(v_diff) < auto_robot_rotate_velocity_diff_min)
        {
            return {vv, 0., NAN};
        }
        auto const rad_to_left{(left_v + right_v) * auto_robot_axle_radius / v_diff};
        if (std::abs(rad_to_left) < auto_robot_rotate_radius_min)
        {
            return {vv, v_diff / 2. / auto_robot_axle_radius, 0.};
        }
        return {vv, vv / rad_to_left, rad_to_left};
    }
    constexpr auto calc_motor_velocities [[nodiscard]] (double target_lin_v, double target_ang_v) noexcept -> std::tuple<double, double>
    {
        if (std::abs(target_ang_v) < math::epsilon)
        {
            return {target_lin_v, target_lin_v};
        }
        auto target_rad_to_left{target_lin_v / target_ang_v};
        if (std::abs(target_rad_to_left) < auto_robot_rotate_radius_min)
        {
            return {-target_lin_v / auto_robot_axle_radius, target_lin_v / auto_robot_axle_radius};
        }
        return {}; // todo: calculate turning
    }
}

AutoRobotADRC::AutoRobotADRC(decltype(m_position) position, double rotation, decltype(m_velocities) velocities, decltype(m_gain) gain, double convergence) noexcept
    : m_position{std::move(position)},
      m_rotation{math::rotation_matrix2(rotation)},
      m_velocities{std::move(velocities)},
      m_gain{gain},
      m_position_control{1., convergence, {0.}},
      m_rotation_control{1., convergence, {0.}}
{
}

auto AutoRobotADRC::update(decltype(m_position) const &target, decltype(m_velocities) const &velocities, double dt) noexcept -> decltype(m_velocities)
{
    double left_v{}, right_v{};
    std::tie(left_v, right_v) = m_velocities;
    left_v *= m_gain;
    right_v *= m_gain;

    double lin_v{}, ang_v{}, rad{};
    std::tie(lin_v, ang_v, rad) = calc_linear_angular_velocities(left_v, right_v);
    if (std::isnan(rad))
    {
        m_position += m_rotation * decltype(m_position){0., lin_v * dt};
    }
    else
    {
        auto const rot_mat{math::rotation_matrix2(ang_v * dt)};
        m_position += (rot_mat - math::identity<double, 2>)*m_rotation * decltype(m_position){0., rad};
        m_rotation = math::orthogonalize_rotation_matrix2(rot_mat * m_rotation);
    }

    m_velocities = velocities;

    auto const forward_vec{m_rotation * decltype(m_position){0., 1.}};
    auto const pos_diff{target - m_position};
    auto const ang_diff{std::acos(math::dot_product(forward_vec, math::unit_vector(pos_diff)))};

    lin_v = m_position_control.update(0., lin_v, -math::magnitude(forward_vec * math::dot_product(forward_vec, pos_diff)), dt);
    ang_v = m_rotation_control.update(0., ang_v, std::isnan(ang_diff) ? 0. : -ang_diff, dt);

    std::tie(left_v, right_v) = calc_motor_velocities(lin_v, ang_v);
    return {left_v / m_gain, right_v / m_gain};
}
