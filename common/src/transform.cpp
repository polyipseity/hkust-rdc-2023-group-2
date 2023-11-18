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
    // https://archive.seattlerobotics.org/encoder/200108/using_a_pid.html#SteeringEquations, https://math.stackexchange.com/a/3680738, https://stackoverflow.com/a/55810955

    constexpr auto const auto_robot_axle_radius{.175};
    constexpr auto const auto_robot_rotate_velocity_diff_min{auto_robot_axle_radius / 16.};
    constexpr auto const auto_robot_self_rotate_radius_min{auto_robot_axle_radius / 16.};
    constexpr auto const auto_robot_max_velocity{1.8};
    constexpr auto const auto_robot_backward_max_velocity{.8}; // Needed because there is a odd wheel
    constexpr auto const auto_robot_max_angular_velocity{math::tau};
    constexpr auto const auto_robot_position_tolerance{1. / 16.};
    constexpr auto calc_auto_robot_velocities [[nodiscard]] (double left_v, double right_v) noexcept -> std::tuple<double, double, double>
    {
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
    constexpr auto calc_auto_robot_motor_velocities [[nodiscard]] (double target_lin_v, double target_ang_v, std::tuple<double, double> const &max_velocities) noexcept -> std::tuple<double, double>
    {
        auto [max_velocity, max_angular_velocity]{max_velocities};
        max_velocity = std::min(max_velocity, auto_robot_max_velocity);
        max_angular_velocity = std::min(max_angular_velocity, auto_robot_max_angular_velocity);
        target_ang_v = std::clamp(target_ang_v, -max_angular_velocity, max_angular_velocity);
        auto const extra_v{target_ang_v * auto_robot_axle_radius};
        target_lin_v = std::clamp(target_lin_v, -max_velocity + extra_v, max_velocity - extra_v);
        return {target_lin_v - extra_v, target_lin_v + extra_v};
    }

    // https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html, https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf

    constexpr auto const task_robot_semi_width{.53 / 2.};
    constexpr auto const task_robot_semi_height{.19 / 2.};
    constexpr auto const task_robot_minimum_angular_velocity{.01};
    constexpr auto const task_robot_forward_kinematics_matrix{[]()
                                                              {
                                                                  math::Matrix<double, 3, 4> ret{
                                                                      1., -1., -1., 1.,
                                                                      1., 1., 1., 1.,
                                                                      -1. / (task_robot_semi_width + task_robot_semi_height), 1. / (task_robot_semi_width + task_robot_semi_height), -1. / (task_robot_semi_width + task_robot_semi_height), 1. / (task_robot_semi_width + task_robot_semi_height)};
                                                                  ret /= 4.;
                                                                  return ret;
                                                              }()};
    constexpr math::Matrix<double, 4, 3> const task_robot_inverse_kinematics_matrix{
        1., 1., -(task_robot_semi_width + task_robot_semi_height),
        -1., 1., task_robot_semi_width + task_robot_semi_height,
        -1., 1., -(task_robot_semi_width + task_robot_semi_height),
        1., 1., task_robot_semi_width + task_robot_semi_height};
    constexpr auto calc_task_robot_velocities [[nodiscard]] (double v_fl, double v_fr, double v_rl, double v_rr) noexcept
    {
        return task_robot_forward_kinematics_matrix * math::Vector<double, 4>{v_fl, v_fr, v_rl, v_rr};
    }
    constexpr auto calc_task_robot_motor_velocities [[nodiscard]] (double v_lin_x, double v_lin_y, double v_ang) noexcept
    {
        return task_robot_inverse_kinematics_matrix * math::Vector<double, 3>{v_lin_x, v_lin_y, v_ang};
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

auto AutoRobotADRC::update(decltype(m_position) target, double target_rot, decltype(m_velocities) const &velocities, double dt, std::tuple<double, double> const &max_velocities) noexcept -> decltype(m_velocities)
{
    auto [left_v, right_v]{m_velocities};
    left_v *= m_gain;
    right_v *= m_gain;

    auto [lin_v, ang_v, rad]{calc_auto_robot_velocities(left_v, right_v)};
    m_position += lin_v * dt;
    if (!std::isnan(rad))
    {
        auto const rot_mat{math::rotation_matrix2(ang_v * dt)};
        m_rotation = math::orthogonalize_rotation_matrix2(rot_mat * m_rotation);
    }

    m_velocities = velocities;

    target_rot = std::fmod(target_rot, math::tau);
    if (target_rot < 0.)
    {
        target_rot += math::tau;
    }

    auto ang_diff{target_rot - math::rotation_matrix2_angle(m_rotation)};
    if (ang_diff > math::pi)
    {
        ang_diff -= math::tau;
    }
    else if (ang_diff < -math::pi)
    {
        ang_diff += math::tau;
    }

    lin_v = m_position_control.update(0., lin_v, m_position - target, dt);
    ang_v = m_rotation_control.update(0., ang_v, -ang_diff, dt);

    std::tie(left_v, right_v) = calc_auto_robot_motor_velocities(lin_v, ang_v, max_velocities);
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

auto AutoRobotTestADRC::update(decltype(m_position) const &target, std::optional<double> target_rot, decltype(m_velocities) const &velocities, double dt, std::tuple<double, double> const &max_velocities) noexcept -> decltype(m_velocities)
{
    auto [left_v, right_v]{m_velocities};
    left_v *= m_gain;
    right_v *= m_gain;

    auto [lin_v, ang_v, rad]{calc_auto_robot_velocities(left_v, right_v)};
    if (std::isnan(rad))
    {
        m_position += m_rotation * decltype(m_position){lin_v * dt, 0.};
    }
    else
    {
        auto const rot_mat{math::rotation_matrix2(ang_v * dt)};
        m_position += (rot_mat - math::identity<double, 2>)*m_rotation * decltype(m_position){0., -rad};
        m_rotation = math::orthogonalize_rotation_matrix2(rot_mat * m_rotation);
    }

    m_velocities = velocities;

    auto const forward_unit{m_rotation * decltype(m_position){1., 0.}};
    auto const pos_diff{target - m_position};
    auto const cur_ang{std::atan2(forward_unit(1), forward_unit(0))};
    auto ang_diff{std::atan2(pos_diff(1), pos_diff(0)) - cur_ang};
    if (math::magnitude(pos_diff) <= auto_robot_position_tolerance)
    {
        if (target_rot)
        {
            auto target_rot2{std::fmod(*target_rot, math::tau)};
            if (target_rot2 < 0.)
            {
                target_rot2 += math::tau;
            }
            ang_diff = target_rot2 - cur_ang;
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

    lin_v = m_position_control.update(0., lin_v, -math::dot_product(forward_unit, pos_diff), dt);
    ang_v = m_rotation_control.update(0., ang_v, -ang_diff, dt);

    std::tie(left_v, right_v) = calc_auto_robot_motor_velocities(lin_v, ang_v, max_velocities);
    return {left_v / m_gain, right_v / m_gain};
}

TaskRobotADRC::TaskRobotADRC(
    decltype(m_position) position,
    double rotation,
    decltype(m_velocities) velocities,
    math::Vector<double, 4> gain,
    double convergence) noexcept
    : m_position{std::move(position)},
      m_rotation{math::rotation_matrix2(rotation)},
      m_velocities{std::move(velocities)},
      m_gain{gain(0), 0., 0., 0.,
             0., gain(1), 0., 0.,
             0., 0., gain(2), 0.,
             0., 0., 0., gain(3)},
      m_position_control{1., convergence, {0.}},
      m_rotation_control{1., adrc_rotation_convergence_factor * convergence, {0.}}
{
}

auto TaskRobotADRC::update(decltype(m_position) const &target, double target_rot, decltype(m_velocities) const &velocities, double dt) noexcept -> decltype(m_velocities)
{
    auto const [v_fl, v_fr, v_rl, v_rr]{(m_gain * velocities).transpose()[0]};
    auto const [v_lin_x, v_lin_y, v_ang]{calc_task_robot_velocities(v_fl, v_fr, v_rl, v_rr).transpose()[0]};
    auto const rot_mat{math::rotation_matrix2(v_ang * dt)};
    m_position += m_rotation * decltype(m_position){v_lin_y, -v_lin_x} * dt;
    m_rotation = math::orthogonalize_rotation_matrix2(rot_mat * m_rotation);

    m_velocities = velocities;

    target_rot = std::fmod(target_rot, math::tau);
    if (target_rot < 0.)
    {
        target_rot += math::tau;
    }
    auto const pos_diff{target - m_position};
    auto const pos_diff_unit{math::unit_vector(pos_diff)};
    auto ang_diff{target_rot - math::rotation_matrix2_angle(m_rotation)};
    if (ang_diff > math::pi)
    {
        ang_diff -= math::tau;
    }
    else if (ang_diff < -math::pi)
    {
        ang_diff += math::tau;
    }

    auto const new_v_lin{m_rotation.transpose() * pos_diff_unit * m_position_control.update(0., math::dot_product(pos_diff_unit, m_rotation * math::Vector<double, 2>{v_lin_y, -v_lin_x}), -math::magnitude(pos_diff), dt)};
    auto const new_v_ang{m_rotation_control.update(0., v_ang, -ang_diff, dt)};

    return decltype(m_gain){
               1. / m_gain(0, 0), 0., 0., 0.,
               0., 1. / m_gain(1, 1), 0., 0.,
               0., 0., 1. / m_gain(2, 2), 0.,
               0., 0., 0., 1. / m_gain(3, 3)} *
           calc_task_robot_motor_velocities(-new_v_lin(1), new_v_lin(0), new_v_ang);
}
