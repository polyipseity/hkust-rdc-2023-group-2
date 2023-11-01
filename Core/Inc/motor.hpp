#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "can.h"
#include "util/adrc.hpp"

class CANMotor
{
    Motor m_handle;

public:
    explicit CANMotor(decltype(m_handle) handle) noexcept;
    auto setInput(float input) noexcept -> void;
    auto getInput() const noexcept -> float;
    auto getPosition() const noexcept -> float;
    auto getVelocity() const noexcept -> float;
    auto getTemperature() const noexcept -> decltype(MotorStats::temperature);
};

template <std::size_t size>
class CANMotors
{
    template <std::size_t size_>
    friend class CANMotorsControl;
    std::array<CANMotor, size> m_handles;

public:
    template <std::size_t size_>
    friend class CANMotorsControl;
    constexpr explicit CANMotors(std::array<Motor, size> const &handles) noexcept
        : m_handles{[&handles]<std::size_t... indices>(std::index_sequence<indices...>) {
              return decltype(m_handles){CANMotor{handles[indices]}...};
          }(std::make_index_sequence<size>{})}
    {
        can_init();
    }

    constexpr auto const &operator[](decltype(size) index) const noexcept
    {
        return m_handles[index];
    }
};

template <std::size_t size>
class CANMotorsControl
{
    CANMotors<size> *m_delegate;

public:
    auto swap(CANMotorsControl &right) noexcept
    {
        using std::swap;
        swap(m_delegate, right.m_delegate);
    }
    friend auto swap(CANMotorsControl &left, CANMotorsControl &right) noexcept
    {
        left.swap(right);
    }
    constexpr CANMotorsControl(CANMotorsControl const &) noexcept = delete;
    constexpr CANMotorsControl(CANMotorsControl &&right) noexcept
        : m_delegate{std::move(right.m_delegate)}
    {
        right.m_delegate = nullptr;
    }
    constexpr auto operator=(CANMotorsControl const &) noexcept -> CANMotorsControl & = delete;
    constexpr auto operator=(CANMotorsControl &&right) noexcept -> CANMotorsControl &
    {
        CANMotorsControl{std::move(right)}.swap(*this);
        return *this;
    };
    ~CANMotorsControl() noexcept
    {
        if (m_delegate) {
            can_ctrl_loop();
        }
    }

    constexpr explicit CANMotorsControl(decltype(*m_delegate) const &delegate) noexcept
        : m_delegate{&delegate}
    {
    }

    constexpr auto &operator[](decltype(size) index) noexcept
    {
        return m_delegate->m_handles[index];
    }
    constexpr auto const &operator[](decltype(size) index) const noexcept
    {
        return (*m_delegate)[index];
    }
};

auto new_motor_ADRC(CANMotor const &motor, float convergence = 16.) -> control::ADRC2f;

template <typename Control>
constexpr auto update_motor_velocity(double dt, CANMotor &motor, Control &control, float velocity)
{
    motor.setInput(control.update(velocity / 2., motor.getInput(), motor.getVelocity(), dt));
}
