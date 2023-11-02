#pragma once

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstddef>
#include <functional>
#include <optional>

#include "main.h"

using RxEventCallbackT = void(std::uint16_t);
using RxEventCallbackF = std::function<RxEventCallbackT>;

auto receiver_rx_event_callback(UART_HandleTypeDef *handle, std::uint16_t size) noexcept -> void;
auto receiver_register_rx_event_callback(UART_HandleTypeDef &handle, RxEventCallbackF callback) noexcept -> bool;
auto receiver_unregister_rx_event_callback(UART_HandleTypeDef &handle) noexcept -> bool;

template <std::size_t size_>
class Receiver
{
public:
    constexpr static auto const size{size_};

private:
    UART_HandleTypeDef *m_handle;
    double m_timeout, m_last_tick;
    std::array<std::uint8_t, size> m_buffer{};

    auto callback(std::uint16_t size) noexcept -> void
    {
        if (size != size_)
            return;
        m_last_tick = HAL_GetTick() / 1000.;
    }

public:
    explicit Receiver(decltype(*m_handle) &handle, decltype(m_timeout) timeout = 1.) noexcept : m_handle{&handle}, m_timeout{timeout}, m_last_tick{-timeout}
    {
        receiver_register_rx_event_callback(handle, [this](std::uint16_t size)
                                            { callback(size); });
    }
    auto update() noexcept -> std::optional<decltype(m_buffer)>
    {
        HAL_UARTEx_ReceiveToIdle_IT(m_handle, std::data(m_buffer), size_);
        if (HAL_GetTick() / 1000. - m_last_tick < m_timeout)
        {
            return m_buffer;
        }
        return {};
    }

    constexpr Receiver(Receiver const &) noexcept = delete;
    constexpr Receiver(Receiver &&) noexcept = default;
    constexpr auto operator=(Receiver const &) noexcept -> Receiver & = delete;
    constexpr auto operator=(Receiver &&) noexcept -> Receiver & = default;
    ~Receiver() noexcept
    {
        receiver_unregister_rx_event_callback(*m_handle);
    }
};
