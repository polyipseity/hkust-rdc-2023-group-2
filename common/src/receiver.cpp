#include "receiver.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <functional>
#include <iterator>
#include <optional>
#include <tuple>
#include <utility>

#include "main.h"

namespace
{
    using RxEventCallbackT = void(std::uint16_t);
    using RxEventCallbackF = std::function<RxEventCallbackT>;

    constexpr auto const callback_size{2};
    std::array<std::tuple<UART_HandleTypeDef *, RxEventCallbackF>, callback_size> callbacks{};

    auto rx_event_callback(UART_HandleTypeDef *handle, std::uint16_t size) noexcept
    {
        for (auto &entry : callbacks)
        {
            if (std::get<0>(entry) != handle)
                continue;
            std::get<1>(entry)(size);
            break;
        }
    }

    auto register_rx_event_callback(UART_HandleTypeDef &handle, RxEventCallbackF callback) noexcept
    {
        HAL_UART_RegisterRxEventCallback(&handle, &rx_event_callback);
        for (auto &entry : callbacks)
        {
            if (std::get<0>(entry))
                continue;
            entry = {&handle, std::move(callback)};
            break;
        }
    }
}

Receiver::Receiver(decltype(*m_handle) &handle, decltype(m_last_tick) timeout) noexcept
    : m_handle{&handle}, m_timeout{timeout}, m_last_tick{-timeout}
{
    register_rx_event_callback(handle, [this](std::uint16_t size)
                               { callback(size); });
}

auto Receiver::callback(std::uint16_t size) noexcept -> void
{
    if (size != buffer_size)
        return;
    for (std::size_t ii{}; ii < Received::bit_count; ++ii)
    {
        m_last_received.m_bits[ii] = m_buffer[ii / 8] & (1 << (ii % 8));
    }
    m_last_tick = HAL_GetTick() / 1000.;
}

auto Receiver::update() noexcept -> std::optional<Received>
{
    HAL_UARTEx_ReceiveToIdle_IT(m_handle, std::data(m_buffer), buffer_size);
    if (HAL_GetTick() / 1000. - m_last_tick < m_timeout)
    {
        char buf[10]{};
        auto const len{std::snprintf(buf, 10, "%d", m_buffer[0])};
        std::uint8_t buf2[10]{};
        std::copy_n(std::cbegin(buf), len, std::begin(buf2));
        HAL_UART_Transmit(m_handle, buf2, len, HAL_MAX_DELAY);
        return m_last_received;
    }
    return {};
}
