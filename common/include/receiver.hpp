#pragma once

#include <array>
#include <cinttypes>
#include <optional>

#include "main.h"

struct Received
{
    constexpr static auto const bit_count{4};
    std::array<bool, bit_count> m_bits;
};

class Receiver
{
public:
    constexpr static auto const buffer_size{(Received::bit_count + 7) / 8};

private:
    UART_HandleTypeDef *m_handle;
    double m_timeout, m_last_tick;
    std::array<std::uint8_t, buffer_size> m_buffer{};
    Received m_last_received{};

    auto callback(std::uint16_t size) noexcept -> void;

public:
    explicit Receiver(decltype(*m_handle) &handle, decltype(m_timeout) timeout = 1.) noexcept;
    auto update() noexcept -> std::optional<Received>;
};
