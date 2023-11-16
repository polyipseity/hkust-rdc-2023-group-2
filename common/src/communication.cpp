#include "communication.hpp"

#include <algorithm>
#include <cinttypes>
#include <iterator>
#include <tuple>
#include <utility>

#include "main.h"

namespace
{
    constexpr auto const receiver_callback_size{4};
    std::array<std::tuple<UART_HandleTypeDef *, RxEventCallbackF>, receiver_callback_size> receiver_callbacks{};

    auto receiver_rx_event_callback(UART_HandleTypeDef *handle, std::uint16_t size) noexcept -> void
    {
        auto entry{std::find_if(std::begin(receiver_callbacks), std::end(receiver_callbacks), [handle](auto &&entry)
                                { return std::get<0>(entry) == handle; })};
        if (entry == std::end(receiver_callbacks))
        {
            return;
        }
        std::get<1> (*entry)(size);
    }
}

auto receiver_register_rx_event_callback(UART_HandleTypeDef &handle, RxEventCallbackF callback) noexcept -> bool
{
    auto entry{std::find_if(std::begin(receiver_callbacks), std::end(receiver_callbacks), [&handle](auto &&entry)
                            { return std::get<0>(entry) == &handle; })};
    if (entry == std::end(receiver_callbacks))
    {
        entry = std::find_if(std::begin(receiver_callbacks), std::end(receiver_callbacks), [](auto &&entry)
                             { return !std::get<0>(entry); });
        if (entry == std::end(receiver_callbacks))
        {
            return false;
        }
        HAL_UART_RegisterRxEventCallback(&handle, &receiver_rx_event_callback);
    }
    *entry = {&handle, std::move(callback)};
    return true;
}

auto receiver_unregister_rx_event_callback(UART_HandleTypeDef &handle) noexcept -> bool
{
    auto entry{std::find_if(std::begin(receiver_callbacks), std::end(receiver_callbacks), [&handle](auto &&entry)
                            { return std::get<0>(entry) == &handle; })};
    if (entry == std::end(receiver_callbacks))
    {
        return false;
    }
    *entry = {nullptr, {}};
    return true;
}
