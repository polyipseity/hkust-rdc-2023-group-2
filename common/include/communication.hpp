#pragma once

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstddef>
#include <functional>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>

#include "main.h"
#include "util.hpp"

/**
 * @brief `RxEventCallback` type
 */
using RxEventCallbackT = void(std::uint16_t);

/**
 * @brief `RxEventCallback` type wrapped in `std::function`
 */
using RxEventCallbackF = std::function<RxEventCallbackT>;

/**
 * @brief Register a `RxEventCallback` listener for the specified UART
 *
 * @param handle UART handle
 * @param callback listener
 * @return true on success
 * @return false on failure
 */
auto receiver_register_rx_event_callback(UART_HandleTypeDef &handle, RxEventCallbackF callback) noexcept -> bool;

/**
 * @brief Unregister a `RxEventCallback` listener for the specified UART
 *
 * @param handle UART handle
 * @return true on success
 * @return false on failure
 */
auto receiver_unregister_rx_event_callback(UART_HandleTypeDef &handle) noexcept -> bool;

/**
 * @brief UART receiver
 *
 * @tparam size_ number of bytes in a message
 * @tparam fixed whether the message size is fixed
 */
template <std::size_t size_, bool fixed_ = true>
class Receiver
{
public:
    /**
     * @brief Number of bytes in a message
     */
    constexpr static auto const size{size_};

    /**
     * @brief Whether the message size is fixed
     */
    constexpr static auto const fixed{fixed_};

private:
    /**
     * @brief The UART handle corresponding to the UART hardware
     */
    UART_HandleTypeDef *m_handle;

    /**
     * @brief Time a message is valid for
     */
    double m_timeout;

    /**
     * @brief Last time a message is received
     */
    double m_last_tick;

    /**
     * @brief Size of last message
     */
    std::size_t m_size{};

    /**
     * @brief Buffer for receiving the message
     */
    std::array<std::uint8_t, size> m_buffer{};

    /**
     * @brief Listener for this `Receiver`
     *
     * @param size number of bytes received
     */
    auto callback(std::uint16_t size) noexcept
    {
        if constexpr (fixed_)
        {
            if (size != size_)
                return;
        }
        m_last_tick = HAL_GetTick() / 1000.;
        m_size = size;
    }

public:
    /**
     * @brief Construct a new `Receiver` object
     *
     * @param handle UART handle
     * @param timeout time a message is valid for
     */
    explicit Receiver(decltype(*m_handle) &handle, decltype(m_timeout) timeout = 1.) noexcept
        : m_handle{&handle}, m_timeout{timeout}, m_last_tick{-timeout}
    {
        if (!receiver_register_rx_event_callback(handle, [this](std::uint16_t size)
                                                 { callback(size); }))
        {
            panic("Receiver::Receiver");
        }
    }

    /**
     * @brief Receive a message
     *
     * @return a message or empty if no messages have been received with the timeout
     */
    auto update() noexcept -> std::conditional_t<fixed_,
                                                 std::optional<decltype(m_buffer)>,
                                                 std::tuple<std::size_t, decltype(m_buffer)>>
    {
        HAL_UARTEx_ReceiveToIdle_IT(m_handle, std::data(m_buffer), size_);
        if (HAL_GetTick() / 1000. - m_last_tick < m_timeout)
        {
            if constexpr (fixed_)
            {
                return m_buffer;
            }
            else
            {
                return {m_size, m_buffer};
            }
        }
        return {};
    }

    auto invalidate() noexcept
    {
        m_last_tick = -m_timeout;
    }

    /**
     * @brief Destroy the `Receiver` object and unregister the listener
     */
    ~Receiver() noexcept
    {
        receiver_unregister_rx_event_callback(*m_handle);
    }

    constexpr Receiver(Receiver const &) noexcept = delete;

    constexpr Receiver(Receiver &&) noexcept = default;

    constexpr auto operator=(Receiver const &) noexcept -> Receiver & = delete;

    constexpr auto operator=(Receiver &&) noexcept -> Receiver & = default;
};

template <std::size_t max_params_>
class Commander
{
public:
    using ParamType = std::array<std::tuple<std::size_t, char const *>, max_params_>;
    using HandlerType = void(ParamType const &);

private:
    std::array<std::function<HandlerType>, 'z' - 'a' + 1> m_handlers{};

public:
    auto handle(char command, std::function<HandlerType> func)
    {
        m_handlers[command - 'a'] = std::move(func);
    }

    template <std::size_t size>
    auto dispatch(std::array<std::uint8_t, size> const &message, std::size_t actual_size)
    {
        if constexpr (size == 0)
        {
            return;
        }
        std::array<char, size + 1> msg_cpy{};
        std::copy_n(std::cbegin(message), actual_size, std::begin(msg_cpy));

        std::function<HandlerType> *func{};
        ParamType params{};
        auto param{std::begin(params)};
        for (auto &chr : msg_cpy)
        {
            if (chr == '\0' || chr == '\n' || chr == '\r')
            {
                chr = '\0';
                break;
            }
            if (!func)
            {
                if (!('a' <= chr && chr <= 'z'))
                {
                    break;
                }
                func = &m_handlers[chr - 'a'];
                continue;
            }
            if (chr == ',')
            {
                chr = '\0';
                if (++param == std::end(params))
                {
                    break;
                }
                continue;
            }
            auto &[p_size, p_ptr]{*param};
            ++p_size;
            if (!p_ptr)
            {
                p_ptr = &chr;
            }
        }
        if (func && *func)
        {
            (*func)(params);
        }
    }
};
