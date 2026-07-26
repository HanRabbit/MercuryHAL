#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include "SEGGER_RTT.h"

template<typename T>
struct JScopeType;

/**
 * @brief JScopeTimestamp 结构体，用于表示时间戳类型的数据。
 * @note 该结构体包含一个 32 位无符号整数值，表示时间戳的数值。
 */
struct JScopeTimestamp {
    uint32_t value;
};

template<>
struct JScopeType<float> {
    static constexpr auto value = "f4";
};


template<>
struct JScopeType<int32_t> {
    static constexpr auto value = "i4";
};


template<>
struct JScopeType<uint32_t> {
    static constexpr auto value = "u4";
};


template<>
struct JScopeType<JScopeTimestamp> {
    static constexpr auto value = "t4";
};

/**
 * @brief JScope 类模板，用于通过 SEGGER RTT 发送多种类型的数据到 JScope 工具进行实时监控和分析。
 *
 * @tparam Args 可变参数模板，表示要发送的数据类型列表。
 */
template<typename... Args>
class JScope {
public:
    struct Config {
        uint8_t channel = 1;
        uint32_t buffer_size = 1024;
        uint32_t mode =
                SEGGER_RTT_MODE_NO_BLOCK_SKIP;
    };

    explicit JScope(
        const Config &config = {}
    );

    void init();

    void send(const Args &... args);

private:
    static constexpr size_t NameSize = 8 + sizeof...(Args) * 2; /* "JScope_" + 每个类型的长度 */

    static const std::array<char, NameSize> &getName();

    Config config_;
};

#include "jscope.inl"
