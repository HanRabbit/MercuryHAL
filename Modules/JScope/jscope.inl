#pragma once

/**
 * @brief JScope 类模板，用于通过 SEGGER RTT 发送多种类型的数据到 JScope 工具进行实时监控和分析。
 *
 * @tparam Args 可变参数模板，表示要发送的数据类型列表。
 */
template<typename... Args>
JScope<Args...>::JScope(const Config &config) : config_(config) {
}

/**
 * @brief 获取 JScope 的名称，用于在 JScope 工具中显示。
 *
 * @return 返回一个包含 JScope 名称的 std::array<char, NameSize> 引用。
 *
 * @note 该函数会根据模板参数生成一个唯一的名称，格式为 "JScope_<类型1><类型2>..."。
 */
template<typename... Args>
const std::array<char,
    JScope<Args...>::NameSize> &
JScope<Args...>::getName() {
    static const auto name = [] {
        std::array<char, NameSize> str{};
        constexpr char prefix[] = "JScope_";
        memcpy(str.data(), prefix, 7);
        char *ptr = str.data() + 7;

        (
            [&] {
                const char *type = JScopeType<Args>::value;
                while (*type) {
                    *ptr++ = *type++;
                }
            }(),
            ...
        );

        *ptr = '\0';
        return str;
    }();
    return name;
}

/**
 * @brief 初始化 JScope，配置 SEGGER RTT 上行缓冲区。
 *
 * @note 该函数会根据配置参数设置缓冲区的大小和模式，并将名称注册到指定的通道。
 */
template<typename... Args>
void JScope<Args...>::init() {
    SEGGER_RTT_ConfigUpBuffer(
        config_.channel,
        getName().data(),
        nullptr,
        config_.buffer_size,
        config_.mode
    );
}

/**
 * @brief 发送数据到 JScope 工具进行实时监控和分析。
 *
 * @param args 可变参数列表，表示要发送的数据值。
 *
 * @note 该函数会将传入的参数打包成字节数组，并通过 SEGGER RTT 发送到指定的通道。
 */
template<typename... Args>
void JScope<Args...>::send(const Args &... args) {
    constexpr uint32_t Size = (sizeof(Args) + ...);
    uint8_t buffer[Size];
    uint8_t *ptr = buffer;

    (
        [&] {
            memcpy(ptr, &args, sizeof(args));
            ptr += sizeof(args);
        }(),
        ...
    );

    SEGGER_RTT_Write(config_.channel, buffer, Size);
}
