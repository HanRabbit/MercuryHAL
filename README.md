<div align="center">

# MercuryBSP

**STM32 通用板级支持包（BSP）+ 常用功能模块（Modules）**

![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)
![MCU](https://img.shields.io/badge/MCU-STM32%20HAL-03234B?logo=stmicroelectronics&logoColor=white)
![RTOS](https://img.shields.io/badge/RTOS-CMSIS--RTOS%20v2-2E7D32)
![CMake](https://img.shields.io/badge/CMake-%E2%89%A53.22-064F8C?logo=cmake&logoColor=white)

</div>

从 [ArmNode](https://github.com/HanRabbit/ArmNode)（F1）与 [MercuryCore](https://github.com/HanRabbit/MIAO-Board)（H7）的 BSP/Modules 合并而来，目标是一份可跨工程复用的 STM32 C++ 外设封装与通用中间件。

---

## 架构

```
Application  →  Modules  →  BSP  →  CubeMX HAL / CMSIS / FreeRTOS
```

| 层级 | 内容 |
| --- | --- |
| **BSP** | `dwt` `gpio` `log` `iic`(+slave) `can`(FDCAN) `usart` `spi` `pwm` `usb`(CDC) + SEGGER RTT |
| **Modules** | `Algorithm` `MessageCenter` `JScope` `Daemon` `LED` `Buzzer` `Motor` `IMU` `Referee` `Vision` |
| **third_party** | SEGGER RTT（日志 / J-Scope） |

HAL 句柄与引脚由**消费工程的 CubeMX** 提供；本库不包含启动文件、链接脚本或芯片专属 HAL 源码。

---

## 快速接入（CMake）

```cmake
# 在你的固件 CMakeLists.txt 中（CubeMX target 已创建之后）
add_subdirectory(path/to/MercuryBSP)   # 或 FetchContent

# 只拉需要的组件（推荐）
mercury_add_bsp(${CMAKE_PROJECT_NAME}
    COMPONENTS dwt gpio log iic segger)
mercury_add_modules(${CMAKE_PROJECT_NAME}
    COMPONENTS Algorithm MessageCenter LED)

# 或一次性全开
# mercury_add_all(${CMAKE_PROJECT_NAME})
```

### FetchContent 示例

```cmake
include(FetchContent)
FetchContent_Declare(MercuryBSP
    GIT_REPOSITORY https://github.com/HanRabbit/MercuryBSP.git
    GIT_TAG        main)
FetchContent_MakeAvailable(MercuryBSP)

mercury_add_bsp(${CMAKE_PROJECT_NAME} COMPONENTS dwt gpio log iic segger)
mercury_add_modules(${CMAKE_PROJECT_NAME} COMPONENTS LED Daemon)
```

### 启动时初始化

```cpp
#include "bsp.h"

void app_init() {
    MercuryBSP_Init(HAL_RCC_GetHCLKFreq());
    LOG_INFO("[App] MercuryBSP ready");
}
```

---

## 组件说明

### BSP

| 组件 | 类 / API | 依赖 | 备注 |
| --- | --- | --- | --- |
| `dwt` | `DWT_Time` | `main.h`（CMSIS） | 跨 F1/F4/H7；勿再写死 `stm32xxxx.h` |
| `gpio` | `GPIO` | CubeMX `gpio.h` | |
| `log` | `LOG_DEBUG/INFO/WARNING/ERROR` | RTT + CMSIS-RTOS2 tick | 编译期 `LOG_MIN_LEVEL` + 运行时 `LOG_SET_LEVEL` |
| `iic` | `IIC` / `IIC_Slave` | CubeMX `i2c.h` | Master 阻塞/IT/DMA + 寄存器协议从机 |
| `can` | `FDCAN` | HAL FDCAN | **H7 FDCAN**；`init()` 根据已注册实例启动句柄 |
| `usart` | `USART` | HAL UART + DMA idle | |
| `spi` | `SPI` | HAL SPI | 阻塞 + DMA |
| `pwm` | `PWM` | HAL TIM | |
| `usb` | `USB` | `usbd_cdc_if.h` | 需工程提供 USB Device CDC |
| `segger` | RTT C 源 | — | 日志 / JScope 共用 |

### Modules

| 组件 | 说明 | 依赖 BSP / 外部 |
| --- | --- | --- |
| `Algorithm` | PID、EKF、`user_lib` | `dwt`；EKF 需 **CMSIS-DSP**（`arm_math.h`） |
| `MessageCenter` | 发布/订阅消息中心 | `log` |
| `JScope` | RTT 波形通道（header-only） | `segger` |
| `Daemon` | 掉线守护（周期 `Daemon::task()`） | — |
| `LED` | GPIO LED + 状态闪烁 | `gpio` + CMSIS-RTOS2 |
| `Buzzer` | PWM 蜂鸣器（Config 注入） | `pwm` |
| `Motor` | `DJI_Motor` + `DM_Motor` + `Motor_Task()` | `can` `dwt` `log` `Daemon` `Algorithm`；DM debug 可选 `usart`+`usb` |
| `IMU` | 超核 CAN / BMI088 SPI 统一接口 | `can` 或 `spi`+`gpio`；姿态融合需 **CMSIS-DSP** |
| `Referee` | RM 裁判系统串口 + UI 绘制 | `usart` `Daemon` CMSIS-RTOS2 |
| `Vision` | 视觉 USB CDC 定长帧协议 | `usb` `Daemon` |

#### Motor 通用化要点

- **DJI 分组发送**：按 `Config.can_init_config.hfdcan` 动态注册总线（最多 4 路），每总线 3 个 TX ID（`0x1FF/0x200/0x2FF`），不再写死 `hfdcan1/2`。
- **DM 调试透传**：`Config.debug=true` 时必须同时提供 `Config.debug_huart`，由应用注入调试串口；默认关闭。

#### IMU / Algorithm

工程 CMake 需提供 CMSIS-DSP 头文件路径（CubeMX 勾选 DSP 后通常已有），例如：

```cmake
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Middlewares/ST/ARM/DSP/Inc)  # 按你的工程实际路径
```

---

## 设计约定

1. **无堆（优先）**：外设实例用静态数组注册；Buzzer 内嵌 `PWM` 成员。
2. **回调分发**：`HAL_*Callback` 在对应 `.cpp` 内 `extern "C"` 实现，按句柄路由到实例。
3. **日志双层过滤**：`LOG_MIN_LEVEL`（编译期裁剪）+ `BSP::Log::setLevel()`（运行时）。
4. **枚举避开全大写**：`BSP::Log::Level::Debug` 等，避免与 `-DDEBUG` 宏冲突。
5. **板级细节在应用层**：引脚、`hfdcan*`、USB CDC 缓冲均由工程提供。

### 已知板级残留（后续可再抽象）

- `IIC::recoverBus()` 默认按 I2C1=PB6/7、I2C2=PB10/11 恢复总线；其他布线请改实现或后续加 pin 配置。
- `USB` 依赖工程侧 `CDC_Init` / `CDC_Transmit_HS` 符号。
- `can` 组件仅覆盖 **FDCAN**（经典 bxCAN 可后续加 `can/bxcan`）。

---

## 目录结构

```
MercuryBSP/
├── BSP/                  # 外设 C++ 封装
│   ├── bsp.h             # 伞头 + MercuryBSP_Init()
│   ├── dwt/ gpio/ log/ iic/ can/ usart/ spi/ pwm/ usb/
├── Modules/              # 通用功能模块
│   ├── Algorithm/ MessageCenter/ JScope/ Daemon/ LED/ Buzzer/
│   ├── Motor/ IMU/ Referee/ Vision/
│   └── general_def.h
├── third_party/SEGGER/   # RTT
├── cmake/                # 预留
├── CMakeLists.txt        # mercury_add_bsp / mercury_add_modules
└── README.md
```

---

## 版本与来源

| 来源 | 并入内容 |
| --- | --- |
| ArmNode (STM32F103) | `iic`(+slave)、增强 `log`、`LED`、部分 dwt/gpio |
| MercuryCore (STM32H723) | `can` `usart` `spi` `pwm` `usb`、全部 Modules（Motor/IMU/Referee/Vision 等） |

后续计划：bxCAN、I2C bus-recover 引脚配置、设备驱动可选分包。

---

## License

MIT（见 [LICENSE](LICENSE)）。SEGGER RTT 遵循 SEGGER 原许可。
