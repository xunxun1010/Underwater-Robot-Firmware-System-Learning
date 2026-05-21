# Underwater-Robot-Firmware-System-Learning
This is a repository used to record the learning of the underwater robot firmware system

---

## 目录

1. [项目概述与系统全景图](#第一章-项目概述与系统全景图)
2. [STM32 基础配置详解](#第二章-stm32-基础配置详解)
3. [FreeRTOS 配置与任务设计](#第三章-freertos-配置与任务设计)
4. [核心知识点深度解析](#第四章-核心知识点深度解析)
   - 4.1 推进器混合矩阵与水动力学模型
   - 4.2 PID 控制算法完全拆解
   - 4.3 一阶低通滤波器
   - 4.4 偏航角 ±180° 归一化
   - 4.5 积分饱和与抗饱和设计
   - 4.6 DSHOT600 数字电调协议
   - 4.7 SBUS 协议解码原理
   - 4.8 AHRS 姿态解算
   - 4.9 RTAutoInit 自动初始化框架
5. [动手实践指南](#第五章-动手实践指南)
6. [面试准备与项目展示](#第六章-面试准备与项目展示)

---

## 第一章 项目概述与系统全景图

### 1.1 这是什么项目？

ROV-X 是一台 **水下遥控航行器（ROV, Remotely Operated Vehicle）** 的嵌入式固件系统。ROV 是一种无人水下机器人，通过脐带缆或无线通信接受操作员遥控，可执行水下观测、采样、检修等任务。

本固件运行在 STM32F405VG 微控制器上（ARM Cortex-M4F 内核，168MHz 主频，1MB Flash，192KB SRAM），基于 FreeRTOS 实时操作系统构建了 13 个并发任务，完成了从传感器数据采集、姿态融合、遥控指令解析、PID 闭环控制到推进器/舵机/灯光输出的完整机器人控制链路。

### 1.2 硬件系统框图

<img width="1392" height="2161" alt="image" src="https://github.com/user-attachments/assets/19e804e7-518a-4ae4-b1be-db92f07f3d63" />


### 1.3 软件架构总览：13 任务数据管道

整个系统可抽象为 **3 层数据管道**：

<img width="1219" height="2631" alt="image" src="https://github.com/user-attachments/assets/dd6c11dd-03f0-4170-9aa0-a1c964ffeb14" />


## 第二章 STM32 基础配置详解

### 2.1 STM32F405VG 芯片特性

STM32F405VG 是 ST 公司基于 ARM Cortex-M4F 内核的高性能 MCU：

| 参数 | 数值 | 对本项目的意义 |
|------|------|---------------|
| **主频** | 168 MHz | 满足 20ms 控制周期内的浮点 PID + DSHOT DMA 计算 |
| **Flash** | 1 MB | 存储完整固件 + FatFs 文件系统 + 网络协议栈 |
| **SRAM** | 192 KB | 13 个任务的栈空间 + 各种通信缓冲区 |
| **FPU** | 硬件单精度浮点 | 加速 AHRS 姿态解算中的大量浮点运算 |
| **DMA** | 2 个控制器共 16 个 stream | DSHOT 波形生成、ADC 扫描、UART 接收均用 DMA |
| **定时器** | 最多 17 个 | TIM1/3 → DSHOT，TIM4 → 舵机，TIM12 → 探照灯，TIM5 → IMU 加热 |
| **SPI** | 3 个 | SPI1 → BMI088，SPI2 → W5500/MS5803 |
| **I2C** | 3 个 | I2C1 → IST8310，I2C2 → MS5837 |
| **UART** | 6 个 | UART1 → 超声波/UART3 → 遥控/UART6 → 上位机 |

### 2.2 时钟树配置

STM32F405 的时钟树是系统的心脏。本项目使用外部 25MHz 晶振（HSE），通过 PLL 倍频到 168MHz：

```text
HSE (25MHz)
  │
  ├──→ PLL_M = 25    →  输入分频  25MHz / 25 = 1MHz
  │     ├──→ PLL_N = 336  →  VCO = 1MHz × 336 = 336MHz
  │     │     └──→ PLL_P = 2   →  SYSCLK = 336MHz / 2 = 168MHz ✓
  │     ├──→ PLL_Q = 7    →  USB时钟 = 336MHz / 7 = 48MHz ✓
  │     └──→ PLL_R = 7    →  (备用)
  │
  └──→ SYSCLK = 168MHz
        ├──→  HCLK  = 168MHz  (AHB总线，CPU/内存/DMA)
        │     └──→  APB1  = 42MHz   (低速外设：UART/I2C/SPI2-3)
        │     └──→  APB2  = 84MHz   (高速外设：TIM1/SPI1/ADC)
        └──→  SysTick = 168MHz / 8 = 21MHz (FreeRTOS时基)
```

**关键理解**：TIM1（用于 DSHOT）挂在 APB2（84MHz）上，TIM3 也挂在 APB1（42MHz×2=84MHz）。DSHOT600 要求信号脉宽精度达到纳秒级，定时器时钟必须足够高。

### 2.3 外设功能分配详解

#### 2.3.1 SPI1 — BMI088 陀螺仪/加速度计

```text
PA5  → SPI1_SCK    (时钟，最高 10MHz)
PA6  → SPI1_MISO   (主入从出，读取传感器数据)
PA7  → SPI1_MOSI   (主出从入，写传感器寄存器)
PC14 → GPIO_OUT    (加速度计片选 ACCEL_NSS)
PC15 → GPIO_OUT    (陀螺仪片选 GYRO_NSS)
```

BMI088 包含两个独立的 SPI 从设备（加速度计和陀螺仪各一个），因此需要两根独立的片选线。读写流程：
1. 拉低对应片选（选中加速度计或陀螺仪）
2. 发送寄存器地址（最高位=1 表示读，=0 表示写）
3. 传输数据（读模式继续发时钟脉冲读取 MISO）
4. 拉高片选

代码读取加速度计 X 轴数据的典型流程：
```c
// 伪代码：读取 BMI088 加速度计 X 轴数据（16bit，寄存器 0x12-0x13）
BMI_ACCEL_NSS_LOW();                    // 拉低加速度计片选
uint8_t tx_data[3] = {0x12 | 0x80, 0xFF, 0xFF}; // 地址 | 读标志位(0x80), 两个dummy字节
uint8_t rx_data[3];
HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 3, 100); // 全双工传输
BMI_ACCEL_NSS_HIGH();                   // 拉高片选
int16_t accel_x = (rx_data[1] << 8) | rx_data[2]; // 拼接高低字节
// 注意：BMI088 加速度数据为有符号 16 位，量程可配（默认 ±24g）
```

#### 2.3.2 I2C1 — IST8310 磁力计

```text
PB8 → I2C1_SCL  (时钟)
PB9 → I2C1_SDA  (数据)
PE0 → GPIO_IT   (数据就绪中断)
PE1 → GPIO_OUT  (复位)
```

IST8310 是 I2C 从设备，7 位地址为 0x0C。每次读取需要先写寄存器地址，再发起读操作：
```c
// 伪代码：读取 IST8310 三轴磁场数据（6字节，从寄存器 0x03 开始）
uint8_t reg = 0x03;
uint8_t mag_data[6];
HAL_I2C_Master_Transmit(&hi2c1, 0x0C << 1, &reg, 1, 100);  // 先写要读的起始寄存器地址
HAL_I2C_Master_Receive(&hi2c1, 0x0C << 1, mag_data, 6, 100); // 再读6字节数据
int16_t mag_x = (mag_data[1] << 8) | mag_data[0];  // 小端序
int16_t mag_y = (mag_data[3] << 8) | mag_data[2];
int16_t mag_z = (mag_data[5] << 8) | mag_data[4];
```

#### 2.3.3 I2C2（GPIO 模拟）— MS5837 深度传感器

MS5837 虽然支持 I2C 协议，但本项目没有使用硬件 I2C2 控制器，而是用 GPIO 位带操作模拟 I2C 时序：

```c
// main.h 中的宏定义实现了 GPIO 模式切换和引脚控制
#define MS5837_SDA_IN()  {GPIOB->MODER &= ~(3 << (11*2)); GPIODB->MODER |= 0 << 11*2;}
// 将 PB11 的 MODER 寄存器位清零 → 输入模式（00）
// 解释：MODER 寄存器每 2 位控制一个引脚的模式，PB11 对应第 11 组
// 00 = 输入, 01 = 输出, 10 = 复用, 11 = 模拟

#define MS5837_SDA_OUT() {GPIOB->MODER &= ~(3 << (11*2)); GPIODB->MODER |= 1 << 11*2;}
// 将 PB11 设置为输出模式（01）

#define MS5837_SCL_Pin GPIO_PIN_10    // SCL 使用 PB10
#define MS5837_SDA_Pin GPIO_PIN_11    // SDA 使用 PB11
```

**为什么用 GPIO 模拟而不用硬件 I2C？**
- I2C2 可能被其他设备占用或存在资源冲突
- 模拟 I2C 更灵活，可精确控制时序
- MS5837 对某些 I2C 时序有特殊要求，硬件 I2C 可能在 STOP 条件处理上有差异

#### 2.3.4 TIM1 + TIM3 — 8 路 DSHOT600 输出

DSHOT 是本项目最具技术含量的外设应用。它利用定时器的比较输出 + DMA 实现任意波形生成：

```text
TIM1（APB2 总线, 84MHz）:
  TIM1_CH1 → PE9  → 左前推进器
  TIM1_CH2 → PE11 → 左中推进器
  TIM1_CH3 → PE13 → 左后推进器
  TIM1_CH4 → PB14 → 右前推进器（注意：此引脚同时是探照灯 LED0，通过重映射解决）

TIM3（APB1 总线, 42MHz×2=84MHz）:
  TIM3_CH1 → PB4  → 右中推进器
  TIM3_CH2 → PB5  → 右后推进器
  TIM3_CH3 → PB0  → 八推左中推进器
  TIM3_CH4 → PB1  → 八推右中推进器
```

#### 2.3.5 UART 配合 DMA + IDLE 中断

项目中的串口接收采用 **DMA 循环模式 + UART IDLE 中断** 的组合方案：

```text
工作原理：
1. 启动 DMA 循环接收，将 UART 数据寄存器持续搬运到内存缓冲区
2. 当串口总线上出现一个字节的空闲（IDLE）时，触发 IDLE 中断
3. 在 IDLE 中断服务程序中：
   - 计算本次接收的字节数 = 设置的总长度 - DMA 剩余计数
   - 将数据交给协议解析函数
   - 重置 DMA 以便接收下一帧
```

这种方式的好处：**不占用 CPU 时间逐字节接收**，只在完整帧到达后才触发一次中断处理。

---

## 第三章 FreeRTOS 配置与任务设计

### 3.1 FreeRTOSConfig.h 关键参数解析

本项目使用 FreeRTOS v10.3.1，配合 CMSIS-RTOS V2 封装层。关键配置：

```c
// === 调度器核心参数 ===

#define configUSE_PREEMPTION            1
// 抢占式调度：高优先级任务就绪时立即抢占低优先级任务的 CPU
// 这是 ImuTask（最高优先级）能保证实时性的前提

#define configUSE_TIME_SLICING          1
// 时间片轮转：同优先级任务轮流执行，每个 tick 切换一次
// 本项目同优先级任务不多，此参数影响不大

#define configTICK_RATE_HZ              ((TickType_t)1000)
// 系统节拍 = 1kHz，即每个 tick = 1ms
// 这意味着 vTaskDelay(50) = 延迟 50ms = ControlTask 的控制周期
// 高 tick 率 = 更精确的延时，但 = 更多的定时器中断开销

#define configMAX_PRIORITIES            ((UBaseType_t)56)
// 最大优先级数 = 56
// CMSIS-RTOS V2 映射：osPriorityIdle=0, osPriorityLow=8, osPriorityNormal=24 等
// 本项目实际使用 7 个不同优先级，56 的上限非常充裕

// === 内存管理 ===

#define configSUPPORT_STATIC_ALLOCATION  1
// 启用静态内存分配：任务的 TCB 和栈都由编译器在 .bss/.data 段中分配
// 优点：内存使用在编译时确定，无运行时碎片，适合安全关键系统
// 对应代码中每个任务都提供了 StaticTask_t 和 stack buffer

#define configSUPPORT_DYNAMIC_ALLOCATION 1
// 同时启用动态分配（部分队列使用动态创建）

#define configTOTAL_HEAP_SIZE           ((size_t)15360)
// 堆大小 = 15KB，使用 heap_4 算法
// heap_4：最佳匹配算法 + 碎片合并，是 FreeRTOS 推荐的内存方案

// === 钩子函数 ===

#define configUSE_IDLE_HOOK             0
// 不启用空闲钩子（空闲钩子常用于低功耗模式，ROV 不需要）

#define configCHECK_FOR_STACK_OVERFLOW  0
// 不启用栈溢出检测（生产版本为节省开销关闭，调试阶段建议设为 2）

// === 软件定时器 ===

#define configUSE_TIMERS                1
#define configTIMER_TASK_PRIORITY       (osPriorityLow)
#define configTIMER_QUEUE_LENGTH        10
#define configTIMER_TASK_STACK_DEPTH    256
```

### 3.2 13 个任务的优先级设计原则

优先级分配遵循 **实时性需求的金字塔原则**：

```text
osPriorityRealtime    = 最高  → ImuTask, InsTask
                                    ↑
osPriorityHigh1       = 高    → HostTask, BlueTask
                                    ↑
osPriorityHigh        = 高    → PressureTask
                                    ↑
osPriorityAboveNormal = 中高  → MoveTask, ServoTask
                                    ↑
osPriorityNormal1     = 中    → RemoteTask
                                    ↑
osPriorityNormal      = 中    → ControlTask, SendTask
                                    ↑
osPriorityNormal3     = 中低  → RadarTask
                                    ↑
osPriorityLow         = 低    → MessageTask, LedTask
                                    ↑
osPriorityLow4        = 最低  → iniTask（初始化后自删除）
```

**设计原则分析**：

1. **传感器采集（ImuTask, PressureTask）→ 最高优先级**
   - 原因：控制算法依赖传感器反馈，数据必须"新鲜"（低延迟）
   - ImuTask 每 1ms 读取一次 IMU，如果被延迟会导致姿态估计误差累积
   - 优先级高于控制任务：确保反馈数据在被使用前已更新

2. **运动执行（MoveTask）→ 中高优先级（高于 ControlTask）**
   - 这是一个反直觉的设计：执行者优先级高于决策者
   - 原因：MoveTask 的 20ms 周期比 ControlTask（50ms）更短
   - DSHOT 输出需要严格的 20ms 周期，任何抖动都会导致推进器抖动
   - ControlTask 发布指令后，MoveTask 立即响应

3. **控制融合（ControlTask）→ 中等优先级**
   - 它依赖各个传感器队列的数据，Peek 操作不会被阻塞
   - 50ms 周期允许一定延迟而不影响整体性能

4. **通信任务（HostTask, BlueTask, RemoteTask）→ 中高优先级**
   - 通信数据丢失比传感器数据丢失更可接受
   - 但协议栈需要一定的实时性以保证帧解析不丢

5. **状态显示（LedTask, MessageTask）→ 低优先级**
   - 灯光和数据聚合对延迟完全不敏感
   - 1s 周期意味着即使被延迟 100ms 也看不出差异

### 3.3 任务间通信：队列类型选择

本项目使用两种队列类型，选择依据非常讲究：

| 队列名称 | 类型 | 长度 | API | 原因 |
|----------|------|------|-----|------|
| `ImuMessageQueueHandle` | Overwrite | 1 | `xQueuePeek` | 只关心最新姿态数据，旧数据无价值 |
| `PressureMessageQueueHandle` | Overwrite | 1 | `xQueuePeek` | 同上，只关心当前深度 |
| `SbusMessageQueueHandle` | Overwrite | 1 | `xQueuePeek` | 遥控器最新指令，旧指令无意义 |
| `RovControlQueueHandle` | Overwrite | 1 | `xQueueOverwrite` | 控制指令只需最新值 |
| `RovStatusQueueHandle` | Overwrite | 1 | `xQueueOverwrite` | 状态数据只需最新值 |
| `netTxQueue` | FIFO | 10 | `xQueueSend` | 网络数据必须保证顺序，不能丢帧 |

**Overwrite Queue 的设计精髓**：

```c
// 创建 Overwrite Queue：长度为 1 的普通队列
RovControlQueueHandle = osMessageQueueNew(1, sizeof(RovControl_t), NULL);

// 发送方：使用 xQueueOverwrite —— 如果队列满，覆盖旧数据
xQueueOverwrite(RovControlQueueHandle, &RovControlMessage);

// 接收方：使用 xQueuePeek —— 读取但不移除，允许多个消费者
xQueuePeek(RovControlQueueHandle, &ControlMessage, 0);
//                                                      ↑
//                                           timeout = 0：不阻塞，立即返回
```

**为什么用 Peek 而不是 Receive？**
- 传感器数据有多个消费者（ControlTask、MoveTask、MessageTask 都需要同一帧 IMU 数据）
- 如果用 Receive，第一个消费者取走数据后队列变空，其他消费者读不到
- Peek 保留数据在队列中，所有消费者都能读到同一帧

**为什么 netTxQueue 用 FIFO？**
- 网络传输需要保证数据包的顺序（Modbus TCP 帧不能乱序）
- 长度=10 提供足够的缓冲，吸收瞬时发送高峰
- 如果队列满了，说明网络带宽不足，阻塞发送方是合理的背压机制

### 3.4 RTAutoInit 自动初始化框架深度解析

这是本项目中架构最优美的部分。它借鉴了 **RT-Thread** 的组件初始化机制，利用 **GCC 链接器的 section 属性** 实现了模块的自动注册和按序初始化。

#### 3.4.1 问题背景

在传统嵌入式项目中，初始化顺序依赖 `main()` 函数中的手动调用：

```c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_I2C1_Init();
    // ... 30 个初始化函数 ...
    MX_FREERTOS_Init();
    // 如果有人不小心调整了顺序，系统可能崩溃
    // 新增模块必须在 main() 中手动插入初始化调用
}
```

这个模式有严重问题：
- **耦合**：每增加一个模块就要修改 `main()`
- **顺序脆弱**：交换两行初始化可能导致依赖错误
- **不利于团队协作**：多人修改同一个 `main()` 容易冲突

#### 3.4.2 RTAutoInit 的解决方案

核心思想：**每个模块在自己的 .c 文件中声明初始化函数，编译器自动将它们按优先级放入不同的内存段（section），运行时按段依次调用。**

##### 第一步：宏定义——将函数指针放入指定 linker section

```c
// autoinit.h 中的核心宏
// 以 GCC 编译器为例：

#define INIT_EXPORT(fn, level)                                               \
    RT_USED const init_fn_t __rt_init_##fn RT_SECTION(".rti_fn." level) = fn
//  ↑        ↑                    ↑                    ↑                ↑
//  ①        ②                    ③                    ④                ⑤

// ① RT_USED = __attribute__((used))
//    告诉编译器：即使看起来没有被显式调用，也保留这个变量（不要被优化掉）

// ② const init_fn_t
//    声明一个常量函数指针，类型为 int (*)(void)

// ③ __rt_init_##fn
//    ## 是 C 预处理器的连接符。例如 INIT_EXPORT(ControlTaskInit, "5")
//    展开为 __rt_init_ControlTaskInit

// ④ RT_SECTION(".rti_fn." level)
//    __attribute__((section(".rti_fn.5")))
//    将此变量放入名为 ".rti_fn.5" 的独立内存段中
//    这是整个框架的关键！所有 level="5" 的模块，
//    其函数指针会被收集到同一个 section 中

// ⑤ = fn
//    将此函数指针初始化为指定的函数
```

##### 第二步：6 级优先级宏

```c
#define INIT_BOARD_EXPORT(fn)      INIT_EXPORT(fn, "1")   // 板级硬件：时钟、GPIO
#define INIT_PREV_EXPORT(fn)       INIT_EXPORT(fn, "2")   // 预初始化：纯软件
#define INIT_DEVICE_EXPORT(fn)     INIT_EXPORT(fn, "3")   // 外设初始化：SPI、I2C
#define INIT_COMPONENT_EXPORT(fn)  INIT_EXPORT(fn, "4")   // 中间件：文件系统、网络栈
#define INIT_ENV_EXPORT(fn)        INIT_EXPORT(fn, "5")   // 环境层：任务创建 ✨
#define INIT_APP_EXPORT(fn)        INIT_EXPORT(fn, "6")   // 应用层：用户逻辑
```

项目中大部分任务使用 `INIT_ENV_EXPORT`（Level 5），这意味着它们在 FreeRTOS 调度器启动后、作为任务创建：

```c
// 每个任务文件末尾：
int ControlTaskInit(void) {
    ControlTaskHandle = osThreadNew(ControlTask_Function, NULL, &ControlTaskattributes);
    if (ControlTaskHandle == NULL)
        lwlog_err("ControlTask Init error.");
    return 0;
}
INIT_ENV_EXPORT(ControlTaskInit);  // ← 自动注册！
// 展开为：
// __attribute__((used, section(".rti_fn.5")))
// const init_fn_t __rt_init_ControlTaskInit = ControlTaskInit;
```

##### 第三步：链接脚本——定义段的边界符号

在链接脚本（.ld 文件）中，所有 `.rti_fn.*` 段被按名称排序并放置在一起：

```text
/* 链接脚本片段（伪代码） */
.rti_fn :
{
    __rt_init_start = .;         /* 记录起始地址 */
    KEEP(*(SORT(.rti_fn.1)))     /* 收集所有 level 1 的函数指针，按名称排序 */
    KEEP(*(SORT(.rti_fn.2)))     /* 收集所有 level 2 的函数指针 */
    KEEP(*(SORT(.rti_fn.3)))
    KEEP(*(SORT(.rti_fn.4)))
    KEEP(*(SORT(.rti_fn.5)))
    KEEP(*(SORT(.rti_fn.6)))
    __rt_init_end = .;           /* 记录结束地址 */
}
```

链接后，内存布局如下：

```text
低地址  ┌──────────────────────┐
        │ __rt_init_start      │ ← 链接器导出的符号
        │ ImuTaskInit (level1) │ ← 如果有的话...
        │ ...                  │
        │ RadarTaskInit        │ ← INIT_PREV_EXPORT (level 2)
        │ LedTaskInit          │ ← INIT_COMPONENT_EXPORT (level 4)
        │ ImuTaskInit          │ ← INIT_ENV_EXPORT (level 5)
        │ ControlTaskInit      │ ← INIT_ENV_EXPORT (level 5)
        │ MoveTaskInit         │ ← INIT_ENV_EXPORT (level 5)
        │ ...                  │
        │ MessageTaskInit      │ ← INIT_APP_EXPORT (level 6)
        │ __rt_init_end        │
高地址  └──────────────────────┘
```

##### 第四步：运行时初始化函数

```c
// autoinit.c 中的实现

void rt_components_board_init(void) {
    // 在调度器启动前调用，执行 Board 级别的初始化
    const init_fn_t *fn_ptr;
    // 遍历 .rti_fn 段中的所有函数指针
    for (fn_ptr = &__rt_init_start; fn_ptr < &__rt_init_end; fn_ptr++) {
        (*fn_ptr)();  // 依次调用每个初始化函数
    }
}

void rt_components_init(void) {
    // 在 iniTask 中调用（调度器已启动），执行其余级别的初始化
    // 同样遍历整个段并依次调用
}
```

##### 第五步：在 main() 中的调用

```c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    // ... CubeMX 生成的硬件初始化 ...

    rt_components_board_init();   // ← 自动调用所有 INIT_BOARD_EXPORT 函数

    osKernelInitialize();         // 初始化 FreeRTOS 内核
    MX_FREERTOS_Init();           // 创建 iniTask（包含 rt_components_init）
    osKernelStart();              // 启动调度器
}
```

**总结：RTAutoInit 的优势**

| 传统方式 | RTAutoInit |
|---------|------------|
| main() 中手动维护初始化顺序 | 编译器自动按 level 排序 |
| 新增模块必须修改 main() | 只需在模块中加一行宏 |
| 顺序错误导致难以排查的 bug | 编译时确定顺序，不可变 |
| 多人协作易冲突 | 各模块独立声明，无冲突 |

---

## 第四章 核心知识点深度解析

### 4.1 推进器混合矩阵与水动力学模型

#### 4.1.1 什么是推进器混合矩阵？

ROV 有 4 个（或 8 个）推进器，但操作员想要控制的是 6 个自由度（X/Y/Z 平移 + 横滚/俯仰/偏航旋转）。**推进器混合矩阵** 就是将目标自由度上的力/力矩指令，分配到各个推进器上的数学变换。

```text
[推进器1推力]               [X方向力]
[推进器2推力]               [Y方向力]
[推进器3推力]  =  Mixer  ×  [Z方向力]
[   ...    ]               [横滚力矩]
[推进器N推力]               [俯仰力矩]
                           [偏航力矩]
```

Mixer 矩阵由推进器在机器人坐标系中的安装位置和朝向决定。

#### 4.1.2 ROV-XE 推进器布局（8 推构型）

```text
            前方 (Forward)
                ↑
                │
     左前 ←  ┌──┴──┐  → 右前
   (LF)     │      │     (RF)
            │  ROV │
     左中 ← │ 俯视 │ → 右中
   (LM)     │      │     (RM)
            │      │
     左后 ← └──┬──┘  → 右后
   (LB)          │      (RB)
                ↓
            后方 (Back)

垂直面（侧视，面向右侧）：
            上方 (Up)
                ↑
     右中 ←  ┌──┴──┐  → 右中上(八推)
   (RM)     │      │     (ERM)
            │  ROV │
            │ 侧视 │
            └──────┘
```

**8 个推进器的角色**：

| 编号 | 名称 | 方向 | 位置 | 控制功能 |
|------|------|------|------|----------|
| RightFront (RF) | 右前 | 水平 | 右前 | X平移 + Y前进 + Yaw偏航 |
| RightBack (RB) | 右后 | 水平 | 右后 | X平移 + Y前进 + Yaw偏航 |
| LeftFront (LF) | 左前 | 水平 | 左前 | X平移 + Y前进 + Yaw偏航 |
| LeftBack (LB) | 左后 | 水平 | 左后 | X平移 + Y前进 + Yaw偏航 |
| RightMiddle (RM) | 右中 | 垂直 | 右中 | Z浮潜 + Roll横滚 + Pitch俯仰 |
| LeftMiddle (LM) | 左中 | 垂直 | 左中 | Z浮潜 + Roll横滚 + Pitch俯仰 |
| EightRightMiddle (ERM) | 八推右中 | 垂直 | 右中后 | Z浮潜 + Roll横滚 + Pitch俯仰 |
| EightLeftMiddle (ELM) | 八推左中 | 垂直 | 左中后 | Z浮潜 + Roll横滚 + Pitch俯仰 |

#### 4.1.3 混合矩阵的推导

以 ROV-XE 的水平面（4 个推进器）为例：

**物理直觉**：
- 要让 ROV 前进（+Y）：4 个推进器同时向前推 → RF+RB+LF+LB 同向
- 要让 ROV 平移（+X）：左推和右推方向相反 → RF 和 RB 向左、LF 和 LB 向右
- 要让 ROV 偏航（+Yaw）：对角线推进器反向 → RF+LB 与 RB+LF 方向相反

**ROV-XE 的混合矩阵**（来自 MoveTask.c 源码）：

```c
// 水平面 4 推进器的混合（开环速度模式）
PropellerOut[RightFront] = -ControlMessage.SpeedX - ControlMessage.SpeedY;
//                         ↑X 右推向左            ↑Y 右推向前
//                         负号：右前推正向（从后向前看顺时针），产生 +X 需要向左推力
//                         -SpeedY：所有推同时向前

PropellerOut[RightBack]  =  ControlMessage.SpeedX - ControlMessage.SpeedY;
//                         ↑X 右后推向右            ↑Y 右后推向前
//                         +SpeedX：右后推正向，产生 -X 需要向右推力

PropellerOut[LeftFront]  =  ControlMessage.SpeedX - ControlMessage.SpeedY;
//                         ↑X 左前推向左            ↑Y 左前推向前
//                         +SpeedX：左前推正向，产生 +X 需要向左推力

PropellerOut[LeftBack]   = -ControlMessage.SpeedX - ControlMessage.SpeedY;
//                         ↑X 左后推向右            ↑Y 左后推向前
//                         -SpeedX：左后推正向，产生 -X 需要向右推力
```

用矩阵形式表达：

```text
┌──────────────┐     ┌                    ┐
│  RF(右前)    │     │  -1    -1     0   │
│  RB(右后)    │     │  +1    -1     0   │    ┌─────┐
│  LF(左前)    │  =  │  +1    -1     0   │ ×  │ X 推力 │
│  LB(左后)    │     │  -1    -1     0   │    │ Y 推力 │
│  RM(右中)    │     │   0     0    +1   │    │ Z 推力 │ (垂直简化)
│  LM(左中)    │     │   0     0    -1   │    └─────┘
│  ERM(八右中) │     │   0     0    -1   │
│  ELM(八左中) │     │   0     0    +1   │
└──────────────┘     └                    ┘
```

**验证**：假设需要纯前进（SpeedX=0, SpeedY=100）：
- RF = 0 - 100 = -100（负 = 正向推进）
- RB = 0 - 100 = -100
- LF = 0 - 100 = -100
- LB = 0 - 100 = -100
- 四个推进器同向推 → ROV 前进 ✓

假设需要纯平移（SpeedX=100, SpeedY=0）：
- RF = -100 - 0 = -100（负方向）
- RB = +100 - 0 = +100（正方向）
- LF = +100 - 0 = +100
- LB = -100 - 0 = -100
- 右侧两个推反向（一正一负），左侧也反向 → ROV 平移 ✓

#### 4.1.4 PID 闭环对混合矩阵的修正

当启用闭环控制时，PID 的输出会叠加（而非替代）到开环混合上：

```c
// 深度闭环（Depth PID）
if ((ControlMessage.Mode & DeepinControl)) {
    DEPTH_PID.set = ControlMessage.PositionZ;           // 目标深度
    DEPTH_PID.fdb = filter(0.5f, PressureMessage.depth, &LastPositionZ);  // 滤波后的当前深度
    PID_Calc(&DEPTH_PID);
    PropellerOut[RightMiddle]      = -DEPTH_PID.out;    // 右中推进器 = PID输出（负=向下推）
    PropellerOut[LeftMiddle]       =  DEPTH_PID.out;    // 左中推进器 = PID输出（正=向上推）
    PropellerOut[EightRightMiddle] =  DEPTH_PID.out;
    PropellerOut[EightLeftMiddle]  = -DEPTH_PID.out;
}

// 横滚闭环（Roll PID）—— 叠加到垂直推进器上
if ((ControlMessage.Mode & PitchRollControl)) {
    ROLL_PID.set = ControlMessage.AngleRoll;              // 目标横滚角
    ROLL_PID.fdb = filter(0.5f, ImuMessage.roll, &LastAngleRoll);  // 当前横滚角
    PID_Calc(&ROLL_PID);
    PropellerOut[RightMiddle]      += ROLL_PID.out;      // += 叠加！不影响深度控制
    PropellerOut[LeftMiddle]       += ROLL_PID.out;
    PropellerOut[EightRightMiddle] -= ROLL_PID.out;
    PropellerOut[EightLeftMiddle]  -= ROLL_PID.out;
}

// 偏航闭环（Yaw PID）—— 叠加到水平推进器上
if ((ControlMessage.Mode & YawControl)) {
    YawError = ImuMessage.yaw - ControlMessage.AngleYaw;
    // ... 归一化 ...
    YAW_PID.set = 0.0f;                                   // set=0 表示目标误差为0
    YAW_PID.fdb = filter(0.2f, YawError, &LastYawError);  // 当前偏航误差
    PID_Calc(&YAW_PID);
    PropellerOut[RightFront] -= YAW_PID.out;               // += 叠加到已有值上
    PropellerOut[RightBack]  -= YAW_PID.out;
    PropellerOut[LeftFront]  += YAW_PID.out;
    PropellerOut[LeftBack]   += YAW_PID.out;
}
```

**叠加机制的精妙之处**：PID 闭环输出叠加在开环混合之上，使得：
- 操作员可以手动控制前进/后退，同时 PID 自动修正偏航
- 深度自动保持的同时，横滚/俯仰 PID 也在工作
- 多个 PID 控制器独立计算，互不干扰

---

### 4.2 PID 控制算法完全拆解

#### 4.2.1 PID 控制器的数学原理

PID 控制器是一种**误差驱动的反馈控制器**。给定目标值（setpoint）和当前值（feedback），计算出一个控制输出使当前值趋近目标值。

```text
                 ┌─────────┐
目标值 r(t) ──→○──┤   PID   ├──→ 控制输出 u(t) ──→ [被控对象] ──→ 当前值 y(t)
              ↑  └─────────┘                                      │
              └───────────────────────────────────────────────────┘
                             反馈 (feedback)
```

**连续时间的 PID 方程**：

```text
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·(de(t)/dt)

其中：
  e(t) = r(t) - y(t)  = 误差 = 目标值 - 当前值
  Kp                    = 比例增益
  Ki                    = 积分增益
  Kd                    = 微分增益
```

**各项的物理意义**：

- **P（比例项）**：`Kp × e(t)` — 当前误差的线性放大
  - 误差大时输出大（快速趋近目标）
  - 误差小时输出小（避免超调）
  - **单独使用的问题**：有稳态误差（靠目标越近，驱动力越小，在到达前就平衡了）

- **I（积分项）**：`Ki × ∫e(t)dt` — 历史误差的累积
  - 只要误差不为零，积分项持续增长，直到误差被消除
  - **解决了 P 的稳态误差问题**
  - **引入的问题**：积分饱和（下面 4.5 节详细讨论）

- **D（微分项）**：`Kd × de/dt` — 误差变化率的预测
  - 相当于"预判"误差趋势，提前刹车
  - 减小超调和振荡
  - **引入的问题**：对噪声敏感（噪声的导数可以很大）

#### 4.2.2 本项目使用的位置式 PID（离散实现）

在嵌入式系统中，积分和微分需要离散化。位置式 PID 的离散形式：

```text
Pout = Kp × error[0]
Iout = Iout + Ki × error[0]       ← 积分累加（注意：保持 Iout 变量）
Dout = Kd × (error[0] - error[1]) ← 微分近似：当前误差 - 上次误差

out  = Pout + Iout + Dout         ← 三项线性叠加
```

**位置式 vs 增量式的区别**：

| | 位置式 PID | 增量式 PID |
|---|---|---|
| 输出 | 控制量的绝对值 | 控制量的变化量 Δu |
| 积分 | 历史全累积 | 不需要积分项 |
| 适用 | 执行器需要绝对值（舵机角度） | 执行器需要变化量（步进电机） |
| 本项目 | ✓ ROV 推进器需要绝对推力值 | |

#### 4.2.3 源码逐行分析

```c
// pid.h — 数据结构定义
typedef struct {
    uint8_t mode;         // PID_POSITION(0) 或 PID_DELTA(1)

    float Kp;             // 比例增益
    float Ki;             // 积分增益
    float Kd;             // 微分增益

    float max_out;        // 总输出限幅（防止推进器过载）
    float max_iout;       // 积分限幅（防积分饱和的关键参数！）

    float set;            // 目标值（setpoint）— 外部写入
    float fdb;            // 反馈值（feedback）— 外部写入

    float out;            // 最终输出值
    float Pout;           // 比例项输出（调试用）
    float Iout;           // 积分项累积值
    float Dout;           // 微分项输出（调试用）
    float Dbuf[3];        // 微分缓冲 [0]=当前 [1]=上一次 [2]=上上次
    float error[3];       // 误差缓冲 [0]=当前 [1]=上一次 [2]=上上次
} PidTypeDef;
```

```c
// pid.c — 初始化函数
void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3],
              int max_out, int max_iout) {
    // 参数检查：确认是位置式还是增量式
    if (pid == NULL || PID == NULL)
        return;

    pid->mode = mode;       // 本项目始终使用 PID_POSITION

    // 从配置数组加载 PID 参数
    pid->Kp = PID[0];       // 例如 DEPTH_PID_KP = 12.0f
    pid->Ki = PID[1];       // 例如 DEPTH_PID_KI = 0.05f
    pid->Kd = PID[2];       // 例如 DEPTH_PID_KD = 160.0f

    pid->max_out  = max_out;   // 例如 DEPTH_PID_MAX_OUT = 500
    pid->max_iout = max_iout;  // 例如 DEPTH_PID_MAX_IOUT = 200

    // 将所有状态变量清零
    pid->set = 0;
    pid->fdb = 0;
    pid->out = 0;
    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
    // Dbuf[3] 和 error[3] 已在 bss 段中自动初始化为 0
}
```

```c
// pid.c — 核心计算函数
float PID_Calc(PidTypeDef *pid) {

    // === 步骤 1：更新误差历史（移位寄存器） ===
    pid->error[2] = pid->error[1];   // 上上次 ← 上一次
    pid->error[1] = pid->error[0];   // 上一次   ← 当前
    pid->error[0] = pid->set - pid->fdb;
    // error[0] = 目标值 - 当前反馈值
    // 例如：深度控制中，set=目标深度1000mm, fdb=当前深度800mm, error=200mm

    // === 步骤 2：位置式 PID 计算 ===
    if (pid->mode == PID_POSITION) {

        // 比例项：误差的线性映射
        pid->Pout = pid->Kp * pid->error[0];
        // 例如：Pout = 12.0 × 200 = 2400
        // 含义：当前偏离目标 200mm，需要 2400 的推力来纠正
        // Kp 越大，同样的误差产生的推力越大，系统响应越快

        // 微分项：误差变化率（用后向差分近似）
        pid->Dbuf[2] = pid->Dbuf[1];   // 微分的历史值也保持
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        // Dbuf[0] = 当前误差 - 上次误差 = 误差的变化量
        // 例如：上次误差=210mm，当前误差=200mm，变化量=-10mm
        // 负值表示误差在缩小，机器人正在靠近目标

        pid->Dout = pid->Kd * pid->Dbuf[0];
        // Dout = 160.0 × (-10) = -1600
        // 负值！这会在机器人快速靠近目标时"刹车"，减小总输出
        // Kd 越大，抑制超调的能力越强

        // 积分项：误差的累积
        pid->Iout += pid->Ki * pid->error[0];
        // Iout = 上次Iout + 0.05 × 200 = Iout + 10
        // 只要误差不为零，Iout 就持续增长（或减小）
        // 作用是消除仅靠 P 无法消除的稳态误差

        // *** 积分限幅（抗饱和关键！）***
        if (pid->Iout > pid->max_iout) {
            pid->Iout = pid->max_iout;   // 积分上限钳位
        } else if (pid->Iout < -pid->max_iout) {
            pid->Iout = -pid->max_iout;  // 积分下限钳位
        }
        // 为什么要限幅？详见 4.5 节

        // 三项叠加得到总输出
        pid->out = pid->Pout + pid->Iout + pid->Dout;

        // *** 输出限幅 ***
        if (pid->out > pid->max_out) {
            pid->out = pid->max_out;     // 总输出上限钳位
        } else if (pid->out < -pid->max_out) {
            pid->out = -pid->max_out;    // 总输出下限钳位
        }
        // 防止 PID 输出超过推进器的物理极限（DSHOT 的油门范围）
    }

    // === 步骤 3：增量式 PID（本项目不使用，仅作为备选） ===
    else if (pid->mode == PID_DELTA) {
        // 增量式 PID 计算的是控制输出的变化量 Δu
        // Δu = Kp*(e[0]-e[1]) + Ki*e[0] + Kd*(e[0]-2*e[1]+e[2])
        // 适用于步进电机等需要增量控制的执行器
        // ...
    }

    return pid->out;  // 返回最终控制输出
}
```

#### 4.2.4 参数整定实操方法

工程中最常用的 PID 整定方法——**试凑法（经验法）**：

**第一步：只调 P（Ki=0, Kd=0）**

```text
Kp = 1   → 系统响应很慢，离目标很远
Kp = 2   → 加快
Kp = 4   → 再加快
...
Kp = 12  → 开始出现小幅度振荡（超调后又欠调）
目标：找到刚好出现小幅持续振荡的 Kp 值
```

在 ROV 中的表现：设目标深度 500mm，P 过小时机器人慢慢下降，P 过大时机器人超调到 550mm 又弹回 450mm。

**第二步：加 D 抑制超调**

```text
在第一步找到的 Kp 基础上，逐渐增大 Kd
Kd = 20  → 超调略微减小
Kd = 80  → 超调明显减小
Kd = 160 → 基本没有超调，但响应变慢
目标：在超调和响应速度间找平衡
```

D 的本质是"提前点刹车"：当机器人快速靠近目标深度时，D 项产生一个反向推力，减小总输出。

**第三步：加 I 消除稳态误差**

```text
在 Kp、Kd 确定后，逐渐增大 Ki
Ki = 0.01 → 稳态误差慢慢消失
Ki = 0.03 → 稳态误差消失更快
Ki = 0.05 → 可能开始有小幅振荡
目标：找到刚好不振荡的最大 Ki 值
```

I 的本质是"历史清算"：如果机器人在 480mm 停住了（差 20mm 不到目标），即使 Kp×20 的推力不够，I 项会随着时间累积逐渐增大，最终补充足够的推力消除这 20mm 误差。

**本项目 ROV-XE 的最终参数**：

| PID 环 | Kp | Ki | Kd | max_out | max_iout | 特点分析 |
|--------|----|----|----|---------|----------|---------|
| 深度 | 12.0 | 0.05 | 160.0 | 500 | 200 | 高 Kd 防止深度超调（超调意味着撞底），较大 Ki 快速消除深度稳态误差 |
| 俯仰 | 6.0 | 0.03 | 180.0 | 1000 | 300 | 高 Kd 保证姿态稳定，大输出限幅允许强俯仰修正 |
| 横滚 | 8.0 | 0.03 | 120.0 | 1000 | 300 | Kp 稍大于俯仰，因为横滚惯量小响应快 |
| 偏航 | 8.0 | 0.003 | 200.0 | 500 | 100 | Ki 极小（偏航不需要精确定点），Kd 很大（抑制偏航振荡） |
| 巡线X | 16.0 | 0.00 | 0.0 | 100 | 10 | 纯 P 控制（不需要消除稳态误差），响应快 |
| 巡线Y | 16.0 | 0.00 | 0.0 | 100 | 10 | 同上 |

---

### 4.3 一阶低通滤波器

#### 4.3.1 为什么需要滤波？

IMU 的原始数据包含高频振动噪声，深度传感器在水中受水流影响也产生波动。如果直接将原始数据送入 PID 控制器：

```text
原始数据：500, 520, 480, 510, 490, 530, 470, 505, ...
PID 微分项（Kd=160）：160×(520-500) + 160×(480-520) + ...
                    = +3200 + (-6400) + ...
                    → 微分项剧烈振荡！导致推进器来回抖动
```

#### 4.3.2 一阶低通滤波的数学原理

一阶低通滤波器是一种**指数平滑**（Exponential Smoothing）算法：

```text
y[n] = α × x[n] + (1 - α) × y[n-1]

其中：
  x[n]   = 当前原始输入值
  y[n]   = 当前滤波输出值
  y[n-1] = 上一次滤波输出值
  α      = 平滑系数 (0 < α ≤ 1)
```

**频域分析**：一阶低通滤波在 z 域的传递函数为：

```text
H(z) = α / (1 - (1-α)·z^(-1))
截止频率 fc ≈ α / (2π·Ts)  （Ts 为采样周期）
```

**α 值的影响**：

| α 值 | fc（Ts=20ms时） | 效果 | 适用场景 |
|------|-----------------|------|---------|
| 1.0 | ~8 Hz | 无滤波（输出=输入） | 不需要 |
| 0.5 | ~4 Hz | 中等滤波，延迟约 1 个周期 | 深度、姿态角（本项目） |
| 0.2 | ~1.6 Hz | 强滤波，延迟约 3 个周期 | 偏航角（本项目） |
| 0.05 | ~0.4 Hz | 很强滤波，延迟约 10 个周期 | 温度测量等极慢变量 |

**为什么偏航用 α=0.2 而深度用 α=0.5？**
- 深度控制对响应速度要求更高（撞底危险）
- 偏航角本身变化较慢（ROV 转向惯性大），可以接受更强的滤波
- 偏航角用了 Kd=200（很大的微分），更强的滤波可以防止微分项被噪声放大

#### 4.3.3 源码实现

```c
// filter.c
float filter(float alpha, float new_value, float *last_value) {
    // alpha:         滤波系数 α
    // new_value:     当前原始采样值
    // *last_value:   上一次的滤波输出值（同时用作 in/out 参数）

    float filtered_value;

    // 核心公式：y[n] = α·x[n] + (1-α)·y[n-1]
    filtered_value = alpha * new_value + (1.0f - alpha) * (*last_value);

    // 更新历史值供下次使用
    *last_value = filtered_value;

    return filtered_value;
}
```

#### 4.3.4 在本项目中的调用

```c
// MoveTask.c 中的典型用法
static float LastPositionZ = 0;  // 保持上一次深度滤波值
// ...
DEPTH_PID.fdb = filter(0.5f, PressureMessage.depth, &LastPositionZ);
//              ↑     ↑                         ↑
//              α=0.5 原始深度传感器读数          上一次滤波值的引用（会被更新）
```

**一个完整的滤波过程示例**（深度 α=0.5）：

```text
采样次数  原始值    滤波后     计算过程
  1      800mm    800.0mm    filter(0.5, 800, &0) = 0.5×800 + 0.5×0 = 400...
                             实际上系统在运行前会先初始化 LastPositionZ = 首次读数
  2      810mm    805.0mm    0.5×810 + 0.5×800 = 805
  3      795mm    800.0mm    0.5×795 + 0.5×805 = 800
  4      805mm    802.5mm    0.5×805 + 0.5×800 = 802.5
  5      800mm    801.25mm   0.5×800 + 0.5×802.5 = 801.25
```

可以看到：原始数据在 795-810 间波动（振幅 15），滤波后在 800-805 间（振幅 5），有效地抑制了高频噪声。

---

### 4.4 偏航角 ±180° 归一化问题

#### 4.4.1 问题描述

偏航角（Yaw）的范围是 [-180°, +180°]（或 [0°, 360°]）。当 ROV 从 +179° 继续顺时针转时，角度跳变为 -179°（或相反）。这个跳变在 PID 控制器中会造成巨大的误差尖峰：

```text
场景：目标偏航角 = +170°，当前偏航角 = -170°

如果直接计算误差：error = 170 - (-170) = 340°  ← 巨大误差！

但实际上最短旋转方向只需要 20°（从 -170° 顺时针转 20° 到 +170°）

PID 输出：Pout = Kp × 340 = 8.0 × 340 = 2720
正确输出：Pout = Kp × (-20) = 8.0 × (-20) = -160
       // 负号表示逆时针旋转更近
```

如果不处理，机器人会绕一个大圈（转 340°）而不是走最近的路径（转 20°）。

#### 4.4.2 解决方案

```c
// MoveTask.c 中的偏航误差归一化代码
YawError = ImuMessage.yaw - ControlMessage.AngleYaw;
// YawError 的原始范围：[-360°, +360°]

// 归一化到 [-180°, +180°]
if (YawError <= -180)
    YawError += 360;     // 例如 -340° → -340 + 360 = +20°
else if (YawError >= 180)
    YawError -= 360;     // 例如 +340° → +340 - 360 = -20°

// YawError 现在代表最短旋转方向上的角度差
// 正值 = 需要顺时针转 |YawError| 度
// 负值 = 需要逆时针转 |YawError| 度
```

**图解**：

```text
    -170°                           +170°
  ←──────┼──────────────────────────┼───────
          \      340° 路径（错误！）/
           \                      /
            \    20° 路径（正确） /
             ────────────────────
```

#### 4.4.3 为什么 PID 控制偏航时 set=0？

```c
YAW_PID.set = 0.0f;
YAW_PID.fdb = filter(0.2f, YawError, &LastYawError);
PID_Calc(&YAW_PID);
```

注意：`set=0` 意味着**目标不是某个角度值，而是"误差为零"**。

这是因为偏航角的闭环控制逻辑不同：目标偏航角已经通过累加方式记录在 `ControlMessage.AngleYaw` 中，所以 PID 的输入变为"当前偏航与目标偏航之间的角度差"，目标就是让这个差值为 0。

这是一种**误差驱动的间接控制**模式，等价于：

```text
传统方式：  PID(set=170°, fdb=当前yaw)  →  输出
本项目方式：YawError = 当前yaw - 170° → PID(set=0°, fdb=YawError)  →  输出
```

两种方式数学上等价，但第二种方式配合归一化更直观。

---

### 4.5 积分饱和与抗饱和设计

#### 4.5.1 什么是积分饱和（Integral Windup）？

积分饱和是 PID 控制中最常见的实际问题。当执行器达到物理极限（饱和）时，积分项继续累积，导致系统失控。

**场景**：ROV 需要从 0mm 深度下潜到 3000mm（水下 3 米），但推进器最大推力对应的深度变化速率是 500mm/s：

```text
时刻 t=0:  深度=0,   误差=3000, Iout=0
时刻 t=1s: 深度=500, 误差=2500, Iout=0 + 0.05×2500 = 125
时刻 t=2s: 深度=1000,误差=2000, Iout=125 + 0.05×2000 = 225
...
时刻 t=6s: 深度=3000,误差=0,    Iout=225 + 0.05×0 = 225  ← 积分停在 225

问题：到达目标深度时，积分已经累积到 225
此时 Pout = Kp×0 = 0（误差为 0 了）
但总输出 out = 0 + 225 + Dout → 机器人继续下潜！
机器人超过目标深度，误差变负，积分开始回退...

结果：深度在目标值附近大幅振荡，甚至失控
```

在没有积分限幅的情况下，ROV 会像弹簧一样在目标深度上下剧烈振荡，这就是**积分饱和导致的超调和振荡**。

#### 4.5.2 本项目的抗饱和方案：积分限幅（Clamping）

```c
// pid.c 中的积分限幅
if (pid->Iout > pid->max_iout) {
    pid->Iout = pid->max_iout;       // 硬钳位到上限
} else if (pid->Iout < -pid->max_iout) {
    pid->Iout = -pid->max_iout;      // 硬钳位到下限
}
```

**max_iout 的选择**：

- `DEPTH_PID_MAX_IOUT = 200`：深度环积分的最大贡献是 ±200
  - 配合 `DEPTH_PID_MAX_OUT = 500`，即使积分满，也只占总输出的 200/500=40%
  - 其余 60% 留给 P 和 D 项，确保动态响应不受积分完全主导

- `YAW_PID_MAX_IOUT = 100`：偏航环积分更保守
  - 偏航不需要精确角度（±几度就行），较小的积分限制防止偏航振荡

#### 4.5.3 其他抗饱和方法（拓展知识）

| 方法 | 原理 | 优缺点 |
|------|------|--------|
| **条件积分** | 仅当误差小于阈值时才启动积分 | 简单有效，但阈值选择困难 |
| **反计算（Back-Calculation）** | 将限幅后的输出与限幅前的输出的差值反馈给积分项 | 最优雅的方案，需要额外参数 |
| **积分限幅** ✓ | 直接钳位积分项的上限（本项目采用） | 最简单，参数直观，适合嵌入式 |

---

### 4.6 DSHOT600 数字电调协议

#### 4.6.1 为什么不用普通 PWM？

传统 PWM 控制无刷电调（ESC）的问题是：
- **分辨率低**：1000-2000μs 脉宽，步进通常 1μs，只有 1000 级
- **需要校准**：每次上电要校准油门范围
- **无反馈**：不知道电调是否收到了信号
- **速度慢**：50Hz（20ms/帧）的更新率限制了控制带宽

DSHOT600 是专为无人机和机器人设计的数字协议：
- **数字信号**：每帧 16 位数字数据，分辨率 = 65536 级
- **自带 CRC 校验**：电调可验证数据完整性
- **不需要校准**：数字值直接对应油门
- **高速**：DSHOT600 = 600kbps，一帧只需约 27μs

#### 4.6.2 DSHOT600 的物理层编码

DSHOT600 使用**脉宽调制**在单线上传输数字信号：

```text
位速率：600 kHz → 每个位周期 = 1/600000 = 1.67μs

位编码：
  逻辑 "0"：高电平 625ns，低电平 1042ns  ───┐ 注意：高低电平时间都在微秒级
  逻辑 "1"：高电平 1250ns，低电平 417ns  ───┘

一帧格式（16 位）：
  ┌─────────────────────────────────────────────────────────┐
  │ bit15-11 │ bit10 │ bit9-6  │ bit5-0 │
  │ throttle │ TLM   │ CRC[3:0]│ 保留   │
  │ (11位)   │ 请求  │ (4位)   │        │
  └─────────────────────────────────────────────────────────┘

油门值范围：
  0      = 禁止（disarmed）
  1-47   = 保留（特殊命令）
  48-2047 = 正向油门（0% ~ 100%）
```

**CRC 计算**：DSHOT 使用 CRC4/ITU 校验：
```
CRC = (throttle[10:4] * 0xD5) ^ (throttle[3:0] * 0x0B)
取结果的高 4 位作为 CRC[3:0]
```

#### 4.6.3 用 STM32 定时器 + DMA 生成 DSHOT 波形

这是本项目中硬件操作最精巧的部分。如何用 STM32 的普通定时器（只能输出固定 PWM）生成可变脉宽的 DSHOT 信号？

**核心思路**：将 DSHOT 的一帧编码为一系列"位周期"的定时器占空比值，利用 DMA 将这些值连续传输到定时器的比较寄存器（CCR），实现任意波形生成。

```c
// dshot.c 中的核心数据结构

// 每个推进器需要一组 DMA 缓冲区，存放编码后的波形数据
// 16位数据 × 每个位需要的CCR值 = 一帧的DMA数据
typedef struct {
    uint16_t *dma_buffer;    // DMA 发送缓冲区（编码后的 CCR 值序列）
    TIM_HandleTypeDef *htim; // 对应的定时器
    uint32_t channel;        // 对应的通道（TIM_CHANNEL_1~4）
} DshotChannel_t;

// 编码一个油门值到 DMA 缓冲区
void DshotEncode(uint16_t throttle, uint16_t *buffer) {
    // 1. 计算 CRC
    uint16_t crc = DshotCRC(throttle);

    // 2. 组装 16 位帧
    uint16_t frame = (throttle << 5) | ((crc & 0x0F) << 1);
    //    throttle 占高 11 位，CRC 占低 4 位

    // 3. 逐位编码为脉宽值（CCR 寄存器值）
    for (int i = 15; i >= 0; i--) {  // MSB first
        if (frame & (1 << i)) {
            // 逻辑 "1"：高电平 1250ns
            // 定时器时钟 = 84MHz, 周期 = 1/84M ≈ 11.9ns
            // CCR值 = 1250ns / 11.9ns ≈ 105
            *buffer++ = 105;   // 在这个比较值切换到低电平
        } else {
            // 逻辑 "0"：高电平 625ns
            // CCR值 = 625ns / 11.9ns ≈ 53
            *buffer++ = 53;
        }
        // 注：实际代码中用 ARR 控制位周期总长度（1.67μs = 140 个 tick）
    }

    // 4. 帧间隔（至少 2μs 的低电平）
    *buffer++ = 0;  // CCR=0 表示输出始终为低
}
```

**DMA 传输流程**：
1. CPU 编码好 16 位的 CCR 值序列到内存缓冲区
2. 启动 DMA，将缓冲区数据逐次传输到 `TIMx->CCRy` 寄存器
3. 定时器在每个溢出事件后自动加载下一个 CCR 值
4. DMA 传输完成后触发中断，准备下一帧

**优势**：CPU 只需在每帧开始前编码一次（~50μs），其余时间完全由 DMA 硬件自动完成，不占用 CPU 时间。8 路推进器 × 600kbps 的波形生成对 CPU 几乎零负载。

---

### 4.7 SBUS 协议解码原理

#### 4.7.1 SBUS 协议简介

SBUS 是 Futaba 公司开发的串行总线遥控协议，一帧包含 16 个比例通道 + 2 个开关通道：

```text
物理层：
  波特率：100,000 bps（非标准！标准串口不支持此波特率）
  数据位：8，偶校验，2 个停止位
  电平：反相逻辑（高=0, 低=1）← 需要硬件反相器或软件处理
  帧率：约 7ms/帧（模拟模式）或 14ms/帧（数字模式）

帧格式（25 字节）：
  ┌──────┬──────────────────────┬──────┬──────┐
  │ 0x0F │ 22字节通道数据       │ 标志 │ 0x00 │
  │ 帧头 │ (16CH×11bit=176bit) │ 字节 │ 帧尾 │
  └──────┴──────────────────────┴──────┴──────┘

通道数据编码：
  CH1[10:0]  → 字节1[7:0] + 字节2[2:0]
  CH2[10:0]  → 字节2[7:3] + 字节3[5:0]
  ...
  CH16[10:0] → 字节22[5:0] + 字节23[7:3]
  
  每通道 11 位，值范围 [0, 2047]（中位=1024）
```

#### 4.7.2 解码流程

```c
// sbus.c 中的核心解码逻辑

// 字节缓冲区中的通道数据是紧密打包的 11 位序列
// 11 位 × 16 通道 = 176 位 = 22 字节

void SbusDecode(uint8_t *raw_data, SbusMsg_t *sbus_msg) {
    // raw_data 是 25 字节完整帧（从 UART IDLE 中断得到）

    // 帧头验证
    if (raw_data[0] != 0x0F) return;  // 不是 SBUS 帧

    // 帧尾验证
    if (raw_data[24] != 0x00) return; // 帧尾异常

    // 逐通道解包（每通道 11 位，紧密排列）
    // CH1: 使用字节0的全部8位 + 字节1的低3位
    sbus_msg->CH[0] = (raw_data[1]       | (raw_data[2] << 8))  & 0x07FF;
    // CH2: 使用字节1的高5位 + 字节2的低6位
    sbus_msg->CH[1] = ((raw_data[2] >> 3) | (raw_data[3] << 5))  & 0x07FF;
    // CH3:
    sbus_msg->CH[2] = ((raw_data[3] >> 6) | (raw_data[4] << 2) | (raw_data[5] << 10)) & 0x07FF;
    // ... 以此类推，每 3 个字节包含 2 个完整的 11 位通道值

    // 将 [0, 2047] 映射到有意义的值域
    // 本项目将通道值减去中位（1024），使之成为有符号值 [-1024, +1023]
    sbus_msg->CH1 = (int16_t)(sbus_msg->CH[0]) - 1024;  // 横滚/平移
    sbus_msg->CH2 = (int16_t)(sbus_msg->CH[1]) - 1024;  // 俯仰/前进
    sbus_msg->CH3 = (int16_t)(sbus_msg->CH[2]) - 1024;  // 油门/浮潜
    sbus_msg->CH4 = (int16_t)(sbus_msg->CH[3]) - 1024;  // 偏航
    sbus_msg->CH5 = (int16_t)(sbus_msg->CH[4]) - 1024;  // 模式开关
    sbus_msg->CH6 = (int16_t)(sbus_msg->CH[5]) - 1024;  // 机械爪
    sbus_msg->CH7 = (int16_t)(sbus_msg->CH[6]) - 1024;  // 定深定姿开关
    sbus_msg->CH8 = (int16_t)(sbus_msg->CH[7]) - 1024;  // 横滚微调

    // 连接状态判断
    // 标志字节 bit3=1 表示遥控器与接收机已对频
    sbus_msg->ConnectState = (raw_data[23] >> 3) & 0x01;
}
```

#### 4.7.3 遥控器通道在 ControlTask 中的使用

```c
// ControlTask.c 中的通道映射

// CH1 → X 轴平移（对于 ROV-XE）
RovControlMessage.SpeedX = SbusMessage.CH1 * SpeedLimit;
//                                        ↑ SpeedLimit = 0.8，限速 80%
// CH1 摇杆左右推 → ROV 左右平移

// CH2 → Y 轴前进/后退
RovControlMessage.SpeedY = SbusMessage.CH2 * SpeedLimit;
// CH2 摇杆前后推 → ROV 前进/后退

// CH3 → Z 轴浮潜（深度/垂直）
// 定深模式：累加目标深度
RovControlMessage.PositionZ -= (float)SbusMessage.CH3 / 400;
// 手动模式：直接速度控制
RovControlMessage.SpeedZ = SbusMessage.CH3 / 5 * 2;

// CH4 → 偏航控制
RovControlMessage.AngleYaw += (float)SbusMessage.CH4 / 300;

// CH5 → 模式开关（三段开关）
// >600: 解锁 + 定深定姿
// <-600 或 >900 (异常): 锁定（所有推进器停转）
// 其他: 解锁但无定深

// CH6 → 机械爪控制
// >400 且 <900: 闭合
// <-400 且 >-900: 张开

// CH7 → 定深 + 定姿模式开关
// -100~+100: 仅锁定姿态（PitchRollControl）
// >600: 定深 + 定姿（DeepinControl + PitchRollControl）
// 其他: 全手动

// CH8 → 横滚独立微调（本项目最新修改）
if ((SbusMessage.CH8 > 200 && SbusMessage.CH8 < 900) ||
    (SbusMessage.CH8 < -200 && SbusMessage.CH8 > -900))
    RovControlMessage.SpeedRoll = SbusMessage.CH8;
// 死区 ±200：防止摇杆中位漂移引发误操作
```

---

### 4.8 AHRS 姿态解算

#### 4.8.1 什么是 AHRS？

AHRS（Attitude and Heading Reference System，航姿参考系统）是一种通过融合加速度计、陀螺仪和磁力计数据来计算载体三维姿态（横滚 Roll、俯仰 Pitch、偏航 Yaw）的算法。

**三传感器的角色**：

| 传感器 | 测量内容 | 优势 | 劣势 |
|--------|---------|------|------|
| 陀螺仪 | 角速度 (°/s) | 动态响应快，短时精度高 | 积分漂移，长时间偏航角会偏离 |
| 加速度计 | 重力方向 + 运动加速度 | 长期稳定，无漂移 | 运动加速度干扰，不能测偏航 |
| 磁力计 | 地磁场方向 | 绝对方向参考（指北） | 易受硬铁/软铁干扰，响应慢 |

**核心思想**：陀螺仪提供动态姿态，加速度计和磁力计提供长期修正，两者通过互补滤波器或卡尔曼滤波器融合。

#### 4.8.2 Mahony 互补滤波算法

本项目使用预编译的 `libahrs.a` 姿态解算库，其核心极有可能是 **Mahony 互补滤波算法**（无人机/机器人领域最常用的轻量级姿态解算方法）。

**Mahony 算法原理**：

```text
步骤 1：陀螺仪积分（预测）
  由当前四元数 q 和陀螺仪角速度 ω，积分得到预测姿态 q_pred
  q̇ = 0.5 × q ⊗ ω
  q_pred = q + q̇ × dt

步骤 2：加速度计 + 磁力计修正（观测）
  加速度计测量重力方向 → 可与 q_pred 推导的"理论重力方向"比较
  磁力计测量地磁方向 → 可与 q_pred 推导的"理论地磁方向"比较
  两者误差的叉积 = 需要修正的角速度方向

步骤 3：互补融合
  ω_corrected = ω_gyro + Kp × error + Ki × ∫error·dt
  //                ↑直接测量  ↑比例修正（类似 PID 的 P 和 I）
  //                Kp 和 Ki 决定了修正的快慢（在 libahrs 中已调优）

步骤 4：用修正后的 ω_corrected 更新四元数
  q_new = q + 0.5 × q ⊗ ω_corrected × dt
  q_new = q_new / ||q_new||（归一化，防止数值漂移）
```

**为什么用四元数而不是欧拉角？**
- 欧拉角存在**万向节死锁**（Gimbal Lock）：俯仰角接近 ±90° 时，横滚和偏航无法区分
- 四元数（4 个分量）无死锁，计算效率高（三角函数 → 乘加运算）
- ROV 在水下可能做大角度俯仰机动，死锁是一个真实威胁

#### 4.8.3 姿态数据的使用

```c
// ImuTask 输出的 ImuMsg_t 结构体包含：
typedef struct {
    float roll;    // 横滚角，单位：度，范围 [-180°, +180°]
    float pitch;   // 俯仰角，单位：度
    float yaw;     // 偏航角，单位：度
    float gyro_x;  // 陀螺仪原始角速度 (可选)
    float gyro_y;
    float gyro_z;
    float accel_x; // 加速度计原始加速度 (可选)
    float accel_y;
    float accel_z;
    float temp;    // IMU 温度
} ImuMsg_t;
```

Roll 和 Pitch 用于 PID 闭环的横滚环和俯仰环，Yaw 用于偏航环。角速度可以直接参与 D 项计算（作为误差变化率的补充），但本项目选择用误差的差分近似微分，避免引入角速度的测量噪声。

---

### 4.9 RTAutoInit 自动初始化框架补充

前面 3.4 节已详细分析了原理，这里补充完整的调用链路：

```text
系统上电
  │
  ├─→ Reset_Handler (startup_stm32f405vgtx.s)
  │     └─→ SystemInit()
  │           └─→ main()
  │                 ├─→ HAL_Init()
  │                 ├─→ SystemClock_Config()       // 168MHz
  │                 ├─→ MX_GPIO_Init()             // 所有引脚初始化
  │                 ├─→ MX_DMA_Init()              // 所有 DMA 通道
  │                 ├─→ MX_SPI1_Init()             // SPI1 (BMI088)
  │                 ├─→ MX_SPI2_Init()             // SPI2 (W5500)
  │                 ├─→ MX_I2C1_Init()             // I2C1 (IST8310)
  │                 ├─→ MX_TIM1_Init()             // TIM1 (DSHOT CH1-4)
  │                 ├─→ MX_TIM3_Init()             // TIM3 (DSHOT CH5-8)
  │                 ├─→ MX_TIM4_Init()             // TIM4 (Servo)
  │                 ├─→ MX_TIM5_Init()             // TIM5 (IMU加热)
  │                 ├─→ MX_TIM12_Init()            // TIM12 (探照灯)
  │                 ├─→ MX_ADC1_Init()             // ADC
  │                 ├─→ MX_SDIO_Init()             // SD卡
  │                 ├─→ MX_UART1_Init()            // 预留 RS485
  │                 ├─→ MX_UART2_Init()            // N100
  │                 ├─→ MX_UART3_Init()            // 遥控 RS485
  │                 ├─→ MX_UART6_Init()            // 上位机/蓝牙
  │                 ├─→ MX_USB_DEVICE_Init()       // USB虚拟串口
  │                 │
  │                 ├─→ rt_components_board_init()  // ✨ RTAutoInit Level 1
  │                 │     遍历 __rt_init_start ~ __rt_init_end
  │                 │     调用所有 INIT_BOARD_EXPORT 函数
  │                 │       - 目前可能为空（硬件已在 CubeMX 中初始化）
  │                 │
  │                 ├─→ osKernelInitialize()        // 初始化 FreeRTOS 内核
  │                 │
  │                 ├─→ MX_FREERTOS_Init()
  │                 │     创建 iniTask（最低优先级）
  │                 │
  │                 └─→ osKernelStart()             // ✨ 启动调度器
  │
  ├─→ iniTask 获得 CPU（唯一就绪任务）
  │     └─→ rt_components_init()                   // ✨ RTAutoInit Level 2-6
  │           遍历 __rt_init_start ~ __rt_init_end
  │           调用所有 INIT_PREV/DEVICE/COMPONENT/ENV/APP_EXPORT 函数
  │             - Level 2: RadarTaskInit            (INIT_PREV_EXPORT)
  │             - Level 4: LedTaskInit              (INIT_COMPONENT_EXPORT)
  │             - Level 5: ImuTaskInit, PressureTaskInit,
  │                        RemoteTaskInit, ControlTaskInit,
  │                        MoveTaskInit, ServoTaskInit,
  │                        HostTaskInit, BlueTaskInit, InsTaskInit
  │                        (全部 INIT_ENV_EXPORT)
  │             - Level 6: MessageTaskInit          (INIT_APP_EXPORT)
  │           每个 Init 函数内部调用 osThreadNew() 创建任务
  │     └─→ iniTask 自我删除 (vTaskDelete(NULL))
  │
  └─→ 调度器开始正常轮转，所有 13 个任务并发执行
```

**关键理解**：`rt_components_init()` 在 iniTask 中运行，这意味着它运行在**调度器已经启动之后**。因此 Level 2-6 的初始化函数可以安全地使用 `osThreadNew()`、`osMessageQueueNew()` 等 FreeRTOS API。Level 1 在调度器启动前运行，只能做纯硬件初始化。

---

## 第五章 动手实践指南

### 5.1 推荐代码阅读顺序

不要从头读到尾！按数据流方向阅读效率最高：

```text
第 1 步（30min）：
  读 Core/Inc/main.h        → 了解硬件配置全景
  读 config/ROV_XE/config.h → 了解 PID 参数和机型配置
  读 User/Control/rov.h      → 理解核心数据结构

第 2 步（1h）：
  读 User/Tasks/ControlTask.c  → 控制逻辑和数据融合
  读 User/Tasks/MoveTask.c     → 运动控制和 PID 调用

第 3 步（1h）：
  读 User/Control/pid.c        → PID 算法实现
  读 User/Control/filter.c     → 一阶低通滤波
  读 User/Control/dshot.c      → DSHOT 协议

第 4 步（1h）：
  读 User/Communication/sbus.c → SBUS 协议
  读 User/System/RTAutoInit/autoinit.h → 自动初始化框架

第 5 步（2h）：
  读 User/Sensor/bmi088/ 下的驱动代码 → SPI IMU 驱动
  读 User/Sensor/ms5837/ 下的驱动代码 → 深度传感器驱动
  读 User/Tasks/ImuTask.c  → 了解如何调用 AHRS 库
```

### 5.2 动手实验建议

#### 实验 1：在 PC 上运行 PID 仿真（2 小时）

用 Python 写一个简单的深度控制仿真，直观理解 PID 参数的作用：

```python
import matplotlib.pyplot as plt

class PIDSimulator:
    def __init__(self, kp, ki, kd, max_out=500, max_iout=200):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.max_out = max_out; self.max_iout = max_iout
        self.iout = 0; self.last_error = 0

    def calc(self, setpoint, feedback, dt):
        error = setpoint - feedback
        pout = self.kp * error
        self.iout += self.ki * error * dt
        self.iout = max(-self.max_iout, min(self.max_iout, self.iout))
        dout = self.kd * (error - self.last_error) / dt
        self.last_error = error
        out = pout + self.iout + dout
        return max(-self.max_out, min(self.max_out, out))

# 模拟一个简单的深度系统：depth_dot = k * thrust - buoyancy
def simulate(kp, ki, kd):
    pid = PIDSimulator(kp, ki, kd)
    depth = 0; depth_target = 1000  # 目标 1000mm
    dt = 0.02  # 20ms = MoveTask 控制周期
    depths = []

    for i in range(1000):  # 模拟 20 秒
        thrust = pid.calc(depth_target, depth, dt)
        # 简化动力学模型：加速度 = 推力/质量 - 浮力 - 水阻力
        depth_dot_dot = thrust * 0.01 - 2.0 - depth_dot * 0.5
        depth_dot += depth_dot_dot * dt
        depth += depth_dot * dt
        depths.append(depth)

    plt.plot(depths)
    plt.axhline(y=depth_target, color='r', linestyle='--')
    plt.xlabel("Time (×20ms)")
    plt.ylabel("Depth (mm)")
    plt.show()

# 实验：尝试不同的 PID 参数
simulate(kp=12, ki=0.05, kd=160)   # 本项目参数
simulate(kp=6, ki=0, kd=0)         # 只调 P → 观察稳态误差
simulate(kp=12, ki=0.05, kd=0)     # 不加 D → 观察超调和振荡
simulate(kp=12, ki=0, kd=160)      # 不加 I → 观察是否有稳态误差
```

#### 实验 2：在 STM32 开发板上跑传感器驱动（4 小时）

如果不具备完整 ROV 硬件，至少可以用 STM32F4 开发板 + 独立模块练习：
- 用 SPI 读取 BMI088 原始数据并打印到串口
- 用 I2C 读取 IST8310 磁场数据
- 实现 UART DMA+IDLE 中断接收任意长度数据帧

#### 实验 3：手写一个简化版 RTAutoInit（1 小时）

在一个小型项目中实践自动初始化：
```c
// 在你的 main.c 中定义宏
#define INIT_EXPORT(fn) \
    __attribute__((used, section(".my_init"))) \
    void (*_init_##fn)(void) = fn

// 模块 A 中：
void module_a_init(void) { printf("A init\n"); }
INIT_EXPORT(module_a_init);

// 模块 B 中：
void module_b_init(void) { printf("B init\n"); }
INIT_EXPORT(module_b_init);

// main() 中：
extern void (*__start_my_init)(void);
extern void (*__stop_my_init)(void);
void (**p)(void) = &__start_my_init;
while (p < &__stop_my_init) { (*p++)(); }
```

---

## 第六章 面试准备与项目展示

### 6.1 简历项目描述模板（三档可选）

#### 精简版（简历 2-3 行）

> **ROV-X 水下航行器嵌入式控制系统** | 项目核心开发者 | 2023.10 – 2024.04
> - 基于 STM32F405 + FreeRTOS 构建了 13 任务实时固件系统，实现从传感器采集（IMU/深度/超声波）到 8 路 DSHOT 推进器控制的完整数据管道
> - 设计并实现了 6 自由度 PID 闭环控制算法，完成参数整定与水下实测验证；用 CMake 多配置构建 + 条件编译实现一套代码适配 5 种 ROV 构型

#### 详细版（面试自我介绍 2-3 分钟）

> 我主导开发了一款水下航行器（ROV）的嵌入式控制系统。硬件平台是 STM32F405，跑 FreeRTOS 实时系统。
>
> 软件架构方面，我设计了 13 个 FreeRTOS 任务的数据管道：传感器采集层通过 SPI/I2C/UART 获取 IMU、深度计、超声波等数据并融合成姿态角和位置信息；控制融合层基于遥控器指令和传感器反馈生成运动目标；运动执行层通过推进器混合矩阵和 6 路 PID 闭环控制器驱动 8 个 DSHOT 无刷推进器，同时控制 4 路舵机和探照灯。
>
> 技术上最核心的是 6 自由度 PID 控制器，我对每个自由度（深度/俯仰/横滚/偏航/巡线）进行了独立的参数整定，包括积分限幅（max_iout）防饱和、反馈通道一阶低通滤波（α=0.2~0.5 视对象特性而定）、偏航角 ±180° 归一化处理等细节。
>
> 工程化方面，我借鉴 RT-Thread 设计实现了 RTAutoInit 自动初始化框架——利用 GCC 的 section 属性和链接脚本，让每个模块在编译时自动注册初始化函数并按 6 个优先级排序执行，彻底消除了在 main() 中手动维护初始化顺序的痛点。通过 CMake 多配置构建 + 条件编译宏，同一份 MoveTask.c 可以为 5 种不同推进器构型的 ROV 编译出不同的混合逻辑。

#### STAR 法则版（面试回答框架）

- **Situation**：团队需要一款能在水下稳定悬停和精确操控的 ROV 控制系统，支持 5 种不同构型的机器人
- **Task**：我负责设计并实现完整的嵌入式固件，包括传感器驱动、姿态解算、控制算法和通信协议
- **Action**：
  - 设计了基于 FreeRTOS 的 13 任务架构，通过 Overwrite Queue 实现传感器数据多消费者共享
  - 独立实现了 6 路位置式 PID 控制器，包含积分限幅、低通滤波、角度归一化等工程优化
  - 实现了 RTAutoInit 自动初始化框架，使模块编译时自动注册，消除了手动维护初始化顺序的问题
  - 通过编译宏 + CMake 多配置实现 5 种机型一套代码，降低维护成本
- **Result**：系统稳定运行，定深精度 ±20mm，定姿精度 ±2°，代码可维护性显著提升（新增模块只需加一行宏，无需修改 main()）

### 6.2 高频面试问题与回答要点

#### Q1：PID 三个参数各起什么作用？你的参数是怎么调出来的？

**回答框架**：
1. 先解释三项的物理意义（P=当前误差放大，I=历史误差累积，D=误差变化率预测）
2. 描述工程整定流程：先 P（找到临界振荡点）→ 加 D（抑制超调）→ 加 I（消除稳态误差）
3. 结合水下场景说明特殊考虑：水阻大，P 可以给大；深度不能超调（撞底危险），D 要给大；偏航不需要精确定点，I 给很小

#### Q2：为什么要用 FreeRTOS？裸机不行吗？

**回答要点**：
- 13 个并发任务（传感器采集/控制/通信），裸机中断+大循环无法保证实时性
- IMU 需要 1ms 采样周期，DSHOT 需要 20ms 更新周期，网络需要随时响应
- FreeRTOS 抢占式调度确保高优先级任务（IMU）不被低优先级任务（LED）阻塞
- 用队列解耦任务之间生产消费速率的不匹配

#### Q3：积分饱和是什么？你怎么解决的？

**回答要点**：
1. 定义：执行器饱和时积分项持续累积导致的超调和振荡
2. 举例：ROV 从水面下潜 3 米过程中积分累积到很大，到达深度后无法及时回退
3. 解决方案：积分限幅（`max_iout`），在 `PID_Calc()` 中直接钳位 `Iout`
4. 参数选择：`max_iout = 200`，占总输出 `max_out = 500` 的 40%，确保动态响应不被积分主导

#### Q4：推进器混合矩阵怎么设计的？

**回答要点**：
1. 先画推进器布局图（4 水平 + 4 垂直）
2. 解释混合逻辑：水平面的 4 个推通过不同方向的组合产生 X/Y/Yaw 三自由度
3. 垂直面的 4 个推通过正反转和差分产生 Z/Roll/Pitch
4. PID 闭环输出如何叠加到开环混合上（`+=` 操作）
5. 不同机型的不同混合矩阵通过编译宏切换

#### Q5：多个任务同时需要传感器数据，怎么处理的？

**回答要点**：
1. 使用 FreeRTOS 的 Overwrite Queue（长度=1）+ `xQueuePeek()`（读取不移除）
2. 传感器任务用 `xQueueOverwrite()` 更新，多个消费者任务用 `xQueuePeek()` 读取同一帧
3. 对比：如果用 `xQueueReceive()` 会导致只有第一个消费者读到数据
4. 对比：如果每个消费者一条队列，浪费内存且增加数据不一致风险

#### Q6：RTAutoInit 自动初始化是怎么实现的？

**回答要点**：
1. 先用宏 `INIT_EXPORT(fn, level)` 将函数指针放入特定 linker section（如 `.rti_fn.5`）
2. 链接脚本将所有 `.rti_fn.*` 段按 level 排序放置，导出 `__rt_init_start` 和 `__rt_init_end` 符号
3. 运行时遍历这两个符号之间的所有函数指针并依次调用
4. 优势：新增模块只需要在模块文件加一行宏，完全不需要修改 `main()`

#### Q7：代码中的低通滤波器 `filter(alpha, new, &last)` 是怎么工作的？

**回答要点**：
1. 公式：`y[n] = α·x[n] + (1-α)·y[n-1]`，是一阶 IIR 低通滤波器
2. α=0.5 时截止频率约为采样率/6（20ms 周期时 ≈4Hz）
3. 深度/姿态用 α=0.5（平衡响应和噪声），偏航用 α=0.2（偏航变化慢，可以更平滑）
4. 不滤波直接送 PID 微分项会放大噪声（Kd=160 时尤其严重）

### 6.3 技术深化方向（体现持续学习意愿）

面试官问"如果继续做下去，你会怎么改进"，可以答：

1. **自适应 PID**：根据水深变化自动调节 PID 参数（浅水区阻力小，深水区阻力大）
2. **前馈控制**：将电调/推进器的稳态模型加入前馈通路，减少 PID 负担
3. **故障检测与容错**：单推进器失效后自动重构混合矩阵
4. **ROS2 桥接**：用 W5500 以太网作为 ROS2 节点，实现与上位机的标准化通信
5. **模型预测控制（MPC）**：用 ROV 水动力模型替代 PID，实现更优的轨迹跟踪
6. **SITL/HITL 仿真**：Gazebo 水下仿真 + 硬件在环测试，加速参数整定

---

## 附录：关键文件索引

| 文件 | 内容 | 学习优先级 |
|------|------|-----------|
| `Core/Inc/main.h` | 全硬件配置宏定义 | ★★★★★ |
| `config/ROV_XE/config.h` | PID 参数与机型配置 | ★★★★★ |
| `User/Control/rov.h` | ROV 核心数据结构 | ★★★★★ |
| `User/Tasks/ControlTask.c` | 控制融合任务 | ★★★★★ |
| `User/Tasks/MoveTask.c` | 运动控制 + 混合矩阵 | ★★★★★ |
| `User/Control/pid.c/h` | PID 控制器实现 | ★★★★★ |
| `User/Control/filter.c/h` | 一阶低通滤波器 | ★★★★ |
| `User/Control/dshot.c/h` | DSHOT600 协议 | ★★★★ |
| `User/Communication/sbus.c/h` | SBUS 协议解码 | ★★★★ |
| `User/System/RTAutoInit/autoinit.h` | 自动初始化框架 | ★★★★ |
| `User/Sensor/bmi088/` | BMI088 IMU 驱动 | ★★★ |
| `User/Sensor/ms5837/` | MS5837 深度传感器 | ★★★ |
| `User/Tasks/ImuTask.c` | IMU 采集任务 | ★★★ |
| `User/Tasks/RemoteTask.c` | 遥控接收任务 | ★★★ |
| `Core/Src/freertos.c` | FreeRTOS 初始化 | ★★★ |
| `Core/Inc/FreeRTOSConfig.h` | FreeRTOS 配置 | ★★★ |
| `CMakeLists.txt` + `CMakePresets.json` | 构建系统 | ★★ |
| `readme.md` | 项目文档 + 引脚表 | ★★ |

