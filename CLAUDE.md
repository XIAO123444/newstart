# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 提供在此代码库中工作的指导说明。

## 项目概述

这是一个基于 MM32F3273G8P 微控制器的嵌入式智能车控制系统。该车是一辆具有视觉循迹功能的自平衡车，使用摄像头 (MT9V03X)、惯性测量单元 (IMU660RA)、编码器、BLDC 负压风扇以及直流电机实现运动控制。

**目标硬件**: MM32F3273G8P (ARM Cortex-M3, 120MHz)
**调试器/烧录器**: CMSIS-DAP 通过 pyocd
**代码规模**: 应用代码约 4,825 行 + 库文件

### 项目结构说明

本代码仓库包含两个主要项目目录：
- **CarCode/**: 原始智能车控制系统
- **newcarcode/**: 新版智能车控制系统（包含更完善的 CMSIS Toolbox 支持）

两个项目在功能模块上完全相同，但 newcarcode 提供了更现代的编译工具链配置。本文档主要针对两个项目通用的架构和模块进行说明。

## 编译与烧录命令

### 使用 IAR Embedded Workbench
**CarCode 项目**:
- 打开工程：`CarCode/iar/program/mm32f327x_g8p.ewp`
- 在 IAR IDE 中编译

**newcarcode 项目**:
- 打开工程：`newcarcode/iar/program/mm32f327x_g8p.ewp`
- 在 IAR IDE 中编译

### 使用 Keil MDK
**CarCode 项目**:
- 打开工程：`CarCode/mdk/mm32f327x_g8p.uvprojx`
- 在 Keil IDE 中编译

**newcarcode 项目**:
- 打开工程：`newcarcode/mdk/mm32f327x_g8p.uvprojx`
- 在 Keil IDE 中编译

### 使用 CMSIS Toolbox (VSCode) - 推荐用于 newcarcode
- 导航到 `newcarcode/mdk/` 目录
- 解决方案文件：`mm32f327x_g8p.csolution.yml`
- 项目文件：`mm32f327x_g8p.cproject.yml`
- 编译器：ARM Compiler 6 (AC6 @6.22.0)
- 优化级别：debug

#### 命令行操作
```bash
# 编译项目
cd newcarcode/mdk/
cbuild mm32f327x_g8p.cproject.yml

# 烧录到目标板
pyocd load --probe cmsisdap: build/mm32f327x_g8p/Release/mm32f327x_g8p.elf

# 擦除 Flash
pyocd erase --probe cmsisdap: --chip

# 启动 GDB 调试服务器
pyocd gdbserver --probe cmsisdap: --connect attach --persist --reset-run
```

#### VSCode 任务
VSCode 任务 "CMSIS Load"、"CMSIS Run" 和 "CMSIS Erase" 已配置完毕可直接使用（定义在 `newcarcode/mdk/.vscode/tasks.json`）。

## 代码架构

### 目录结构

```
CarCode/ 或 newcarcode/
├── code/           # 应用模块（17个模块，平衡、PID、电机、循迹、菜单等）
│   ├── Balance.c/h          # 自平衡算法和 IMU 数据滤波
│   ├── BLDC.c/h             # 无刷电机（负压风扇）控制
│   ├── buzzer.c/h           # 蜂鸣器控制
│   ├── encoder.c/h          # 车轮编码器读取
│   ├── Filter.c/h           # 滤波算法框架
│   ├── flash.c/h            # 参数持久化存储（4个配置槽位）
│   ├── INS.c/h              # 惯导系统结构定义
│   ├── key.c/h              # 物理按键扫描（4个按键）
│   ├── lora3a22.c/h         # LoRa 远程无线控制
│   ├── menu.c/h             # LCD 菜单系统（参数调节）
│   ├── motor.c/h            # 直流电机控制（PWM + 方向）
│   ├── photo_chuli.c/h      # 摄像头图像处理（OTSU、边界检测）
│   ├── PID.c/h              # 5种 PID 实现（位置式、增量式、陀螺仪补偿）
│   ├── screen.c/h           # 屏幕显示辅助函数
│   ├── speed.c/h            # 速度控制模块
│   ├── steer_pid.h          # 转向 PID 头文件
│   └── streer_pid.c         # 转向 PID 实现（注：文件名拼写错误）
│
├── user/           # 主入口和中断服务程序
│   ├── inc/isr.h           # 中断服务程序头文件
│   └── src/
│       ├── main.c          # 主函数初始化和控制循环
│       └── isr.c           # 所有中断处理器实现（10+ 个中断）
├── iar/            # IAR 工程文件
└── mdk/            # Keil MDK 和 CMSIS 构建文件

libraries/
├── zf_common/      # 通用工具（时钟、调试、FIFO、字体等，7个文件）
├── zf_device/      # 高层设备驱动（摄像头、显示屏、传感器等，30+ 设备）
├── zf_driver/      # 底层硬件驱动（GPIO、PWM、UART、SPI 等，15+ 驱动）
├── sdk/            # 芯片专用 SDK 和 CMSIS 头文件
└── components/     # 第三方组件（fatfs 文件系统）
```

### 控制系统架构

智能车使用**串级控制回路**，运行在定时器中断中（`isr.c` 中的 TIM6_IRQHandler）：

1. **平衡控制**（最内层，最快）：
   - **角度环** (5ms)：IMU 角度 → PID_angle → 角速度目标值
   - **陀螺仪环** (3ms)：陀螺仪 → PID_gyro → BLDC 风扇占空比
   - BLDC 风扇提供负压下压力以保持车体平衡

2. **速度控制** (5ms)：
   - 编码器（左右轮）→ PID_speed → 电机功率基准值

3. **转向控制**（每 1ms）：
   - 摄像头循迹 → PID_steer → 差速电机控制

4. **电机输出**：
   - 左电机：`PID_speed.out - PID_steer.out`
   - 右电机：`PID_speed.out + PID_steer.out`

### 关键模块

#### **控制系统核心**
- **code/PID.c/h**: 5种 PID 实现（位置式、增量式、陀螺仪补偿）
  - `PID_gyro`: 角速度环（Kp=0.03, Ki=0.001, maxout=80）
  - `PID_angle`: 角度环（Kp=1, Ki=0, maxout=5000）
  - `PID_speed`: 速度环（Kp=2, Ki=0.009, maxout=1500）
  - `PID_steer`: 转向环（Kp=0, Ki=0, Kd2=0.1, maxout=5000）
  - `PID_BLDC`: BLDC 风扇控制（Kp=40, Ki=0.8, Kd2=15, maxout=3000）
  - 函数接口：`PID_update()`, `increment_pid_update()`, `PID_gyro_update()`, `PID_calculate()`

- **code/motor.c/h**: 直流电机控制（方向 + PWM）
  - 左电机：PWM on TIM5_CH4 (A3), 方向 GPIO (A2)
  - 右电机：PWM on TIM5_CH2 (A1), 方向 GPIO (A0)
  - PWM 频率：17kHz
  - 函数：`motor_init()`, `motor_run()`, `motor()`, `Motor_SetLeftSpeed()`, `Motor_SetRightSpeed()`

- **code/BLDC.c/h**: 无刷电机控制，用于负压风扇
  - 使用 TIM2 PWM (CH1 on A15, CH2 on B3)
  - PWM 周期：10ms
  - 参数结构：`basic_duty=400`, `encoder_p=5`, `max_output=600`, `min_output=-400`
  - 函数：`BLDC_init()`, `BLDC_run()`, `BLDC_unlock_UP/DOWN()`, `BLDC_STOP()`

- **code/Balance.c/h**: 自平衡算法
  - 一阶低通滤波角度处理
  - IMU 数据滤波和校准
  - 陀螺仪积分以获得姿态角
  - 函数：`first_order_filtering()`, `imu_filter()`, `angle_init()`, `lift_protection()`

- **code/encoder.c/h**: 车轮编码器读取
  - 使用 TIM3 和 TIM4 正交编码器接口
  - 变量：`encoder_L/R`, `encoder_L_d/R_d` (差分值), `encoder_L_last/R_last`
  - 函数：`Encoder_Init()`, `Encoder_GetInfo_L/R()`

#### **视觉与循迹**
- **code/track.c/h**: 循迹和赛道元素检测
  - 赛道元素检测（十字路口、岛、斜坡、弯道等）
  - 中线计算
  - 函数：`element_check()`, `output_middle2/3/4()`, `output_addspeedflag()`

- **code/photo_chuli.c/h**: 摄像头图像处理
  - OTSU 自适应阈值化
  - 边界检测和边界延长
  - 特征点检测
  - 函数：`photo_image_process_all()`, `photo_displayimage()`, `set_b_imagine()`, `image_boundary_process()`
  - 特征点函数：`Find_Down_Point()`, `Find_Up_Point()` 等

- **code/streer_pid.c/h**: 转向 PID（注：文件名拼写错误）
  - 基于摄像头中线输出的转向控制
  - 包含陀螺仪补偿
  - 函数：`S_PID_CAL()` - 计算转向 PID，集成陀螺仪反馈

#### **用户界面与配置**
- **code/menu.c/h**: LCD 菜单系统，用于参数调节
  - 多层级菜单结构（4层深度）
  - 支持参数类型：int8/16/32, uint8/16/32, float, double
  - 菜单模式：normal, edit_int/float, edit_confirm, special_show_element1, stop_debug_display
  - 菜单项结构：`MENU` 包含优先级、文本、位置、参数指针、类型、操作函数
  - 车辆模式：stop, remote, Start_Calibrate, Now_Calibrate, car_run_mode1/2
  - 停止原因记录：timer_count_stop, blackprotect_stop, zebra_stop, liftup_stop, remotestop 等

- **code/flash.c/h**: 参数持久化存储
  - 4个参数配置槽位
  - 函数：`flash_save_config_1/2/3/4()`, `flash_load_config_1/2/3/4()`, `flash_load_config_default()`, `flash_reset()`
  - 保存的参数包括 PID 增益、BLDC 设置和赛道元素处理配置

- **code/key.c/h**: 物理按键输入
  - 4个按键输入 (E3, E2, E4, E5)
  - TIM7 (1ms) 定时扫描
  - 消抖处理 (100ms 延迟)
  - 函数：`Key_init()`, `Key_Scan()`

#### **其他模块**
- **code/lora3a22.c/h**: 通过 LoRa 无线电进行远程控制
  - 函数：`remote_check_stoponly()`, `remote_speed_control()`, `remote_param_init()`

- **code/speed.c/h**: 速度控制模块
  - 函数：`Velocity_Control()` - 速度反馈控制

- **code/buzzer.c/h**: 蜂鸣器模块
  - 函数：`BUZZ_init()`, `BUZZ_cycle()`, `BUZZ_START()`

- **code/screen.c/h**: 屏幕显示辅助函数
  - 函数：`show_line()` - 显示循迹中线

- **code/INS.c/h**: 惯导系统结构定义
  - 结构体：`struct_INS` 包含 `intg_encoder`（里程数）和 `intg_gyro_yaw`（偏航角）

- **code/Filter.c/h**: 滤波算法框架（当前仅包含头文件）

### 中断架构 (isr.c)

系统使用多个中断处理器实现实时控制和数据采集：

| 中断源 | 周期/触发 | 功能描述 | 文件位置 |
|------|----------|---------|---------|
| **TIM6** | 1ms | 主控制循环 - 平衡、速度、转向 PID；BLDC 控制 | user/src/isr.c |
| **TIM7** | 1ms | 按键消抖处理 | user/src/isr.c |
| **TIM8_UP** | 可配 | TSL1401 线性传感器采集 | user/src/isr.c |
| **UART2** | 异步 | GPS 数据接收 (gps_uart_callback) | user/src/isr.c |
| **UART6** | 异步 | LoRa 远程控制 (lora3a22_uart_callback) | user/src/isr.c |
| **UART8** | 异步 | 摄像头数据接收 (camera_uart_handler) | user/src/isr.c |
| **EXTI6** | 异步 | 摄像头场同步信号 (camera_vsync_handler) | user/src/isr.c |
| **DMA1_CH4** | 异步 | 摄像头 DMA 传输完成 (camera_dma_handler) | user/src/isr.c |
| **EXTI10** | 可配 | DM1XA 光信号 (dm1xa_light_callback) | user/src/isr.c |
| **EXTI11** | 可配 | DM1XA 声信号 (dm1xa_sound_callback) | user/src/isr.c |

#### TIM6 主控制循环详细逻辑 (1ms 周期)

```
1. BLDC 校准状态机处理（电调校准序列）
2. S_PID_CAL() - 转向 PID 计算（每 1ms）
3. IMU 数据获取与滤波（每 1ms）
4. 每 5ms 执行：
   - 编码器读取（左右轮）
   - 速度 PID 更新
5. 每 5ms 执行：
   - 一阶滤波处理
   - 角度 PID 更新
6. 每 3ms 执行：
   - 陀螺仪 PID 更新
7. 启动阶段计时（0-5000ms）：
   - 3000ms: BLDC 风扇启动
   - 5000ms: 电机启动、PID 清零
   - 50000ms: 超时停车（50秒运行限制）
8. 远程控制模式处理
9. 停车模式处理（BLDC_STOP + motor(0,0)）
```

**时序关系**：
- 转向环：1ms（最快）
- 陀螺仪环：3ms
- 角度环/速度环：5ms

### 设备库组织

**libraries/zf_device/** 包含高层驱动（30+ 设备）：
- **摄像头**：`zf_device_mt9v03x.*` (MT9V03X)、`zf_device_ov7725.*`、`zf_device_scc8660.*`、`zf_device_camera.*` (通用接口)
- **显示屏**：`zf_device_ips200.*` (IPS200 LCD, ST7789)、`zf_device_ips114.*`、`zf_device_tft180.*`、`zf_device_oled.*`
- **IMU 传感器**：`zf_device_imu660ra.*` (IMU660RA 6轴)、`zf_device_imu963ra.*`、`zf_device_icm20602.*`、`zf_device_mpu6050.*`
- **通信模块**：`zf_device_gps_tau1201.*` (GPS)、`zf_device_bluetooth_ch9141.*`、`zf_device_wireless_uart.*`、`zf_device_lora3a22.*` (LoRa)
- **其他传感器**：`zf_device_tsl1401.*` (线性传感器)、`zf_device_dm1xa.*` (光/声传感器)、`zf_device_dl1a.*`
- **存储**：`zf_device_sdcard.*`
- **输入**：`zf_device_key.*`
- 设备驱动调用底层的 `zf_driver/` 函数

**libraries/zf_driver/** 包含 MCU 外设驱动（15+ 驱动）：
- **基础 IO**：GPIO, EXTI (外部中断)
- **定时器**：Timer, PIT (周期中断定时器), Encoder (正交编码器)
- **通信**：UART, SPI, I2C, Soft_IIC (软件 I2C)
- **模拟**：ADC, DAC
- **PWM**：PWM 输出
- **存储**：Flash, SDIO
- **DMA**：DMA 传输控制

### 编码规范

1. **包含顺序**：始终首先包含 `"zf_common_headfile.h"`
2. **数据类型**：
   - 推荐使用 stdint 类型：`int8_t`, `uint8_t`, `int16_t`, `uint16_t`, `int32_t`, `uint32_t`
   - 也支持自定义类型别名：`int8`, `uint8`, `int16`, `uint16`, `int32`, `uint32`（定义在 `zf_common_typedef.h`）
3. **模块结构**：每个模块都有 `.c`/`.h` 文件对，接口通过头文件暴露
4. **头文件保护**：格式为 `CODE_<MODULE>_H_` 或 `_<MODULE>_H`
5. **已知问题**：转向 PID 源文件命名有拼写错误（`streer_pid.c` 应为 `steer_pid.c`）

## 常见开发工作流程

### 调节 PID 参数

PID 参数定义在 `code/PID.c` 中，有 5 个独立的 PID 实例：

#### PID 实例默认值
```c
PID_gyro:  Kp=0.03,  Ki=0.001, Kd=0,  Kd2=0,  maxout=80,   minout=0     // 陀螺仪环
PID_angle: Kp=1,     Ki=0,     Kd=0,  Kd2=0,  maxout=5000, minout=-5000 // 角度环
PID_speed: Kp=2,     Ki=0.009, Kd=0,  Kd2=0,  maxout=1500, minout=-1500 // 速度环
PID_steer: Kp=0,     Ki=0,     Kd=0,  Kd2=0.1,maxout=5000, minout=-5000 // 转向环
PID_BLDC:  Kp=40,    Ki=0.8,   Kd=0,  Kd2=15, maxout=3000, minout=-3000 // BLDC 风扇
```

#### 调参方法
1. **运行时调节**：通过菜单系统 (menu.c/h) 使用 LCD 和按键进行实时调整
2. **持久化保存**：使用 Flash 模块保存到 4 个配置槽位之一
3. **加载配置**：启动时调用 `flash_load_config_default()` 加载保存的参数

菜单系统支持以下参数类型的在线编辑：
- 整数：int8/16/32, uint8/16/32
- 浮点：float, double

### 添加新的控制功能

1. 在 `code/` 目录中添加模块，包含 `.c`/`.h` 文件
2. 在 `user/src/main.c` 的初始化函数 (`all_init()`) 中包含初始化调用
3. 在 `user/src/isr.c` 的相应 ISR 中添加控制逻辑：
   - 实时控制逻辑：通常添加到 **TIM6_IRQHandler** (1ms 周期)
   - 按键响应：添加到 **TIM7_IRQHandler**
   - 传感器数据采集：添加到对应的 UART/DMA 中断
4. 如需要，通过菜单系统暴露参数（在 `menu.c` 中添加 `MENU` 结构体）
5. 更新编译配置（如果需要）：
   - Keil: 在 `mdk/mm32f327x_g8p.uvprojx` 中添加文件
   - IAR: 在 `iar/program/mm32f327x_g8p.ewp` 中添加文件
   - CMSIS: 在 `mdk/mm32f327x_g8p.cproject.yml` 的 `groups` 中添加文件

### 理解车辆模式

在 `menu.h` 中定义为 `car_mode` 枚举：
- `stop`: 电机和风扇停止（安全状态）
- `remote`: 通过 LoRa 远程控制模式
- `Start_Calibrate`: BLDC 电调校准开始
- `Now_Calibrate`: BLDC 电调校准进行中
- `car_run_mode1`: 自主驾驶模式1
- `car_run_mode2`: 自主驾驶模式2

#### BLDC 电调校准流程
电调校准在 TIM6 中断中的状态机处理：
```
1. 进入 Start_Calibrate 模式
2. 0-2000ms: BLDC 输出高电平（校准上升沿）
3. 2000-3000ms: BLDC 输出低电平（校准下降沿）
4. 3000ms: 校准完成，自动切换到 stop 模式
```

#### 自主驾驶启动序列
进入 car_run_mode 后的启动时序（在 TIM6 中断中）：
```
0-3000ms:    BLDC 风扇启动
3000-5000ms: 运行准备阶段
5000ms:      电机启动、PID 积分清零、屏幕清除
5001-50000ms: 正常自主驾驶运行
50000ms+:    超时停车（stopdebug = timer_count_stop）
```

模式切换方式：
- 按键输入（通过菜单选择）
- 远程命令（通过 LoRa）
- 自动状态机切换（如校准完成后自动进入 stop）

### 摄像头与循迹

#### 摄像头数据采集流程
1. 摄像头 (MT9V03X) 在场同步信号触发中断 (EXTI6)
2. DMA1_CH4 传输图像数据到内存缓冲区
3. DMA 传输完成后触发 DMA1_CH4 中断

#### 图像处理流程（主循环中执行）
在 `main()` 主循环中调用 `photo_image_process_all()` 处理图像：
1. **阈值化**：OTSU 自适应阈值算法将灰度图转为二值图
2. **边缘检测**：检测赛道左右边界
3. **边界延长**：处理不完整的边界线
4. **中线计算**：基于左右边界计算赛道中线
5. **特征点检测**：检测关键特征点（`Find_Down_Point()`, `Find_Up_Point()` 等）

#### 赛道元素检测
`element_check()` 函数检测以下赛道特征：
- 十字路口
- 环岛（左环岛/右环岛）
- 斜坡
- 弯道（急弯/缓弯）
- 斑马线（停车标志）
- 加速区域

#### 转向控制
1. `S_PID_CAL()` 在 TIM6 中断中每 1ms 调用一次
2. 基于摄像头计算的中线偏差计算转向 PID
3. 集成陀螺仪反馈进行姿态补偿
4. 输出差速控制信号到电机

### Flash 存储

`code/flash.c/h` 处理参数持久化存储：

#### 存储功能
- **4 个配置槽位**：支持保存 4 套不同的参数配置
- **保存函数**：`flash_save_config_1()`, `flash_save_config_2()`, `flash_save_config_3()`, `flash_save_config_4()`
- **加载函数**：`flash_load_config_1()`, `flash_load_config_2()`, `flash_load_config_3()`, `flash_load_config_4()`
- **默认配置**：`flash_load_config_default()` - 在启动时调用（`main.c` 的 `all_init()` 中）
- **重置功能**：`flash_reset()` - 恢复出厂设置

#### 保存的参数包括
- **PID 增益**：5 个 PID 实例的 Kp, Ki, Kd, Kd2, maxout, minout
- **BLDC 设置**：basic_duty, encoder_p, max_output, min_output
- **赛道元素处理配置**：各种元素检测的阈值和参数
- **其他控制参数**：速度目标值、转向补偿系数等

#### 使用流程
1. 通过菜单系统调整参数
2. 选择保存到某个槽位（1-4）
3. 参数写入 Flash 非易失性存储
4. 下次上电时自动加载

## 重要注意事项

### 系统架构特点
- **所有控制均在中断中运行**：主循环仅处理菜单和图像处理（非实时任务）
- **时序关键**：平衡环最内层为 1ms，任何延迟都可能导致不稳定
- **混合架构**：中断驱动的实时控制 + 轮询式的图像处理

### 时序与单位
- **编码器值**：通常为每个采样周期（5ms）的计数
- **角度单位**：陀螺仪输出的角速度需积分转换为角度
- **控制周期**：
  - 转向环：1ms
  - 陀螺仪环：3ms
  - 角度环/速度环：5ms

### 安全机制
- **启动延迟**：`start_count` 提供约 5 秒启动延迟后才完全激活控制
  - 0-3000ms: BLDC 风扇启动
  - 3000-5000ms: 准备阶段
  - 5000ms: 电机启动、PID 清零
- **停车条件**：监控多种停车条件并记录原因
  - `timer_count_stop`: 超时停车（50秒）
  - `blackprotect_stop`: 黑线保护
  - `zebra_stop`: 斑马线停车
  - `liftup_stop`: 抬起检测停车
  - `remotestop`: 远程停止命令
  - `gyro_intrg_pitch_stop`: 俯仰积分过大
  - `gyro_intrg_roll_stop`: 侧滚积分过大
  - `gyro_intrg_yaw_stop`: 偏航积分过大

### 硬件连接
- **电机**：
  - 左电机：PWM on TIM5_CH4 (PA3), 方向 (PA2)
  - 右电机：PWM on TIM5_CH2 (PA1), 方向 (PA0)
- **BLDC 风扇**：TIM2 PWM (CH1 on PA15, CH2 on PB3)
- **编码器**：TIM3 (左轮), TIM4 (右轮)
- **按键**：PE3, PE2, PE4, PE5
- **IMU**：IMU660RA 通过 SPI 接口
- **摄像头**：MT9V03X 通过 UART8 + DMA
- **LoRa**：通过 UART6
- **GPS**：通过 UART2

### 初始化顺序（main.c 中的 all_init()）
```
1. clock_init(SYSTEM_CLOCK_120M)  - 系统时钟 120MHz
2. debug_init()                    - 调试初始化
3. imu660ra_init()                 - IMU 初始化
4. Encoder_Init()                  - 编码器 + TIM6 1ms 定时器
5. Menu_Screen_Init()              - 屏幕初始化
6. Key_init()                      - 按键 + TIM7 1ms 定时器
7. BUZZ_init()                     - 蜂鸣器初始化
8. motor_init()                    - 电机初始化（17kHz PWM）
9. BLDC_init()                     - BLDC 初始化（10ms PWM）
10. lora3a22_init()                - LoRa 初始化
11. remote_param_init()            - 远程参数初始化
12. flash_load_config_default()    - 加载保存的参数
13. MT9V03X 摄像头初始化循环       - 重试直到成功
```

### 调试技巧
- **菜单系统**：通过 LCD 屏幕实时查看和调整所有参数
- **停止原因显示**：菜单模式 `stop_debug_display` 可查看停车原因
- **赛道元素显示**：菜单模式 `special_show_element1` 可查看检测到的赛道元素
- **图像显示**：主循环中可调用 `photo_displayimage()` 在 LCD 上显示处理后的图像
