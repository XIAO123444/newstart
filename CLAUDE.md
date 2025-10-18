# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 提供在此代码库中工作的指导说明。

## 项目概述

这是一个基于 MM32F3273G8P 微控制器的嵌入式智能车控制系统。该车是一辆具有视觉循迹功能的自平衡车，使用摄像头 (MT9V03X)、惯性测量单元 (IMU660RA)、编码器、BLDC 负压风扇以及直流电机实现运动控制。

**目标硬件**: MM32F3273G8P (ARM Cortex-M3)
**调试器/烧录器**: CMSIS-DAP 通过 pyocd

## 编译与烧录命令

### 使用 IAR Embedded Workbench
- 打开工程：`CarCode/iar/program/mm32f327x_g8p.ewp`
- 在 IAR IDE 中编译

### 使用 Keil MDK
- 打开工程：`CarCode/mdk/mm32f327x_g8p.uvprojx`
- 在 Keil IDE 中编译

### 使用 CMSIS Toolbox (VSCode)
- 导航到 `CarCode/mdk/` 目录
- 解决方案文件：`mm32f327x_g8p.csolution.yml`
- 使用 VSCode 任务（定义在 `CarCode/mdk/.vscode/tasks.json`）：
  - **烧录**: `pyocd load --probe cmsisdap: --cbuild-run <cbuild-run-file>`
  - **擦除**: `pyocd erase --probe cmsisdap: --chip --cbuild-run <cbuild-run-file>`
  - **调试**: `pyocd gdbserver --probe cmsisdap: --connect attach --persist --reset-run --cbuild-run <cbuild-run-file>`

VSCode 任务 "CMSIS Load"、"CMSIS Run" 和 "CMSIS Erase" 已配置完毕可直接使用。

## 代码架构

### 目录结构

```
CarCode/
├── code/           # 应用模块（平衡、PID、电机、循迹、菜单等）
├── user/           # 主入口和中断服务程序
│   ├── src/main.c  # 主函数初始化和控制循环
│   └── src/isr.c   # 所有中断服务函数
├── iar/            # IAR 工程文件
└── mdk/            # Keil MDK 和 CMSIS 构建文件

libraries/
├── zf_common/      # 通用工具（时钟、调试、FIFO 等）
├── zf_device/      # 高层设备驱动（摄像头、显示屏、传感器）
├── zf_driver/      # 底层硬件驱动（GPIO、PWM、UART、SPI 等）
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

- **CarCode/code/PID.c/h**: 多种 PID 实现（位置式、增量式、陀螺仪补偿）
- **CarCode/code/motor.c/h**: 直流电机控制（方向 + PWM）
- **CarCode/code/BLDC.c/h**: 无刷电机控制，用于负压风扇
- **CarCode/code/Balance.c/h**: 自平衡算法
- **CarCode/code/track.c/h**: 循迹和赛道元素检测
- **CarCode/code/photo_chuli.c/h**: 摄像头图像处理
- **CarCode/code/menu.c/h**: LCD 菜单系统，用于参数调节
- **CarCode/code/encoder.c/h**: 车轮编码器读取
- **CarCode/code/key.c/h**: 物理按键输入
- **CarCode/code/lora3a22.c/h**: 通过 LoRa 无线电进行远程控制

### 中断架构 (isr.c)

- **TIM6** (1ms)：主控制循环 - 平衡、速度、转向 PID
- **TIM7**: 按键消抖
- **TIM8**: TSL1401 线性传感器采集
- **UART2**: GPS 数据
- **UART6**: LoRa 远程控制
- **UART8**: 摄像头数据
- **EXTI6**: 摄像头场同步信号
- **DMA1_CH4**: 摄像头 DMA 传输

### 设备库组织

**libraries/zf_device/** 包含高层驱动：
- `zf_device_ips200.*`: IPS200 LCD 显示屏 (ST7789)
- `zf_device_mt9v03x.*`: MT9V03X 摄像头模块
- `zf_device_imu660ra.*`: IMU660RA 6 轴传感器
- `zf_device_camera.*`: 通用摄像头接口
- 设备驱动调用底层的 `zf_driver/` 函数

**libraries/zf_driver/** 包含 MCU 外设驱动：
- GPIO, PWM, UART, SPI, I2C, ADC, Timer, Encoder, DMA 等

### 编码规范

1. **包含顺序**：始终首先包含 `"zf_common_headfile.h"`
2. **数据类型**：使用 stdint 类型（`int16_t`, `uint32_t`），而非自定义类型如 `int16`
3. **模块结构**：每个模块都有 `.c`/`.h` 文件对，接口通过头文件暴露
4. **头文件保护**：格式为 `CODE_<MODULE>_H_` 或 `_<MODULE>_H`

## 常见开发工作流程

### 调节 PID 参数

PID 参数定义在 `CarCode/code/PID.c` 中：
- `PID_gyro`: 角速度环（Kp=0.03, Ki=0.001）
- `PID_angle`: 角度环（Kp=1, Ki=0）
- `PID_speed`: 速度环（Kp=2, Ki=0.009）
- `PID_steer`: 转向环（可通过菜单配置）
- `PID_BLDC`: BLDC 风扇控制

菜单系统 (menu.c/h) 允许通过 LCD 和按键进行运行时调整。

### 添加新的控制功能

1. 在 `CarCode/code/` 中添加模块，包含 `.c`/`.h` 文件
2. 在 `CarCode/user/src/main.c` 的初始化函数 (`all_init()`) 中包含
3. 在 `CarCode/user/src/isr.c` 的相应 ISR 中添加控制逻辑（通常是 TIM6）
4. 如需要，通过菜单系统暴露参数

### 理解车辆模式

在 `menu.h` 中定义为 `car_mode` 枚举：
- `stop`: 电机和风扇停止
- `remote`: 通过 LoRa 远程控制
- `Start_Calibrate`/`Now_Calibrate`: BLDC 电调校准序列
- `car_run_mode1`/`car_run_mode2`: 自主驾驶模式

模式切换通常通过按键输入或远程命令发生。

### 摄像头与循迹

1. 摄像头 (MT9V03X) 在场同步信号触发中断 (EXTI6)
2. DMA 传输图像数据
3. 主循环中的 `photo_image_process_all()` 处理图像：
   - 阈值化（OTSU 算法）
   - 边缘检测
   - 中线计算
4. `element_check()` 检测赛道特征（十字路口、弯道、坡道、斑马线）
5. `S_PID_CAL()` 根据中线更新转向 PID

### Flash 存储

`CarCode/code/flash.c/h` 处理参数持久化：
- `flash_load_config_default()`: 加载保存的参数
- 保存的参数包括 PID 增益、BLDC 设置和赛道元素处理配置

## 重要注意事项

- **所有控制均在中断中运行**：主循环仅处理菜单和图像处理
- **时序关键**：平衡环为 1ms，任何延迟都可能导致不稳定
- **单位**：编码器值通常为每个采样周期的计数
- **安全机制**：`start_count` 提供启动延迟（约 5 秒）后才完全激活控制
- **停车条件**：监控多种停车条件（抬起检测、斑马线、定时器、远程停止）
