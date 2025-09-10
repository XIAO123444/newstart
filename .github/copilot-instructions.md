# Copilot Instructions for smartcarcode

## 项目架构与主要目录

- `CarCode/`：主工程目录，包含核心代码（`code/`）、IAR/MDK 工程文件、输出目录等。
  - `code/`：主要 C 代码，分为各功能模块（如 `buzzer.c/h`, `encoder.c/h`, `menu.c/h`, `screen.c/h` 等）。
  - `iar/`, `mdk/`：分别为 IAR 和 Keil MDK 工程配置及相关脚本。
- `libraries/`：外部依赖和通用库，包括 `zf_common/`（通用工具）、`zf_device/`（设备驱动）、`sdk/`（芯片相关）、`components/`（如 fatfs 文件系统）。

## 关键开发流程

- **编译/烧录/调试**：
  - 推荐使用 IAR 或 Keil MDK 工程文件进行开发。
  - VSCode 下可用 CMSIS 相关 task（如“CMSIS Load”、“CMSIS Run”）配合 pyocd 工具进行烧录和调试。
- **常用命令**：
  - 烧录：`pyocd load --probe cmsisdap: --cbuild-run ...`
  - 擦除：`pyocd erase --probe cmsisdap: --chip ...`
  - 调试：`pyocd gdbserver --probe cmsisdap: --connect attach ...`

## 项目约定与风格

- 头文件引用采用相对路径，常用公共头文件如 `zf_common_headfile.h`。
- 类型定义优先使用 `<stdint.h>`（如 `int16_t`），避免自定义类型名如 `int16`。
- 代码分为功能模块，每个模块有独立的 `.c`/`.h` 文件，接口通过头文件暴露。
- 设备/外设相关代码集中在 `zf_device/`，通用工具在 `zf_common/`。

## 依赖与集成

- 依赖芯片 SDK（`sdk/`）、第三方组件（如 `fatfs`）。
- 通过 pyocd 支持 CMSIS-DAP 调试器。
- 工程配置文件（如 `.csolution.yml`、`.cbuild.yml`）位于 `mdk/` 目录。

## 典型模式示例

- 头文件保护：
  ```c
  #ifndef CODE_SCREEN_H_
  #define CODE_SCREEN_H_
  // ...
  #endif
  ```
- 功能接口暴露：
  ```c
  void show_line(void);
  ```
- 头文件引用：
  ```c
  #include "zf_common_headfile.h"
  #include <stdint.h>
  ```

## 其他说明

- 若遇到头文件找不到，优先检查 `libraries/` 下相关目录及 include 路径。
- 代码风格偏向嵌入式 C，注重模块化与硬件抽象。

---
如需补充具体工作流或有特殊约定，请在此文档补充说明。
