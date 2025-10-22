# Flash 参数系统升级说明

## 概述

本次升级重新设计了 Flash 参数保存系统，解决了以下问题：
1. **消除了内存共用冲突**：原代码索引 24-27 被重复使用导致数据覆盖
2. **参数顺序更合理**：按模块分组，易于维护
3. **自动初始化新参数**：添加新参数时不会导致程序卡死

## 主要改进

### 1. 新的参数布局（无内存冲突）

所有参数都有独立的 Flash 索引位置：

| 索引范围 | 参数模块 | 参数数量 | 说明 |
|---------|---------|---------|------|
| 0-4 | PID_gyro（陀螺仪环） | 5个 | kp, ki, kd, maxout, minout |
| 5-9 | PID_angle（角度环） | 5个 | kp, ki, kd, maxout, minout |
| 10-15 | PID_speed（速度环） | 6个 | kp, ki, kd, maxout, minout, targ |
| 16-21 | PID_steer（转向环） | 6个 | kp, ki, kd, kd2, maxout, minout |
| 22-27 | PID_BLDC（BLDC环） | 6个 | kp, ki, kd, kd2, maxout, minout |
| 28-30 | 前瞻参数 | 3个 | forwardsight, forwardsight2, forwardsight3 |
| 31-34 | BLDC参数 | 4个 | basic_duty, encoder_p, max_output, min_output |
| 35-36 | 元数据 | 2个 | 版本号, 参数计数 |

**总计：37 个参数位置**

### 2. 版本管理机制

在 `flash.h` 中定义了两个关键宏：

```c
#define FLASH_PARAM_VERSION     1       // 参数版本号，修改参数结构时增加此值
#define FLASH_PARAM_COUNT       37      // 当前保存的参数总数（修改时更新此值）
```

- **FLASH_PARAM_VERSION**：用于标识参数结构的版本，未来可扩展兼容性检查
- **FLASH_PARAM_COUNT**：记录当前代码中定义的参数总数

### 3. 自动初始化新参数的机制

#### 问题场景
原来的问题：如果你有 28 个已保存的参数，现在添加 2 个新参数，启动时调用 `flash_load_config()` 会读取未初始化的 Flash 区域（索引 28-29），可能包含随机数据，导致程序卡死。

#### 解决方案
新的 `flash_load_config()` 函数会自动检测并初始化新参数：

```c
void flash_load_config(int16 i)
{
    uint32 saved_version = 0;
    uint32 saved_param_count = 0;

    flash_buffer_clear();
    flash_read_page_to_buffer(100 + i/4, i%4);

    // 读取 Flash 中保存的参数数量
    saved_version = flash_union_buffer[FLASH_IDX_VERSION].uint32_type;
    saved_param_count = flash_union_buffer[FLASH_IDX_PARAM_COUNT].uint32_type;

    // 检测是否有新增参数
    if(saved_param_count < FLASH_PARAM_COUNT)
    {
        // 自动将新增参数的区域清零
        for(uint32 idx = saved_param_count; idx < FLASH_PARAM_COUNT - 2; idx++)
        {
            flash_union_buffer[idx].uint32_type = 0;
        }
    }

    // 然后正常加载所有参数...
}
```

**工作流程：**
1. 从 Flash 读取 `saved_param_count`（上次保存时的参数数量，例如 28）
2. 对比当前代码的 `FLASH_PARAM_COUNT`（例如新增后为 30）
3. 如果 `28 < 30`，则将索引 28-29 的 Flash buffer 清零
4. 这样加载新参数时会得到 0，而不是随机值

**优势：**
- 无需手动删除 `flash_load_config_default()` 调用
- 无需先运行一次 `flash_save_config()` 来初始化
- 首次启动即可安全加载，新参数自动为 0

### 4. 使用宏定义索引

所有参数索引都在 `flash.h` 中使用宏定义，避免魔术数字：

```c
// ====== PID_gyro 陀螺仪环 (索引 0-4) ======
#define FLASH_IDX_GYRO_KP       0
#define FLASH_IDX_GYRO_KI       1
#define FLASH_IDX_GYRO_KD       2
#define FLASH_IDX_GYRO_MAXOUT   3
#define FLASH_IDX_GYRO_MINOUT   4

// ====== 元数据 (索引 35-36) ======
#define FLASH_IDX_VERSION       35
#define FLASH_IDX_PARAM_COUNT   36
```

在代码中使用：
```c
// 保存
flash_union_buffer[FLASH_IDX_GYRO_KP].float_type = PID_gyro.kp;

// 加载
PID_gyro.kp = flash_union_buffer[FLASH_IDX_GYRO_KP].float_type;
```

## 如何添加新参数

### 步骤说明

假设你要添加 2 个新参数 `new_param1` 和 `new_param2`：

#### 1. 在 flash.h 中添加索引定义

```c
// 在现有定义后添加
#define FLASH_IDX_NEW_PARAM1    37
#define FLASH_IDX_NEW_PARAM2    38

// 更新参数总数
#define FLASH_PARAM_COUNT       39      // 从 37 改为 39
```

#### 2. 在 flash.c 的 flash_save_config() 中添加保存代码

在函数末尾、写入 Flash 之前添加：

```c
void flash_save_config(int16 i)
{
    // ... 现有代码 ...

    // ====== 新增参数 (索引 37-38) ======
    flash_union_buffer[FLASH_IDX_NEW_PARAM1].float_type = new_param1;
    flash_union_buffer[FLASH_IDX_NEW_PARAM2].float_type = new_param2;

    // ====== 元数据 (索引 35-36) ======
    flash_union_buffer[FLASH_IDX_VERSION].uint32_type      = FLASH_PARAM_VERSION;
    flash_union_buffer[FLASH_IDX_PARAM_COUNT].uint32_type  = FLASH_PARAM_COUNT;

    // 写入Flash
    flash_write_page_from_buffer(100 + i/4, i%4);
}
```

#### 3. 在 flash.c 的 flash_load_config() 中添加加载代码

在函数末尾添加：

```c
void flash_load_config(int16 i)
{
    // ... 现有的版本检查和参数加载代码 ...

    // ====== 新增参数 (索引 37-38) ======
    new_param1 = flash_union_buffer[FLASH_IDX_NEW_PARAM1].float_type;
    new_param2 = flash_union_buffer[FLASH_IDX_NEW_PARAM2].float_type;
}
```

#### 4. 声明外部变量（如果需要）

在 flash.c 顶部添加：

```c
extern float new_param1;
extern float new_param2;
```

### 完整示例

假设要添加一个 `turn_coefficient` 转向系数参数：

**修改 flash.h：**
```c
// 在索引定义区域末尾添加
#define FLASH_IDX_TURN_COEFF    37

// 更新总数
#define FLASH_PARAM_COUNT       38      // 从 37 改为 38
```

**修改 flash.c（顶部）：**
```c
extern float turn_coefficient;
```

**修改 flash_save_config()：**
```c
// 在元数据之前添加
flash_union_buffer[FLASH_IDX_TURN_COEFF].float_type = turn_coefficient;
```

**修改 flash_load_config()：**
```c
// 在函数末尾添加
turn_coefficient = flash_union_buffer[FLASH_IDX_TURN_COEFF].float_type;
```

**首次启动时：**
- 旧的 Flash 只有 35 个参数（saved_param_count = 35）
- 系统检测到 `35 < 38`
- 自动将索引 35-37 清零
- `turn_coefficient` 被安全地初始化为 0.0

## 数据类型说明

`flash_union_buffer` 支持多种数据类型：

- **float_type**：浮点数（PID 参数等）
- **int16_type**：16位有符号整数（前瞻参数、BLDC参数等）
- **uint32_type**：32位无符号整数（版本号、计数等）

根据参数类型选择合适的成员访问。

## 配置槽位说明

系统支持 5 个独立的配置槽位：

| 槽位 | 函数调用 | Flash地址 | 用途 |
|-----|---------|----------|------|
| 0 | `flash_save/load_config_default()` | (100, 0) | 默认配置 |
| 1 | `flash_save/load_config_1()` | (100, 1) | 用户配置1 |
| 2 | `flash_save/load_config_2()` | (100, 2) | 用户配置2 |
| 3 | `flash_save/load_config_3()` | (100, 3) | 用户配置3 |
| 4 | `flash_save/load_config_4()` | (101, 0) | 用户配置4 |

在 `main.c` 的 `all_init()` 中调用 `flash_load_config_default()` 加载默认配置。

## 常见问题

### Q1: 添加新参数后需要重新烧录所有配置吗？

**不需要！** 系统会自动检测并初始化新参数为 0。但建议：
1. 首次启动后，通过菜单调整新参数的值
2. 保存到 Flash（调用 `flash_save_config_default()` 或通过菜单保存）
3. 下次启动就会加载正确的值

### Q2: 如果我想修改现有参数的位置怎么办？

**不建议修改现有参数的索引！** 这会导致加载旧数据时出错。推荐做法：
1. 增加 `FLASH_PARAM_VERSION` 的值
2. 在 `flash_load_config()` 中添加版本检测逻辑
3. 根据版本号进行数据迁移

### Q3: 如何清除所有保存的配置？

调用 `flash_reset()` 函数会擦除所有 5 个配置槽位的 Flash 数据。

### Q4: 元数据索引为什么要放在最后？

元数据（版本号、参数计数）放在最后（索引 35-36）是为了：
1. 方便扩展：添加新参数只需增加索引，不影响元数据位置
2. 兼容性：老版本代码如果没有保存元数据，加载时会读取到 0，系统可以识别

## 迁移注意事项

### 从旧版本迁移

如果你的系统之前使用了旧的 flash.c（有索引冲突的版本）：

1. **备份重要参数**：记录当前运行良好的 PID 参数值
2. **更新代码**：使用新的 flash.c 和 flash.h
3. **首次启动**：
   - 注释掉 `main.c` 中的 `flash_load_config_default()` 调用
   - 通过菜单手动输入之前备份的参数
   - 调用 `flash_save_config_default()` 保存
   - 恢复 `flash_load_config_default()` 调用
4. **后续启动**：正常加载，新参数会自动初始化为 0

或者更简单的方法：
1. 调用 `flash_reset()` 清除所有旧数据
2. 系统会使用代码中的默认参数值
3. 通过菜单调整参数后保存

## 代码文件说明

### 修改的文件

1. **CarCode/code/flash.h**
   - 添加了版本管理宏定义
   - 添加了所有参数的索引宏定义
   - 文件位置：`C:\Users\76343\newstart\CarCode\code\flash.h`

2. **CarCode/code/flash.c**
   - 重新实现了 `flash_save_config()` 函数
   - 重新实现了 `flash_load_config()` 函数，添加自动初始化逻辑
   - 清理了代码注释
   - 文件位置：`C:\Users\76343\newstart\CarCode\code\flash.c`

### 未修改的文件

- 所有其他代码文件保持不变
- 函数接口保持兼容，无需修改调用代码

## 总结

本次升级的核心优势：

1. **更安全**：消除了索引冲突，避免数据覆盖
2. **更灵活**：添加新参数时自动初始化，不会卡死
3. **更清晰**：使用宏定义索引，代码可读性强
4. **更易维护**：参数按模块分组，便于管理

建议在使用新系统前先用 `flash_reset()` 清除旧数据，然后重新调整和保存参数。

---

**文档版本：** 1.0
**更新日期：** 2025-10-22
**适用代码：** CarCode 项目（MM32F3273G8P）
