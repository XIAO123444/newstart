# Screen.c 差分线显示优化方案

## 优化目标
将 `display_delta_line()` 函数从"批量清除区域"改为"记录并清除上一帧绘制的点"，以提升显示刷新效率。

## 性能对比分析

### 当前方案
- **清除操作**: 2 × (60 × 43) = 5,160 像素（批量清除整个显示区域）
- **绘制操作**: 60 × 2 × 9 = 1,080 像素（每个点用3x3矩阵）
- **总计**: 约 6,240 像素操作

### 优化方案
- **清除操作**: 60 × 2 × 9 = 1,080 像素（只清除上一帧实际绘制的点）
- **绘制操作**: 60 × 2 × 9 = 1,080 像素
- **总计**: 约 2,160 像素操作
- **性能提升**: 约 **65%** （减少 4,080 像素操作）

### 内存开销
- 新增静态数组: `60 × 2 × 2 = 240 字节`（记录上一帧的Y坐标）
- 新增标志位: `1 字节`
- **总计**: 241 字节（完全可接受）

---

## 代码修改详情

### 文件: `CarCode/code/screen.c`

#### 1. 在文件顶部添加静态变量（第51行之后）

```c
/*
 * @brief     差分线显示优化：记录上一帧绘制的点坐标
 */
static int16 last_left_y[MT9V03X_H];   // 上一帧左线各行的Y坐标
static int16 last_right_y[MT9V03X_H];  // 上一帧右线各行的Y坐标
static bool first_frame = true;        // 首帧标志（首次绘制不清除）
```

#### 2. 替换 `display_delta_line()` 函数实现（第74-127行）

**原始代码**:
```c
void display_delta_line(int16 startX1,int16 startY1,int16 startX2,int16 startY2)
{
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    ips200_show_string(0,0, "delta_leftline");
    // 边界检查
    if (startY1+20>=320||startY1-20<0||startX1+MT9V03X_H>=240
         ||startX1<0||startY2+20>=320||startY2-20<0||
         startX2+MT9V03X_H>=240||startX2<0)
    {
        ips200_show_string(0,20, "out of range");
        return;
    }

    // 性能优化：使用 ips200_clear_region 批量清除区域（性能提升约85%）
    // 清除左线显示区域（43行高度）
    ips200_clear_region((uint16)startX1, (uint16)(startY1-21),
                        (uint16)(startX1+MT9V03X_H-1), (uint16)(startY1+21));

    // 清除右线显示区域（43行高度）
    ips200_clear_region((uint16)startX2, (uint16)(startY2-21),
                        (uint16)(startX2+MT9V03X_H-1), (uint16)(startY2+21));

    // 绘制差分线数据
    int16 search_start = search_stop + 1;
    for(int16 i=0; i<MT9V03X_H; i++)
    {
        if(i >= search_start)
        {
            // 处理左线差分值（限幅到±10）
            int16 temp_leftline = delta_leftline[i];
            if(temp_leftline > 20)
                temp_leftline = 20;
            else if(temp_leftline < -20)
                temp_leftline = -20;

            // 处理右线差分值（限幅到±10）
            int16 temp_rightline = delta_rightline[i];
            if(temp_rightline > 20)
                temp_rightline = 20;
            else if(temp_rightline < -20)
                temp_rightline = -20;

            // 绘制差分点（3x3）
            ips200_draw_point33((uint16)(startX1+i), (uint16)(temp_leftline+startY1), RGB565_GREEN);
            ips200_draw_point33((uint16)(startX2+i), (uint16)(temp_rightline+startY2), RGB565_BROWN);
        }
        else
        {
            // 绘制基准线（搜索停止区域）
            ips200_draw_point33((uint16)(startX1+i), (uint16)startY1, RGB565_WHITE);
            ips200_draw_point33((uint16)(startX2+i), (uint16)startY2, RGB565_WHITE);
        }
    }
}
```

**优化后代码**:
```c
void display_delta_line(int16 startX1,int16 startY1,int16 startX2,int16 startY2)
{
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    ips200_show_string(0,0, "delta_leftline");

    // 边界检查
    if (startY1+20>=320||startY1-20<0||startX1+MT9V03X_H>=240
         ||startX1<0||startY2+20>=320||startY2-20<0||
         startX2+MT9V03X_H>=240||startX2<0)
    {
        ips200_show_string(0,20, "out of range");
        return;
    }

    // 性能优化：只清除上一帧实际绘制的点（性能提升约65%）
    if (!first_frame)
    {
        // 清除上一帧的左线和右线点（3x3矩阵，用背景色覆盖）
        for(int16 i=0; i<MT9V03X_H; i++)
        {
            ips200_draw_point33((uint16)(startX1+i), (uint16)last_left_y[i], RGB565_BLACK);
            ips200_draw_point33((uint16)(startX2+i), (uint16)last_right_y[i], RGB565_BLACK);
        }
    }
    else
    {
        // 首帧需要完整清除区域（防止残留）
        ips200_clear_region((uint16)startX1, (uint16)(startY1-21),
                            (uint16)(startX1+MT9V03X_H-1), (uint16)(startY1+21));
        ips200_clear_region((uint16)startX2, (uint16)(startY2-21),
                            (uint16)(startX2+MT9V03X_H-1), (uint16)(startY2+21));
        first_frame = false;
    }

    // 绘制差分线数据并记录当前帧坐标
    int16 search_start = search_stop + 1;
    for(int16 i=0; i<MT9V03X_H; i++)
    {
        int16 current_left_y, current_right_y;

        if(i >= search_start)
        {
            // 处理左线差分值（限幅到±20）
            int16 temp_leftline = delta_leftline[i];
            if(temp_leftline > 20)
                temp_leftline = 20;
            else if(temp_leftline < -20)
                temp_leftline = -20;

            // 处理右线差分值（限幅到±20）
            int16 temp_rightline = delta_rightline[i];
            if(temp_rightline > 20)
                temp_rightline = 20;
            else if(temp_rightline < -20)
                temp_rightline = -20;

            // 计算当前帧Y坐标
            current_left_y = temp_leftline + startY1;
            current_right_y = temp_rightline + startY2;

            // 绘制差分点（3x3）
            ips200_draw_point33((uint16)(startX1+i), (uint16)current_left_y, RGB565_GREEN);
            ips200_draw_point33((uint16)(startX2+i), (uint16)current_right_y, RGB565_BROWN);
        }
        else
        {
            // 绘制基准线（搜索停止区域）
            current_left_y = startY1;
            current_right_y = startY2;

            ips200_draw_point33((uint16)(startX1+i), (uint16)current_left_y, RGB565_WHITE);
            ips200_draw_point33((uint16)(startX2+i), (uint16)current_right_y, RGB565_WHITE);
        }

        // 记录当前帧坐标供下一帧清除使用
        last_left_y[i] = current_left_y;
        last_right_y[i] = current_right_y;
    }
}
```

---

## 修改说明

### 新增内容
1. **静态缓冲数组** (`last_left_y[]`, `last_right_y[]`):
   - 记录上一帧每行绘制的Y坐标
   - 数组大小: `MT9V03X_H` (60行)
   - 数据类型: `int16`（与原坐标类型一致）

2. **首帧标志** (`first_frame`):
   - 初始值: `true`
   - 首次绘制时使用批量清除（防止屏幕残留）
   - 之后切换为点清除模式

### 修改逻辑
1. **清除策略**:
   - **首帧**: 使用 `ips200_clear_region()` 批量清除（保持原逻辑）
   - **后续帧**: 遍历上一帧记录的坐标，用 `RGB565_BLACK` 覆盖清除

2. **绘制流程**:
   - 计算当前帧坐标 (`current_left_y`, `current_right_y`)
   - 绘制新的3x3点
   - **同时记录坐标到缓冲数组**（供下一帧清除）

3. **边界情况处理**:
   - `search_start` 之前的区域绘制白色基准线，坐标同样记录
   - 保持原有的差分值限幅逻辑（±20）

---

## 注意事项

### 1. 内存安全
- 静态数组大小固定为 `MT9V03X_H`（60），与循环边界一致
- 无动态内存分配，无内存泄漏风险

### 2. 首帧处理
- 首次调用时完整清除区域，避免屏幕上的随机残留
- `first_frame` 标志确保后续帧切换到高效模式

### 3. 颜色选择
- 清除使用 `RGB565_BLACK`（背景色）
- 如果背景色不是黑色，需要修改为实际背景色

### 4. 3x3 覆盖保证
- 由于每个点用3x3矩阵绘制，即使Y坐标偏移1-2像素，新点也能完全覆盖旧点边缘
- 这避免了差分值剧烈变化时的残影问题

### 5. 模式切换
- 如果需要重新初始化显示（如切换菜单模式），可以将 `first_frame` 重置为 `true`
- 建议在菜单模式切换函数中添加: `extern void reset_delta_line_display(void);`

---

## 可选扩展功能

### 1. 添加重置函数（建议添加到 screen.h）

```c
/*
 * @brief     重置差分线显示状态（切换模式时调用）
 */
void reset_delta_line_display(void)
{
    first_frame = true;
}
```

### 2. 动态背景色适配（如果背景色可能改变）

在 `screen.c` 顶部添加:
```c
static rgb565_color_enum background_color = RGB565_BLACK;

void set_delta_line_background(rgb565_color_enum color)
{
    background_color = color;
}
```

然后修改清除代码中的 `RGB565_BLACK` 为 `background_color`。

---

## 测试建议

1. **首次运行**: 观察是否有残留（验证首帧清除逻辑）
2. **连续运行**: 观察差分线刷新是否流畅（验证点清除逻辑）
3. **剧烈变化**: 快速转动车辆，观察是否有拖影（验证3x3覆盖）
4. **边界测试**: 差分值达到±20限幅时，观察显示是否正常

---

## 总结

此优化方案通过"记录上一帧坐标 + 精确清除"的策略，将像素操作量从 6,240 降至 2,160，**理论性能提升约 65%**，同时仅增加 241 字节内存开销。非常适合实时嵌入式系统中的高帧率显示需求。
