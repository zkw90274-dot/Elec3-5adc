# IMU 四元数姿态解算 - STC32G 移植指南

## 目录

- [一、文件准备](#一文件准备)
- [二、Keil C251 编译器配置](#二keil-c251-编译器配置)
- [三、代码兼容性修改](#三代码兼容性修改)
- [四、数学库配置](#四数学库配置)
- [五、性能优化](#五性能优化)
- [六、最小移植示例](#六最小移植示例)
- [七、常见编译错误](#七常见编译错误)
- [八、测试验证](#八测试验证)
- [九、STC32G 特定优化](#九stc32g-特定优化)

---

## 一、文件准备

### 1.1 复制核心文件

将以下文件从原工程复制到目标工程：

```
目标工程/
├── quaternion.h    ← 核心头文件
└── quaternion.c    ← 核心实现
```

### 1.2 添加到 Keil 工程

1. 在 Keil 中打开工程
2. 在 **Project Workspace** 中右键点击目标工程组
3. 选择 **Add Existing Files to Group**
4. 选择 `quaternion.h` 和 `quaternion.c`
5. 确保文件已添加到工程树中

---

## 二、Keil C251 编译器配置

### 2.1 目标设置

打开 **Options for Target**（快捷键 Alt+F7）：

| 选项卡 | 设置项 | 推荐值 | 说明 |
|--------|--------|--------|------|
| **Target** | Memory Model | SMALL | 节省 RAM |
| **Target** | Use Extended Linker (LX51) | ✓ 勾选 | 支持更大程序 |
| **Target** | Use Extended Linker (LX51) | ✓ 勾选 | 支持更大程序 |
| **C251** | Optimization Level | 8 或 9 | 8=速度优先，9=代码大小优先 |
| **C251** | Warnings | All Warnings | 检测潜在问题 |

### 2.2 关键：浮点支持

```
C251 选项卡 → Floating Point Arithmetic → 选择 "Non-Retargetable"
```

**重要说明**：
- STC32G **没有硬件 FPU**
- 所有浮点运算通过软件模拟
- 运算速度比 ARM 慢约 10-50 倍

### 2.3 链接器配置

```
Link 选项卡 → Misc
确保包含：C251FPL.LIB（浮点运算库）
```

---

## 三、代码兼容性修改

### 3.1 Fast_InvSqrt 函数（必须修改）

**原因**：STC32G 软件浮点不支持快速平方根倒数的位操作技巧

**修改文件**：`quaternion.c` 第 9-17 行

**原代码**：
```c
float Fast_InvSqrt(float x)
{
    float halfx = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - halfx * x * x);
    return x;
}
```

**修改为**：
```c
float Fast_InvSqrt(float x)
{
    // STC32G 软件浮点，使用标准库函数
    return 1.0f / sqrtf(x);
}
```

### 3.2 数据类型检查

STC32G 的数据类型：
- `char`：8 位
- `int`：16 位
- `long`：32 位
- `float`：32 位 IEEE 754

**检查代码**：`quaternion.c` 第 26-44 行的轴重映射函数

```c
// 代码中使用的位操作需要确认
int map_x = AXIS_MAP_X;  // 如果 AXIS_MAP_X > 32767 需改为 long
```

**建议**：保持原代码，如果编译报错再调整。

### 3.3 包含头文件

确保 `quaternion.c` 顶部正确包含：

```c
#include "quaternion.h"
#include <math.h>       // STC32G 数学库
#include <stdint.h>     // 标准整数类型
```

---

## 四、数学库配置

### 4.1 必需的数学函数

代码中使用以下函数，需要确保数学库正确链接：

| 函数 | 用途 |
|------|------|
| `sqrtf()` | 平方根 |
| `atan2f()` | 反正切（四元数转欧拉角） |
| `asinf()` | 反正弦（四元数转欧拉角） |

### 4.2 验证数学库

在编译时如果出现以下错误：
```
ERROR: UNRESOLVED EXTERNAL SYMBOL - sqrtf
```

**解决方法**：
1. 检查是否已添加 `#include <math.h>`
2. 在链接器设置中添加 `C251FPL.LIB`

---

## 五、性能优化

### 5.1 降低计算频率

由于 STC32G 浮点运算较慢，建议降低采样频率：

**修改文件**：`quaternion.h`

```c
// 原配置
#define SAMPLE_FREQ 200.0f

// 修改为
#define SAMPLE_FREQ 100.0f   // 或 50.0f，根据实际性能调整
```

### 5.2 简化算法参数

```c
// 可选：略微放宽参数以减少计算量
#define MAHONY_KP  0.5f      // 从 0.3f 提高到 0.5f
#define MAHONY_KI  0.001f    // 从 0.002f 降低到 0.001f
```

### 5.3 降低输出频率

**修改文件**：`main.c`

```c
// 原代码：每 10 次循环输出一次（20Hz）
if (++loop_count >= 10)

// 修改为：每 20 次循环输出一次（10Hz）
if (++loop_count >= 20)
```

---

## 六、最小移植示例

### 6.1 传感器数据接口要求

```c
// 您需要提供以下传感器数据：
float ax, ay, az;  // 加速度计，单位：g (9.8 m/s²)
float gx, gy, gz;  // 陀螺仪，单位：°/s（度每秒）
float dt;          // 采样时间间隔，单位：秒
```

### 6.2 完整示例代码

```c
#include "STC32G.h"         // STC32G 头文件
#include "quaternion.h"

// ========== 传感器接口示例（替换为您的实际驱动）==========
float Get_Acc_X(void) { /* 返回 X 轴加速度 (g) */ }
float Get_Acc_Y(void) { /* 返回 Y 轴加速度 (g) */ }
float Get_Acc_Z(void) { /* 返回 Z 轴加速度 (g) */ }

float Get_Gyro_X(void) { /* 返回 X 轴角速度 (°/s) */ }
float Get_Gyro_Y(void) { /* 返回 Y 轴角速度 (°/s) */ }
float Get_Gyro_Z(void) { /* 返回 Z 轴角速度 (°/s) */ }
//=========================================================

MahonyAHRS_t ahrs;
Quaternion_t quat;

/**
 * @brief IMU 初始化
 */
void IMU_Init(void)
{
    Mahony_Init(&ahrs);
}

/**
 * @brief IMU 数据更新（在主循环或定时器中断中调用）
 */
void IMU_Update(void)
{
    // 1. 获取传感器数据
    float ax = Get_Acc_X();
    float ay = Get_Acc_Y();
    float az = Get_Acc_Z();
    float gx = Get_Gyro_X();
    float gy = Get_Gyro_Y();
    float gz = Get_Gyro_Z();

    // 2. 加速度计归一化（重要！）
    float norm = Fast_InvSqrt(ax*ax + ay*ay + az*az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 3. 陀螺仪转换为弧度
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    // 4. 更新姿态
    float dt = 0.01f;  // 100Hz 采样，根据实际调整
    Mahony_Update(&ahrs, gx, gy, gz, ax, ay, az, dt);

    // 5. 获取四元数结果
    Mahony_GetQuaternion(&ahrs, &quat);

    // 6. 使用 quat.q0, quat.q1, quat.q2, quat.q3
}

/**
 * @brief 主函数示例
 */
void main(void)
{
    // STC32G 硬件初始化
    // ... (时钟、串口、定时器等初始化代码)

    IMU_Init();

    while(1)
    {
        IMU_Update();

        // 处理姿态数据
        // printf("q0=%.4f, q1=%.4f, q2=%.4f, q3=%.4f\n",
        //        quat.q0, quat.q1, quat.q2, quat.q3);

        Delay_ms(10);  // 控制更新频率为 100Hz
    }
}
```

### 6.3 中断方式更新（推荐）

为保证 `dt` 参数准确，建议在定时器中断中更新：

```c
// 假设定时器 0，1ms 中断一次
void Timer0_ISR(void) interrupt 1
{
    static uint16_t count = 0;
    static float dt = 0.01f;  // 固定 10ms 更新一次

    count++;

    if (count >= 10)  // 每 10ms 更新一次姿态（100Hz）
    {
        count = 0;
        IMU_Update();
    }
}
```

---

## 七、常见编译错误

| 错误信息 | 原因 | 解决方法 |
|----------|------|----------|
| `UNRESOLVED EXTERNAL SYMBOL - sqrtf` | 未链接数学库 | 添加 `C251FPL.LIB` 到链接器 |
| `UNRESOLVED EXTERNAL SYMBOL - atan2f` | 未链接数学库 | 同上 |
| `error C202: 'sqrtf': undefined` | 缺少 math.h | 添加 `#include <math.h>` |
| `warning C306: conversion loss` | 数据类型转换 | 可忽略，或调整类型 |
| `OUT OF MEMORY` | RAM 或 Flash 不足 | 降低 SAMPLE_FREQ 或优化代码 |
| `STACK OVERFLOW` | 栈空间不足 | 增加栈大小或减少局部变量 |

### 解决 OUT OF MEMORY

如果遇到内存不足：

1. **降低采样频率**
   ```c
   #define SAMPLE_FREQ 50.0f  // 从 100Hz 降到 50Hz
   ```

2. **禁用不需要的功能**
   ```c
   // 如果不需要欧拉角输出，可注释掉 Quaternion_ToEuler 函数
   ```

3. **优化编译选项**
   ```
   C251 → Optimization Level → 9 (代码大小优先)
   ```

---

## 八、测试验证

### 8.1 编译测试

```
步骤：
1. Project → Rebuild All
2. 检查输出窗口：
   - 0 Error(s)
   - 少量 Warning 可接受
```

### 8.2 功能测试

#### 静止测试

传感器平放静止时，输出应接近：

```
quat.q0 ≈ 1.0000  (标量部分)
quat.q1 ≈ 0.0000  (X 分量)
quat.q2 ≈ 0.0000  (Y 分量)
quat.q3 ≈ 0.0000  (Z 分量)
```

**验证公式**：`q0² + q1² + q2² + q3² ≈ 1.0`

#### 运动测试

1. 绕 X 轴旋转 → q1 应有明显变化
2. 绕 Y 轴旋转 → q2 应有明显变化
3. 绕 Z 轴旋转 → q3 应有明显变化

### 8.3 性能测试

测量单次 `Mahony_Update()` 的执行时间：

```c
// 假设定时器频率为 1MHz
uint32_t start = TIMER0_COUNTER;  // 读取定时器计数值

IMU_Update();  // 或 Mahony_Update(...)

uint32_t end = TIMER0_COUNTER;
uint32_t time_us = end - start;  // 微秒

// 判断：
// time_us < 5000μs (5ms) → 性能良好
// time_us > 10000μs (10ms) → 需要优化或降低频率
```

---

## 九、STC32G 特定优化

### 9.1 定点数优化（高级）

如果浮点运算仍然太慢，可以考虑定点数版本。这需要大量重写代码，不推荐新手尝试。

### 9.2 中断优先级

建议将 IMU 更新中断设置为**较高优先级**，保证 `dt` 准确性：

```c
// STC32G 中断优先级配置
IP = 0x10;  // 设置定时器中断为较高优先级
```

### 9.3 DMA 优化（如果支持）

部分 STC32G 型号支持 DMA，可用于传感器数据采集，减少 CPU 占用。

### 9.4 电源管理

```c
// 如果使用电池供电，注意浮点运算功耗较高
// 建议在不使用时降低采样频率或进入低功耗模式
```

---

## 十、移植检查清单

移植前：
- [ ] 确认 STC32G 型号和内存大小
- [ ] 安装 Keil C251 编译器
- [ ] 准备传感器驱动代码

编译配置：
- [ ] 复制 quaternion.h/c 到工程
- [ ] 添加到工程文件列表
- [ ] 设置 Memory Model 为 SMALL
- [ ] 启用浮点运算（Non-Retargetable）
- [ ] 添加数学库 C251FPL.LIB
- [ ] 设置优化级别为 8 或 9

代码修改：
- [ ] 修改 Fast_InvSqrt 为 sqrtf 版本
- [ ] 检查数据类型兼容性
- [ ] 降低 SAMPLE_FREQ 到 100Hz 或更低
- [ ] 实现传感器数据接口

测试验证：
- [ ] 编译通过（0 Error）
- [ ] 静止时输出接近 (1,0,0,0)
- [ ] 单次更新时间 < 5ms
- [ ] 运动时四元数平滑变化

---

## 附录

### A. 支持的 STC32G 型号

- STC32G12K128
- STC32G12K64
- STC32G12K32
- 其他 STC32G 系列（需确认内存充足）

### B. 参考资源

- STC 官方网站：[www.stcmcudata.com](http://www.stcmcudata.com)
- STC32G 技术手册
- Keil C251 用户手册

### C. 技术支持

如有问题，请检查：
1. 编译器版本是否最新
2. 数学库是否正确链接
3. STC32G 数据手册中的浮点运算说明

---

**文档版本**：v1.0
**最后更新**：2025-01-25
**适用版本**：quaternion v1.0
