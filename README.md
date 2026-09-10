# AI8051U 无刷电机电调（BLDC ESC）

基于 **STC AI8051U** 单片机的无刷直流电机电调固件，使用反电动势（BEMF）过零检测实现无感六步换相，支持正弦启动、RC 油门信号输入、电池低压保护和堵转保护。

代码从 STC32G 电调移植而来（见 [bldc/bldc_version.txt](bldc/bldc_version.txt)），底层驱动使用逐飞科技（SEEKFREE）的 AI8051U 开源库。

> **本工程源码统一使用 GBK/ANSI 编码**（详见文末「编码约定」）。用编辑器打开如果中文乱码，是编码没设对，不是代码问题。

---

## 目录

- [1. 一分钟理解这个项目在干什么](#1-一分钟理解这个项目在干什么)
- [2. 硬件与引脚分配](#2-硬件与引脚分配)
- [3. 工程目录结构](#3-工程目录结构)
- [4. 编译与烧录](#4-编译与烧录)
- [5. 核心架构：电机状态机](#5-核心架构电机状态机)
- [6. 无感换相是怎么实现的](#6-无感换相是怎么实现的)
- [7. 定时器与中断资源分配](#7-定时器与中断资源分配)
- [8. 各模块详解](#8-各模块详解)
- [9. 关键数据结构](#9-关键数据结构)
- [10. 调参指南](#10-调参指南)
- [11. 新人上手步骤](#11-新人上手步骤)
- [12. 常见坑与注意事项](#12-常见坑与注意事项)
- [13. 编码约定](#13-编码约定)

---

## 1. 一分钟理解这个项目在干什么

这是一个**电调（ESC）**——就是航模/车模里夹在电池和电机之间那块小板子。它做三件事：

1. **读油门**：从 P21 引脚捕获外部 RC 信号（1~2ms 高电平的 PWM），换算成目标占空比 `motor.duty`。
2. **驱动电机**：用 PWMA 输出三路互补 PWM 到三相全桥（P00~P05），按六步换相表轮流导通。
3. **自己判断什么时候该换相**：无感 BLDC 没有霍尔传感器，靠比较器（P44/P46/P50/P51）检测**未导通相的反电动势过零点**，推算出转子位置，延时一个「进角」后换相。

启动时电机静止、反电动势为 0，无法检测，所以先用**正弦强拉**（`sine_control.c`）把电机拖起来，等转速够了再切到闭环。

**主控制循环**在 `TM1_Isr`（50µs 一次）里跑状态机，这是整个工程的心脏。

---

## 2. 硬件与引脚分配

### 三相全桥 PWM

| 引脚 | 功能 | 宏定义 |
|---|---|---|
| P00 | A 相上桥 | `PWM_A_H_PIN` |
| P01 | A 相下桥 | `PWM_A_L_PIN` |
| P02 | B 相上桥 | `PWM_B_H_PIN` |
| P03 | B 相下桥 | `PWM_B_L_PIN` |
| P04 | C 相上桥 | `PWM_C_H_PIN` |
| P05 | C 相下桥 | `PWM_C_L_PIN` |

宏定义在 [bldc/pwm_out.h](bldc/pwm_out.h)。

### 比较器（反电动势过零检测）

| 引脚 | 功能 |
|---|---|
| P44 | 虚拟中性点输入（`COMPARATOR_MID_PIN`） |
| P46 | A 相电压输入（`COMPARATOR_A_PIN`） |
| P50 | B 相电压输入（`COMPARATOR_B_PIN`） |
| P51 | C 相电压输入（`COMPARATOR_C_PIN`） |
| P41 | 比较器输出（`CMPO_S = 1` 时选择 P41） |

定义在 [bldc/comparator.c](bldc/comparator.c) 顶部。

### 其他外设

| 引脚 | 功能 | 定义位置 |
|---|---|---|
| P21 | 油门 PWM 输入捕获（PWMB） | `PWMIN_PIN` @ [bldc/signal_input.c](bldc/signal_input.c) |
| P23 | 同批初始化的输入引脚（PWMB 引脚切换 `PWMB_PS = 0x0A` 相关） | [bldc/signal_input.c](bldc/signal_input.c) |
| P15 | 电池电压 ADC 采样 | [bldc/battery.c](bldc/battery.c) |
| P36 | RUN 指示灯 | `RUN_LED` @ [bldc/pit_timer.c](bldc/pit_timer.c) |
| P37 | ERR 指示灯 | `ERR_LED` @ [bldc/pit_timer.c](bldc/pit_timer.c) |

### LED 指示含义

在 [main.c](project/user/main.c) 注释里也有说明，`led_control()` 实现：

| 现象 | 含义 |
|---|---|
| RUN 慢闪（400ms 周期），ERR 熄灭 | 空闲待机（`MOTOR_IDLE`） |
| RUN 常亮，ERR 熄灭 | 电机运行中（`MOTOR_START`/`OPEN_LOOP`/`CLOSE_LOOP`） |
| RUN 熄灭，ERR **常亮** | **电池电压过低**（`BYTE_LOW_VOLTAGE`） |
| RUN 熄灭，ERR 快闪（100ms 周期） | **堵转停机**（`MOTOR_RESTART`） |

> LED 为**低电平点亮**（写 0 = 亮）。
>
> ⚠️ 低压保护这一行注意：`main.c` 注释和 `led_control()` 里的注释都写的是「ERR 慢闪」，但**代码实际是 `ERR_LED = 0` 常亮，并不闪**（见 [bldc/pit_timer.c](bldc/pit_timer.c) `led_control()`）。以代码为准。

### 电池分压

`battery_voltage = pin_voltage × 120/20 = pin_voltage × 6`，ADC 参考 3.3V，所以量程约 19.8V。分压电阻若与实物不符，改 [bldc/battery.c](bldc/battery.c) 里的 `* 120/20`。

---

## 3. 工程目录结构

```
AI8051/
├── bldc/                      ← 本项目的核心业务代码（移植/自研部分）
│   ├── bldc_config.h          所有可调参数的集中配置
│   ├── motor_control.c/.h     换相中断、比较器中断、状态标志、鸣叫
│   ├── comparator.c/.h        比较器（过零检测）配置
│   ├── pwm_out.c/.h           六步换相表 + PWMA 初始化
│   ├── sine_control.c/.h      正弦启动（开环强拉）
│   ├── signal_input.c/.h      PWMB 输入捕获，解析 RC 油门
│   ├── battery.c/.h           ADC + DMA 电池电压采样
│   ├── pit_timer.c/.h         50µs 周期中断 + 状态机 + LED（最重要）
│   └── bldc_version.txt       本项目版本记录
│
├── project/
│   ├── user/                  用户代码
│   │   ├── main.c             入口，初始化顺序在这里
│   │   └── isr.c/.h           串口 DMA 中断（含串口自动下载）
│   ├── mdk/                   Keil 工程
│   │   ├── seekfree.uvproj    工程文件（可提交）
│   │   ├── seekfree.uvopt     工程选项（可提交）
│   │   └── out_file/          编译产物（已 gitignore）
│   └── code/                  预留的空白分组（Keil 里是空组）
│
├── libraries/                 逐飞科技官方库（第三方，尽量别改）
│   ├── zf_common/             类型定义、时钟、调试、字体、中断
│   ├── zf_driver/             GPIO/PWM/UART/Timer/PIT/Delay/ADC 驱动
│   ├── zf_device/             各外设模块（本项目基本没用到）
│   └── zf_components/         上位机通信组件
│
├── version/version.txt        版本文件导航说明
└── README.md                  本文档
```

**判断该改哪里的原则**：`libraries/` 是逐飞官方库，改动会破坏后续升级能力，业务逻辑一律写在 `bldc/`。Keil 工程里 `zf_common` / `zf_driver` / `user` / `bldc` 四个分组对应上述目录。

---

## 4. 编译与烧录

### 工具链

| 项 | 值 |
|---|---|
| IDE | Keil µVision V5.39（实测） |
| 工具链 | **PK251（C251 编译器）V5.60**，路径 `D:\Keil5\C251\BIN` |
| 目标器件 | STC8051U-32Bit Series（Vendor: STC） |
| Keil 目标名 | `AI8051U_32bit` |
| 内存模型 | `MemoryModel = 3`，优化 `Optim = 7`（9 级最优） |
| 工程文件 | [project/mdk/seekfree.uvproj](project/mdk/seekfree.uvproj) |

> ⚠️ **这是 251 架构，不是 ARM。** 装的必须是 PK251（C251）工具链，不是 MDK-ARM。没有 C251 授权的话工程打不开、编译不了。

### 头文件包含路径

```
..\..\libraries\zf_common
..\..\libraries\zf_components
..\..\libraries\zf_device
..\..\libraries\zf_driver
..\user          ← 注意：相对于 project/mdk 的 ..\user，即 project/user
..\code
..\..\bldc
```

### 编译输出

`project/mdk/out_file/SEEKFREE.hex`。当前占用：

```
data=8.0  edata+hdata=1626  xdata=74  const=1222  code=17121
```

### 烧录

支持**串口自动下载**：[project/user/isr.c](project/user/isr.c) 的 `DMA_UART1_IRQHandler` 里监听串口收到的 `0x7F`，连续收到 20 次以上就置 `IAP_CONTR = 0x60` 触发复位进 ISP。所以用 STC-ISP 工具直接点下载、再给板子复位即可，不用手动按键。

同目录下的 `MDK…bat`（文件名编码已损坏）是清理脚本，内容是：

```bat
rd Out_File /s/q
DEL SEEKFREE.uvg*.*
DEL SEEKFREE.uvopt
exit
```

---

## 5. 核心架构：电机状态机

状态机定义在 [bldc/motor_control.h](bldc/motor_control.h) 的 `run_state_enum`，**在 `TM1_Isr`（50µs 周期）里推进**，实现在 [bldc/pit_timer.c](bldc/pit_timer.c) 的 `pit_motor_control()`。

```
                    motor.duty != 0
   ┌─────────┐  ────────────────────────►  ┌──────────────┐
   │MOTOR_IDLE│                            │ MOTOR_START  │
   └─────────┘  ◄────────────────────────  └──────────────┘
        ▲           重启延时结束                  │
        │                                        │ 正弦强拉 / 开环延时
        │                                        ▼
        │                                 ┌──────────────┐
        │                                 │MOTOR_OPEN_LOOP│
        │                                 └──────────────┘
        │                                        │ 换相次数 > BLDC_OPEN_LOOP_WAIT
        │                                        ▼
        │                                 ┌──────────────┐
        │                                 │MOTOR_CLOSE_LOOP│◄──┐
        │                                 └──────────────┘   │ 正常运行
        │                                        │            │
        │              堵转 / 换相失败过多       │            │
        │                                 ┌──────────────┐   │
        │                                 │MOTOR_STOP_STALL│──┘
        │                                 └──────────────┘
        │                                        │
        │                                 ┌──────────────┐
        └─────────────────────────────────│ MOTOR_RESTART│
                MOTOR_RESTART 延时结束     └──────────────┘

   任意状态下，只要检测到电池电压过低 ──► BYTE_LOW_VOLTAGE（最高优先级，锁死）
```

### 各状态做什么

| 状态 | 行为 |
|---|---|
| `MOTOR_IDLE` | 待机。关定时器0中断、关定时器4中断、关比较器中断、电机停转。LED 慢闪 |
| `MOTOR_START` | 关所有中断 → 打开全部 PWM 通道 → 正弦强拉（`sine_init` + `sine_start` 循环到 `motor_a_position==0`，即电角度转满一圈）→ 用 `6000` 初始化 6 次换相时间缓存 → 输出第一次换相 → 转 `MOTOR_OPEN_LOOP` |
| `MOTOR_OPEN_LOOP` | **无感开环**：不靠比较器，靠 `pit_timer` 里轮询比较器输出电平的跳变（`gpio_state` 移位滤波，连续 8 次同电平才认），换相后等 15° 再走下一步。连续换相 `BLDC_OPEN_LOOP_WAIT` 次后打开比较器中断、进入闭环 |
| `MOTOR_CLOSE_LOOP` | **无感闭环**：由比较器中断触发换相（见下节）。同时做堵转检测和占空比缓慢加减速 |
| `MOTOR_STOP_STALL` | 停机善后：`motor_stop()` + 关比较器/定时器4中断 → 转 `MOTOR_RESTART` |
| `MOTOR_RESTART` | 倒计时 `BLDC_START_DELAY`，归零后回 `MOTOR_IDLE`，于是电机可以再次尝试启动 |
| `BYTE_LOW_VOLTAGE` | 电池欠压，最高优先级，任何状态一旦命中就锁死在此状态不再启动 |

### 状态切换的触发者

- `TM1_Isr` 每 50µs 检查 `motor.duty`：为 0 → `MOTOR_IDLE`；非 0 且当前 `MOTOR_IDLE` → `MOTOR_START`。
- `TM1_Isr` 同时调用 `battery_voltage_get()`，欠压直接切 `BYTE_LOW_VOLTAGE`。
- `TM4_Isr`（换相超时）在闭环下超过换相次数后置 `MOTOR_STOP_STALL`。
- `TM0_Isr` 里换相失败计数超 `BLDC_COMMUTATION_FAILED_MAX` 时 `motor_stop()` + `MOTOR_STOP_STALL`。

---

## 6. 无感换相是怎么实现的

这是整个项目最核心也最难懂的部分，三个中断配合完成。

### 硬件基础

三相全桥任意时刻只导通两相（一相上桥、一相下桥），第三相**悬空**。悬空相的反电动势会随转子位置变化，穿过中点电压时就是**过零点**。过零点之后转子再转 30°（理想情况）就该换相了。

### 三个中断的角色

**① `comparator_isr`（interrupt 21）——过零捕获**

比较器检测到悬空相电压穿越中性点（`CMP_SELECT_A/B/C` 选的通道决定看哪一相），触发中断：

```c
uint16 tim_com = 0xffff - (motor.filter_commutation_time_sum * BLDC_MOTOR_ANGLE / 360);
temp_commutation_time = (T4H << 8) | T4L;   // 读出本步实际用时
xc_flag = 0;                                 // 标记：可以换相了
// 重启 T4 重新计时，把「进角延时」装进 T0
comparator_close_isr();                      // 关比较器中断，等换相后重开
```

**② `TM0_Isr`（interrupt 1）——进角延时后换相**

`xc_flag` 是个两段式握手标志：

- `xc_flag == 0`（过零已发生）→ 真正执行换相：`motor_next_step()` + `pwm_x_output[motor.step]()`，然后计算下一次换相时间、更新滑动窗口、做换相错误判断，最后把 `xc_flag = 1`，并重装 T0。
- `xc_flag == 1` → 这是换相后等待消磁的延时（`xc_flag` 置 2），到期后调用 `comparator_open_isr()` 重新打开比较器中断，并关掉 T0（`ET0 = 0; TR0 = 0;`）。之后回到 `xc_flag == 0` 等待下一次过零。

> `xc_flag` 注释里写的三个含义（0-换相 / 1-消磁 / 2-消磁结束）与实际用法略有出入，以上面的流程为准。

**③ `TM4_Isr`（interrupt 20）——堵转看门狗**

闭环下如果长时间没换相，直接置 `MOTOR_STOP_STALL`，兜底防止 PWM 一直烧某一相。

### 换相时间测量与滤波

- **T4** 是 0.5µs 时基的自由计时器，每次换相后被清零，用来量「本步走了多久」。
- `motor.commutation_time[6]` 是一个 **6 元素滑动窗口**（一个电周期 6 步），`commutation_time_sum` 是窗口和，即**一个完整电周期的时间**。
- `filter_commutation_time_sum` 是**一阶低通滤波**结果，用于抗抖动：

```c
// 闭环换相中断里的系数是 7:1
motor.filter_commutation_time_sum = (motor.filter_commutation_time_sum * 7 + motor.commutation_time_sum * 1) >> 3;
// 开环里的系数是 3:1
motor.filter_commutation_time_sum = (motor.filter_commutation_time_sum * 3 + motor.commutation_time_sum * 1) >> 2;
```

### 换相错误判断

闭环稳定后（`commutation_num > BLDC_CLOSE_LOOP_WAIT`），检查本步用时是否落在「整圈时间的 30/360 ~ 90/360」区间内：

```c
if((temp_commutation_time > (motor.filter_commutation_time_sum * 30 / 360)) &&
   (temp_commutation_time < (motor.filter_commutation_time_sum * 90 / 360)))
{
    if(motor.commutation_failed_num) motor.commutation_failed_num--;   // 正常，减计数
}
else
{
    motor.commutation_failed_num++;                                    // 异常，加计数
    if(BLDC_COMMUTATION_FAILED_MAX < motor.commutation_failed_num)
    {
        motor_stop();
        motor.run_flag = MOTOR_STOP_STALL;                             // 判定堵转
    }
}
```

### 六步换相表

换相顺序和比较器通道选择在 [bldc/pwm_out.c](bldc/pwm_out.c) 的 `pwm_x_output[6]` 函数指针数组里：

| step | 导通 | 比较器监测相 |
|---|---|---|
| 0 | A 上 / B 下 | C |
| 1 | A 上 / C 下 | B |
| 2 | B 上 / C 下 | A |
| 3 | B 上 / A 下 | C |
| 4 | C 上 / A 下 | B |
| 5 | C 上 / B 下 | A |

每个函数做两件事：设置 `PWMA_ENO`（使能哪几路输出）和拉低对应下桥、`CMP_SELECT_x` 选择要监测的悬空相。

`g_use_complementary` / `BLDC_USR_COMPLEMENTARY` 控制是否开互补输出（`PWMA_ENO` 多带一位低桥）。

---

## 7. 定时器与中断资源分配

**这是最容易踩坑的地方——资源冲突不会报错，只会表现为电机抽风。**

| 中断 | 向量号 | 周期/触发 | 职责 | 实现位置 |
|---|---|---|---|---|
| `TM0_Isr` | 1 | 进角延时 | 换相 + 消磁延时 | [motor_control.c](bldc/motor_control.c) |
| `TM1_Isr` | 3 | **50µs 固定** | **主状态机 + LED + 电压检测** | [pit_timer.c](bldc/pit_timer.c) |
| `DMA_UART1_IRQHandler` | 4 | 串口收字节 | 串口接收 / 自动下载 | [isr.c](project/user/isr.c) |
| `TM2_IRQHandler` | 12 | — | 预留回调 | [isr.c](project/user/isr.c) |
| `DMA_UART3/4` | 17/18 | 串口收字节 | 预留回调 | [isr.c](project/user/isr.c) |
| `TM3_IRQHandler` | 19 | — | 预留回调 | [isr.c](project/user/isr.c) |
| `TM4_Isr` | 20 | 换相超时 | 堵转看门狗 | [motor_control.c](bldc/motor_control.c) |
| `comparator_isr` | 21 | 过零事件 | 反电动势过零捕获 | [motor_control.c](bldc/motor_control.c) |
| `TM11_IRQHandler` | 24 | — | 预留回调 | [isr.c](project/user/isr.c) |
| `pwmb_isr` | 27 | PWM 边沿 | 油门输入捕获 | [signal_input.c](bldc/signal_input.c) |

### 优先级设置（`motor_init()` 里）

```
IP/IPH  bit1  → 定时器0 优先级 3（最高）
IP2/IP2H bit5 → 比较器  优先级 3（最高）
IP2H     bit3 → PWMB 输入捕获 优先级 2
```

定时器0 和比较器同为最高优先级，因为它们直接决定换相时刻的精度。

### 定时器时基

| 定时器 | 用途 | 时基 | 设置 |
|---|---|---|---|
| T0 | 进角延时 | 0.5µs | `TM0PS = (system_clock/1000000/2) - 1` |
| T1 | 主循环 PIT | 50µs | `pit_us_init(TIM1_PIT, 50)` |
| T4 | 换相计时 | 0.5µs | `TM4PS = (system_clock/1000000/2) - 1`，1T 模式 |
| PWMA | 三相 PWM | 43.333kHz | `BLDC_PWM_ARR_MAX = 923`，40MHz/923 |
| PWMB | 输入捕获 | 1µs 分辨率 | `PWMB_PSCRL = system_clock/1000000 - 1` |

> 系统主频 40MHz（`clock_init(SYSTEM_CLOCK_40M)`）。注意 Keil 工程里器件属性写的是 `CLOCK(35000000)`，那是调试器/仿真设置，**不是实际主频**，实际以 `main.c` 的 `clock_init` 为准。改主频后 PWM 频率会跟着变，`BLDC_PWM_ARR_MAX` 需要重算。

---

## 8. 各模块详解

### `project/user/main.c` — 入口

初始化顺序**不能随意调整**：

```c
clock_init(SYSTEM_CLOCK_40M);   // 1. 时钟最优先
debug_init();                   // 2. 调试串口

battery_init();                 // 3. 电池电压采样
led_init();                     // 4. LED
pwm_input_init();               // 5. 油门输入捕获
comparator_init();              // 6. 比较器
motor_init();                   // 7. 电机（含 GPIO、定时器、鸣叫）

pwm_out_init();                 // 8. ⚠️ PWM 必须放在电机初始化之后！

pit_timer_init();               // 9. 50µs 周期中断

EA = 1;                         // 10. 开总中断
while(1) { }                    // 主循环是空的，所有工作在中断里
```

源码里 `pwm_out_init()` 那行**重复写了三遍警告注释**：「PWM初始化必须放在电机初始化之后，否则会烧电机」。务必保留这个顺序。

`while(1)` 是空的——这是中断驱动的架构，全部逻辑跑在中断里。

### `bldc/pit_timer.c` — 主控制循环（最重要）

- `TM1_Isr`：50µs 一次。`pit_count++` → 电压检测 → 更新 `run_flag`（IDLE/START 切换）→ `led_control()` → `pit_motor_control()`。
- `pit_motor_control()`：状态机主体，即第 5 节描述的全部内容。
- `led_control()`：按 `motor.run_flag` 驱动 RUN/ERR 灯，用 `TICK_TO_MS(ms) = ms*20` 换算（因为 50µs 一拍，1ms = 20 拍）。
- `pit_timer_init()`：`pit_us_init(TIM1_PIT, 50)` 后把 `run_flag` 置 `MOTOR_IDLE`。

**注意**：`MOTOR_START` 状态里的正弦强拉是**在中断里跑 `do-while` 阻塞循环**的，会占住 50µs 中断很久。这是已知设计（启动阶段不需要主循环响应），但改代码时要意识到。

### `bldc/motor_control.c` — 换相与中断

- `motor_next_step()`：step 在 0~5 循环。
- `tim4_reconfig()`：读取并清零 T4 计数值（部分代码路径用，实际换相路径直接操作寄存器）。
- `TM0_Isr` / `comparator_isr` / `TM4_Isr`：见第 6 节。
- `motor_power_on_beep()`：**用电机线圈当喇叭**——通过 PWM 在三相上轮流导通发出 523/587/659/698/783 Hz 的音阶，占空比即音量。上电鸣叫表示初始化完成。
- `motor_stop()`：占空比清零 + `pwm_close_output()` + 关比较器中断。
- `motor_init()`：GPIO 推挽初始化、变量清零、T0/T4 配置、中断优先级。最后按 `BLDC_BEEP_ENABLE` 决定是否鸣叫。

### `bldc/comparator.c` — 过零检测

- `comparator_rising()` / `comparator_falling()`：`CMPCR1` 配置上升/下降沿触发。
- `comparator_open_isr()`：**注意**里面按 `motor.step % 2` 决定用上升沿还是下降沿——换相后悬空相的变化方向是交替的，这是正确检测过零的关键。
- `comparator_init()`：P45/P44/P46/P50/P51 设为高阻输入，`CMPCR1 = 0x8C | 1<<1`，`CMPCR2 = 63 | 1<<6`（63 个时钟数字滤波，关闭 0.1µF 模拟滤波电容），`CMPO_S = 1` 选 P41 输出。
- `COMPARATOR_MID_PIN = P44` 是**虚拟中性点**，通常由三个等值电阻星形连接得到。

### `bldc/pwm_out.c` — PWM 输出与换相表

- `pwm_x_output[6]`：六步换相函数指针表（见第 6 节表格）。
- `pwm_out_init()`：PWMA 配置。`PWMA_PS = 0x55` 引脚映射，`PWMA_CCMRx = 0x78`（PWM 模式 2，预装载允许），`PWMA_DTR = BLDC_PWM_DEADTIME` 死区，`PWMA_BKR = 0x80` 主输出使能（总开关），`PWMA_CR1 = 0x85`。
- `pwm_out_duty_update()`：三路 CCR 同时写同一个值。
- `pwm_brake()`：关输出 + 三个下桥全开（能耗制动）。
- `pwm_close_output()`：六路全关。
- `delay_500ns()`：8 个 `_nop_()` 的软件延时。

### `bldc/sine_control.c` — 正弦启动

- `sine_pwm_arr[360]`：360 点正弦表，峰值 1024。
- `sine_init()`：三相位置设为 0 / 119 / 239（**相差 120°**），开启所有 PWM 通道。
- `sine_start(duty)`：三相位置各减 1（回绕到 359），查表乘占空比后写入三路 CCR：

```c
temp = ((uint32)sine_pwm_arr[motor_a_position] * duty) >> 10;   // >>10 即 /1024
```

在 `MOTOR_START` 里循环调用直到 `motor_a_position` 归零，即电角度转满一圈，用旋转磁场把转子强行拖动起来。

### `bldc/signal_input.c` — 油门输入

- `pwmb_isr`（interrupt 27）三段：
  - `SR1 & 0x02`：捕获周期 → `pwmin.period` → 算频率 `system_clock / (PSCRL+1) / period`。
  - `SR1 & 0x04`：捕获高电平 → `pwmin.high_value`。只在频率 30~400Hz 且高电平 1000~3000µs 范围内有效（标准 RC 信号）。
  - `SR1 & 0x01`：更新事件 → 超时检测，连续 2 次没收到信号就把油门清零（**失控保护**）。
- 油门换算：`motor.duty = (uint32)pwmin.throttle * BLDC_PWM_ARR_MAX / 1000`，即高电平 1000~2000µs 映射到 0~`BLDC_PWM_ARR_MAX`。
- `pwm_input_init()`：PWMB 双通道输入捕获，`SMCR = 0x54` 复位模式（TI1 上升沿复位计数器），`PSCRL = system_clock/1000000 - 1` 得到 1µs 分辨率。

> `pwmin.high_time` 直接就是 µs，因为预分频做成了 1µs 一拍。

### `bldc/battery.c` — 电池电压

- ADC 通过 **DMA 自动搬运**到 `adc_dma_buff[1][6]`，固定地址 `0x800`。
- ADC 左对齐（`ADCCFG = 0x0F`），代码读的是 `adc_dma_buff[0][0]` 这个 **uint8 高字节**，所以按 8 位精度换算：`pin_voltage = adc_reg_value * 3300 / 256`。
- `battery_voltage_get()` 每次调用都会重启一次 DMA 转换，并在返回 1 之前要求连续 2 秒低于 `BLDC_MIN_BATTERY`（`low_power_num/20 > 2000`，20 次/ms × 2000 = 2s），避免瞬时压降误触发。
- 因为它是在 50µs 的 `TM1_Isr` 里调用的，所以 `low_power_num` 累加很快。

### `project/user/isr.c` — 串口中断

DMA 方式的 UART1~4 接收中断，每个都有「接收完成」和「数据丢弃（缓冲区被覆盖）」两个分支。UART1 里带**串口自动下载**逻辑。文件底部注释掉了完整的向量号对照表，改中断向量时可以直接查。

### `libraries/` — 逐飞官方库

本项目实际用到的驱动：`zf_driver_gpio`、`zf_driver_pwm`、`zf_driver_timer`、`zf_driver_pit`、`zf_driver_delay`、`zf_driver_adc`、`zf_driver_uart`、`zf_driver_exti`，以及 `zf_common_clock/debug/typedef/interrupt`。

`zf_common_headfile.h` 是总头文件，`main.c` 和 `isr.c` 只 include 它。里面 `#pragma warning disable = 115/188` 屏蔽了未使用函数和未引用变量告警。

> 库文件改动请谨慎——升级官方库时会被覆盖。业务代码放 `bldc/`。

---

## 9. 关键数据结构

### `motor_struct`（[bldc/motor_control.h](bldc/motor_control.h)）

全局唯一实例 `motor`，**中断与主循环共享**：

| 字段 | 含义 |
|---|---|
| `step` | 换相步序 0~5 |
| `duty` | **目标**占空比（用户/输入捕获写，范围 0~`BLDC_PWM_ARR_MAX`） |
| `duty_register` | **当前**占空比（闭环里缓慢逼近 `duty`，实现加减速斜坡） |
| `run_flag` | 状态机的状态，取值见 `run_state_enum` |
| `motor_start_delay` | 开环启动换相延时 |
| `motor_start_wait` | 开环换相次数统计 |
| `restart_delay` | 停机后重启倒计时 |
| `commutation_failed_num` | 换相错误累计次数，超阈值判堵转 |
| `commutation_time[6]` | 最近 6 次换相时间（一个电周期） |
| `commutation_time_sum` | 上述 6 个之和 |
| `commutation_num` | 累计换相次数，用于状态切换和稳定判据 |
| `filter_commutation_time_sum` | 低通滤波后的电周期时间，**换相计时和进角都用它** |

**关键区分**：`duty` 是目标，`duty_register` 是实际输出。应用层只该改 `duty`，让闭环的斜坡逻辑去追。

### `pwmin_struct`（[bldc/signal_input.h](bldc/signal_input.h)）

`frequency` / `period` / `high_value` / `high_time` / `throttle`，由输入捕获中断填写。

### `xc_flag`（[bldc/motor_control.c](bldc/motor_control.c) 静态变量）

过零与换相之间的两段式握手标志，见第 6 节。**非 0 表示正处于消磁等待期**。

---

## 10. 调参指南

所有可调参数集中在 **[bldc/bldc_config.h](bldc/bldc_config.h)**，改完重新编译即可，不用动逻辑代码。

| 宏 | 默认值 | 含义 | 调整建议 |
|---|---|---|---|
| `BLDC_PWM_ARR_MAX` | 923 | PWM 最大占空比（ARR）。40MHz/923 ≈ 43.3kHz | 改主频必须重算。调 PWM 频率也改这里 |
| `BLDC_MAX_DUTY` | 10 | 启动最大占空比（百分之一，即 10%） | 限制启动电流，太小启不动，太大会烧管 |
| `BLDC_MIN_DUTY` | 5 | 启动最小占空比（5%） | 同上 |
| `BLDC_PWM_DEADTIME` | 5 | 上下桥死区 | **绝对不能设 0**，否则直通烧管 |
| `BLDC_MIN_BATTERY` | 8000 | 低压保护阈值（mV） | 按电池节数设 |
| `BLDC_COMMUTATION_FAILED_MAX` | 200 | 换相错误累计上限，超了判堵转 | 调小更敏感但易误停 |
| `BLDC_START_DELAY` | 10000 | 堵转后重启延时（×50µs = 500ms） | 太小会反复冲击 |
| `BLDC_CLOSE_LOOP_WAIT` | 200 | 闭环后再等多少次换相才开始加减速斜坡 | |
| `BLDC_OPEN_LOOP_WAIT` | 200 | 开环换相多少次后切闭环 | **最关键**：太小切早了会失步，太大启动慢且费电 |
| `BLDC_SPEED_INCREMENTAL` | 2 | 加减速响应，1 最快 20 最慢 | 注释建议从 20 开始一点点减小 |
| `BLDC_POLES` | 7 | **电机极对数** | ⚠️ 必须按实际电机改！错了换相全乱 |
| `BLDC_BEEP_ENABLE` | 1 | 上电鸣叫开关 | |
| `BLDC_BEEP_VOLUME` | 60 | 鸣叫音量 0~100 | |
| `BLDC_MOTOR_ANGLE` | 10 | 进角（度），过零后延时多少度换相，范围 0~30 | 影响效率与转矩，高速时可适当增大 |
| `BLDC_USE_SINE_START` | 1 | 用正弦启动（1）还是纯开环延时启动（0） | |
| `BLDC_USR_COMPLEMENTARY` | 1 | 是否启用互补输出 | |

### 调参顺序建议

1. **先确认 `BLDC_POLES` 极对数正确**——这是最容易搞错、后果最严重的参数。
2. 确认 `BLDC_PWM_ARR_MAX` 与主频匹配（`system_clock / 目标PWM频率`）。
3. 死区 `BLDC_PWM_DEADTIME` 按驱动管的开关速度调，宁大勿小。
4. 启动困难 → 调大 `BLDC_MIN_DUTY` / `BLDC_MAX_DUTY`，或调 `BLDC_OPEN_LOOP_WAIT`。
5. 运行中失步/停机 → 调大 `BLDC_SPEED_INCREMENTAL`（放慢加减速），或减小 `BLDC_COMMUTATION_FAILED_MAX` 的敏感度。
6. 高速效率差 → 试改 `BLDC_MOTOR_ANGLE`。

---

## 11. 新人上手步骤

### 第一天：把环境跑通

1. 装 **Keil µVision + PK251（C251）工具链**，确认 `C251.exe` 可用。ARM 版 MDK 打不开这个工程。
2. 打开 [project/mdk/seekfree.uvproj](project/mdk/seekfree.uvproj)，目标选 `AI8051U_32bit`。
3. **确认编码显示正常**——见第 13 节。Keil 里 `Edit → Configuration → Editor → Encoding` 要设成 ANSI（中文 Windows 下即 GBK）。
4. 编译，应该 `0 Error(s), 0 Warning(s)`。产物在 `project/mdk/out_file/SEEKFREE.hex`。
5. 用 STC-ISP 串口下载（点下载 → 板子上电/复位，见第 4 节）。
6. **上电应该听到电机鸣叫**——那是 `motor_power_on_beep()` 的自检提示音。听到就说明初始化和时钟都正常。

### 第二天：读懂流程

按这个顺序读，不要一上来就啃换相：

1. [project/user/main.c](project/user/main.c) —— 初始化顺序（10 行）
2. [bldc/motor_control.h](bldc/motor_control.h) —— `run_state_enum` 和 `motor_struct`，先建立概念
3. [bldc/pit_timer.c](bldc/pit_timer.c) 的 `TM1_Isr` + `pit_motor_control()` —— 状态机全貌
4. [bldc/pwm_out.c](bldc/pwm_out.c) 的 `pwm_x_output[6]` 表 —— 六步换相
5. 最后再啃第 6 节描述的三个中断协作

### 第三天：动手验证

- 改 `BLDC_BEEP_VOLUME`，重新编译烧录，听音量变化 —— 验证整条「改代码→编译→烧录」链路通。
- 用示波器看 P00~P05，确认六步换相波形和死区。
- 用示波器/逻辑分析仪看 P21，确认油门信号被正确捕获。
- 空载启动电机，观察 LED 状态迁移是否符合第 5 节的图。

---

## 12. 常见坑与注意事项

### 会烧硬件的

1. **`BLDC_PWM_DEADTIME` 不能设为 0**。上下桥直通 = 电源短路，必炸管。
2. **`pwm_out_init()` 必须放在 `motor_init()` 之后**。`main.c` 里连写了三遍警告注释，不是啰嗦，是踩过的坑。
3. **`BLDC_POLES` 必须和实际电机一致**。错了会导致换相时序完全错乱，可能堵转烧线圈。
4. 调 PWM 频率后记得重算 `BLDC_PWM_ARR_MAX`。

### 会让人怀疑人生的

5. **中文乱码**：源码是 GBK，编辑器要设对。用 VSCode 打开本工程已经配好了（`.vscode/settings.json`）；Keil 里要设 ANSI。**这是最高频的「假 bug」**。
6. **`project/mdk/out_file/` 是编译产物**，已 gitignore。不要提交，也不要在里面找源码。
7. **主循环 `while(1)` 是空的**。别以为代码没跑起来——全在中断里。
8. **`MOTOR_START` 在中断里阻塞执行正弦强拉**，这期间 50µs 周期中断被长时间占用。这是有意设计，但改这段代码要意识到时序影响。
9. **`motor.duty` 和 `motor.duty_register` 别混用**。前者是目标，后者是当前输出。直接改后者会绕过加减速斜坡。
10. **中断共享变量**：`motor` 结构体的字段同时被多个中断读写，加字段时要考虑是否需要原子性（比如 `commutation_time_sum` 是 `uint32`，251 上非原子）。
11. **Keil 器件属性里的 `CLOCK(35000000)` 不是实际主频**，实际是 `main.c` 里 `clock_init(SYSTEM_CLOCK_40M)` 的 40MHz。
12. 代码里有些注释是从 STC32G 移植时带过来的，与实际引脚/时序**不完全对应**（例如 `pit_timer.c` 里写「通过 PA2 引脚电平」但实际读的是 `CMPCR1 & 0x01` 寄存器位；堵转检测注释写「0.1ms 一次」但实际 PIT 是 50µs）。读代码以**代码本身**为准，注释仅作参考。

### 工程管理

13. 两个 `version.txt`：[bldc/bldc_version.txt](bldc/bldc_version.txt) 是业务版本，[libraries/doc/version.txt](libraries/doc/version.txt) 是驱动库版本。改了代码请更新前者。
14. `project/mdk/` 下两个文件名是**编码损坏的乱码**（`MDK….bat` 和 `project/code/….txt`），内容正常（清理脚本 / 一句 to do），是历史遗留，暂未处理。

---

## 13. 编码约定

### 源码：GBK / ANSI

**`bldc/`、`libraries/`、`project/user/` 下的所有 `.c` / `.h` 文件统一使用 GBK（ANSI）编码，无 BOM。**

原因：这是 Keil MDK C251 工具链和逐飞原库的原生编码。工程历史上曾是 GBK/UTF-8 混用状态，导致中文注释在不同工具里一半正常一半乱码，后统一为 GBK。

**新增或修改含中文的文件时务必注意**：

- 编辑器要设成 GBK/ANSI，否则保存时会把文件偷偷转成 UTF-8，破坏其他人的显示。
- Keil：`Edit → Configuration → Editor → Encoding` 选 ANSI。
- VSCode：工程内已配置 [.vscode/settings.json](.vscode/settings.json) 自动用 GBK 打开。
- 命令行工具/脚本写文件时要显式指定编码，不要用默认的 UTF-8。

### 例外：`.md` 文档用 UTF-8

**Markdown 文档（本 README）使用 UTF-8**，这是有意为之——GitHub 的 Markdown 渲染器强制按 UTF-8 解析，存成 GBK 的话网页上全是乱码。Keil 不读 `.md`，两者不冲突。VSCode 侧已通过语言级配置覆盖。

### 换行符

工程统一使用 **LF**。仓库级已设 `core.autocrlf=false`，保证签出后与仓库内容字节一致，避免 Keil 里看到的内容和提交的内容不一致。

---

## 附：相关链接

- 逐飞科技官网 / 淘宝：<https://seekfree.taobao.com/>
- AI8051U 开源库（逐飞）文档见 `libraries/doc/`
- 本仓库：<https://github.com/amzzzt/ai8051>

## 附：版本历史

| 版本 | 日期 | 变更 |
|---|---|---|
| V2.0.2 | 2026-03-31 | PWM 频率 48kHz → 43.333kHz，解决 20kHz 电磁信号源干扰问题 |
| V2.0.1 | 2026-03-25 | 增加电调稳定性 |
| V2.0.0 | 2026-01-09 | 从 STC32G 电调移植到 AI8051U |
