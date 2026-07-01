这是一个TI MSPM0G3507的循迹小车项目
引脚定义在 引脚定义.md 中
各个模块的功能都要封装在对应的文件夹中
编译使用`D:\\ElecCompetation\\TI\\Softwares\\CCStudio\\ccs\\utils\\bin\\gmake`和`D:/ElecCompetation/TI/Softwares/CCStudio/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe`
代码要求写清注释，尽量保持模块功能解耦

---

# 项目代码定位与导航

## 1. 项目概览

- **硬件平台**: TI MSPM0G3507 (LQFP-64, ARM Cortex-M0+)
- **功能**: 车载倒立摆/循迹小车，8路灰度传感器循迹 + WDD35D4摆杆角度检测 + MPU6050/JY901S姿态感知 + 编码器速度闭环
- **开发环境**: TI CCS (Code Composer Studio), SysConfig 自动生成 `ti_msp_dl_config.c/h`
- **SDK**: mspm0_sdk@2.10.00.04
- **编译链**: tiarmclang (LLVM) + gmake
- **主控逻辑**: `main.c` — 全部业务逻辑集中在主循环，通过 `Stage.h` 定义的阶段枚举驱动状态机

## 2. 目录结构

```
Main/
├── main.c                          # 主程序：初始化 + 阶段状态机主循环 + ISR
├── Stage.h                         # 阶段枚举定义 + 命令序列 (commandList)
├── 引脚定义.md                      # 完整引脚映射表
├── AGENT.md                        # 本文件
├── CLAUDE.md                       # 项目约束说明
│
├── BasicMicroLib/                  # 基础库（与硬件抽象无关的工具）
│   ├── delay.h / delay.c           # 微秒/毫秒延时 (delay_us, delay_ms)
│   ├── getTime.h / getTime.c       # SysTick时间基准 (getNowMs, getTimeMs)
│   ├── usart.h / usart.c           # UART初始化与蓝牙串口TX DMA发送
│   └── PID.h / PID.c               # 通用PID结构体与计算函数 (PID_calculate)
│
├── Motor/                          # 电机驱动与速度闭环
│   ├── newmotor_driver.h / .c      # 底层PWM驱动 (NewMotor_SetWheelPwmTicks, NewMotor_Stop)
│   └── newmotor_speed_ctrl.h / .c  # 速度闭环PID控制器 (NewMotorSpeedCtrl_*, 增量式PID)
│
├── GrayScale/                      # 灰度传感器
│   ├── grayscale_sensor.h / .c     # 传感器底层读取 (8通道CD4051模拟开关)
│   └── Grayscale_Scan.h / .c       # 循线算法 + 十字/直角判断 (Grayscale_Line, Grayscale_Cross)
│
├── IMU/                            # IMU惯性测量
│   ├── imu.h / imu_data.h          # 统一IMU入口与标准数据结构
│   ├── MPU6050/mpu6050.h / .c      # MPU6050 I2C驱动 (含校准、低通滤波、零偏补偿)
│   └── JY901S/jy901s.h / .c        # JY901S/WT901 I2C驱动
│
├── wdd35d4/                        # WDD35D4角位移传感器
│   └── wd35d4.h / .c               # PB17/ADC1_CH4采样 + 标定 + 默认直通角度换算
│
├── ultrasonic/                     # 超声波测距
│   └── ultrasonic.h / .c           # HC-SR04旧驱动；PB17已改作WDD35D4 ADC后不再初始化
│
├── OLED/                           # 显示屏
│   ├── oled.h / oled.c             # SSD1306底层I2C驱动
│   └── display.h / display.c       # 上层显示封装 (Display_ShowString, Display_Clear)
│
├── Emm/                            # 云台舵机 (串口控制)
│   └── Emm.h / Emm.c               # Emm舵机协议 (Emm_Init, Emm_Loc_Control, Emm_Stop)
│
└── Debug/                          # SysConfig自动生成
    └── ti_msp_dl_config.h / .c     # 引脚/外设/中断配置（由SysConfig生成，勿手动改）
```

## 3. 模块详解

### 3.1 主程序 (`main.c`)

**核心入口**，包含以下关键部分：

| 区域 | 功能 |
|------|------|
| 全局变量声明 | PID参数、速度、时间戳、阶段索引、编码器计数、WDD35D4实时数据等 |
| `main()` 初始化段 | 外设初始化、WDD35D4自动零点校准、IMU初始化、OLED按键选模式 |
| 主循环 | 两段定时：编码器速度闭环更新 + 阶段状态机 |
| 阶段switch-case | 根据 `command[StageIndex]` 执行对应阶段逻辑 |
| `GROUP1_IRQHandler()` | GPIOA/GPIOB 编码器脉冲计数 + 按键中断 |
| `UART_0_INST_IRQHandler()` | 串口0接收中断（蓝牙数据）+ TX DMA完成 |
| `UART_MAIXCAM_INST_IRQHandler()` | MaixCAM视觉模块串口接收 |
| `process_imu_for_horizontal_motion()` | IMU偏航角积分（当前未在主循环启用） |
| `buzzer_beep()` | 蜂鸣器鸣响3声 |

**关键全局变量**:
- `motorLeftCount` / `motorRightCount`: 编码器脉冲累计（ISR中更新，主循环读取并清零）
- `StageIndex` / `StageFlag`: 阶段状态机索引与子状态
- `TextIndex`: 按键选择的命令序列编号
- `grayscale[8]`: 8路灰度传感器二值化状态
- `BaseSpeed` (默认450) / `RoundSpeed` (默认200): 基础线速度和旋转速度

### 3.2 阶段系统 (`Stage.h`)

定义了比赛的全部动作阶段：

```c
enum Stage {
    StageRush=1,       // 猛冲
    StageRight=2,      // 循迹+右转判断
    StageRightRound=3, // 右原地旋转
    StageLeft=4,       // 循迹+左转判断
    StageLeftRound=5,  // 左原地旋转
    StageCross=6,      // 十字路口检测+L1/L2测距
    Stageultrasonic=7, // 超声波避障（已注释未启用）
    StageStartJudge=8, // 起点判断（AB/AD路线分支）
    StageFinsih=9,     // 终点停车
    StageBizz=10,      // 停车鸣笛
    StageFake=11,      // 假线过滤
    StageStop=12,      // 倒车退线
    StageTurn145=13,   // IMU控角度145°转弯
    StageSkip=14       // 跳转（根据Goal选路线终点）
};
```

`commandList[]` 包含4套预置命令序列（对应按键选择），`command0`~`command3`。

### 3.3 电机模块 (`Motor/`)

**`newmotor_driver.h/c`** — 底层PWM驱动:
- `NewMotor_InitPwm()`: 初始化左右电机PWM定时器
- `NewMotor_SetWheelPwmTicks(left, right)`: 设置PWM（含死区补偿），正=前进负=后退
- `NewMotor_Stop(mode)`: 停止（COAST滑行 / BRAKE电子刹车）
- `NewMotor_EncoderDeltaToDistanceMm(counts)`: 编码器增量→位移(mm)
- `NewMotor_EncoderDeltaToWheelSpeedMmps(counts, dt)`: 编码器增量→速度(mm/s)
- 机械参数宏: `NEWMOTOR_WHEEL_DIAMETER_MM`(66mm), `NEWMOTOR_WHEEL_BASE_MM`(45mm), `NEWMOTOR_GEAR_RATIO`(28:1)

**`newmotor_speed_ctrl.h/c`** — 速度闭环PID:
- `NewMotorSpeedCtrl_Init()`: 初始化双轮速度控制器
- `NewMotorSpeedCtrl_SetTargetWheelMmps()`: 设左右轮目标速度(mm/s)
- `NewMotorSpeedCtrl_SetTargetRobot()`: 差速底盘模型（vx线速度 + wz角速度→左右轮速度）
- `NewMotorSpeedCtrl_UpdateByEncoderDelta()`: 每周期调用，增量式PID计算并输出PWM

### 3.4 灰度传感器 (`GrayScale/`)

**`grayscale_sensor.h/c`** — 底层8路读取:
- 通过 CD4051 模拟开关选通8通道，`AD0/AD1/AD2` 三位地址线选通道
- `Grayscale_Sensor_Read_All(bool[8])`: 读全部8通道
- `_read_channel_stable(ch)`: 单通道稳定读取（带滤波）
- 阈值归一化: `THRESHOLD=1000`

**`Grayscale_Scan.h/c`** — 循线算法:
- `Grayscale_Line(pid, sensor_values)`: 返回转向修正值（角速度 rad/s），内部使用位置式PID处理偏差
- `Grayscale_Cross(sensor_values, status)`: 判断十字/直角 (status: 0=十字/丁字, 1=右直角, 2=左直角)
- `Grayscale_Zero(sensor_values)`: 重置传感器状态（中间两个传感器设为1）
- `Grayscale_OnlineNum(sensor_values)`: 返回在线传感器数量
- 传感器权重: `{-3.0, -2.7, -2.5, -2.0, 2.0, 2.5, 2.7, 3.0}` (0最左, 7最右)

### 3.5 IMU (`IMU/`)

`IMU/imu_data.h` 定义统一数据结构 `JY901S_Data_t`，并提供 `IMU_Data_t` 与旧
`MPU6050_Data_t` 兼容别名。上层业务代码优先使用 `JY901S_Data_t` 或 `IMU_Data_t`。

完整MPU6050 I2C驱动，使用 `I2C0` (PA28=SDA, PA31=SCL)，地址 0x68:

| API | 功能 |
|-----|------|
| `MPU6050_Init()` | 默认初始化 (±2g, ±250°/s) |
| `MPU6050_CalibrateGyro(n)` | 静止校准陀螺仪零偏 |
| `MPU6050_ReadAll(data)` | 读加速度(g)+角速度(°/s)+温度 |
| `MPU6050_ReadAllCalibrated(data)` | 读数据并执行零偏扣除+死区+低通滤波 |
| `MPU6050_SetFilter(alpha, gyroDz, accelDz)` | 设置低通滤波系数和死区 |

当前在主循环中 `MPU6050_ReadAllCalibrated` 仅用于 `StageTurn145` 的IMU角度积分。

### 3.6 超声波 (`ultrasonic/`)

HC-SR04旧驱动，原设计Trig=PB19, Echo=PB17:
- `Ultrasonic_Init()`: 配置GPIO
- `Ultrasonic_GetDistance()`: 返回距离(cm)，超时返回0

当前PB17已在SysConfig中配置为 `WDD35D4_ADC` 的ADC输入，`main.c` 不再调用
`Ultrasonic_Init()`。如果恢复超声波，需要重新分配Echo引脚或取消WDD35D4 ADC占用。

### 3.7 WDD35D4角位移传感器 (`wdd35d4/`)

WDD35D4按模拟电位器型角度传感器使用，滑动端接 `PB17`，SysConfig实例名
`WDD35D4_ADC`，外设为 `ADC1`，通道为 `DL_ADC12_INPUT_CHAN_4`。

**主要API**:
- `WDD35D4_Init()`: 在 `SYSCFG_DL_init()` 后调用，重置标定/滤波状态并使能ADC
- `WDD35D4_SetCalibration(raw_min, raw_max)`: 设置有效ADC量程
- `WDD35D4_GetCalibration(raw_min, raw_max)`: 读取当前ADC标定范围
- `WDD35D4_CalibrateZero(sample_count)`: 自动采样当前ADC平均值作为竖直零点
- `WDD35D4_SetZeroRaw(zero_raw)`: 设置竖直平衡点ADC零点
- `WDD35D4_GetZeroRaw()`: 读取当前竖直零点ADC值
- `WDD35D4_SetDirection(direction)`: 设置角度方向，`direction < 0` 时取反
- `WDD35D4_GetDirection()`: 读取当前角度方向，返回 `1` 或 `-1`
- `WDD35D4_ReadRaw(raw)`: 触发一次ADC转换并返回原始值
- `WDD35D4_ReadFilteredRaw(alpha, filtered_raw)`: 原始值读取；`alpha=0` 时直通，`0<alpha<=1` 时一阶低通
- `WDD35D4_ReadVoltage(voltage)`: 读取滑动端电压
- `WDD35D4_ReadAngleDeg(angle_deg)`: 输出0到标定角度范围的单端角
- `WDD35D4_ReadSignedAngleDeg(angle_deg)`: 输出相对竖直零点的有符号角，用于倒立摆控制
- `WDD35D4_ReadData(data)`: 按默认滤波设置一次读取raw、电压、单端角、有符号角；当前默认不使用软件低通
- `WDD35D4_RawToVoltage/AngleDeg/SignedAngleDeg()`: 原始ADC值换算辅助函数

**零点校准流程**:
启动时 `main.c` 在 `WDD35D4_Init()` 后调用 `WDD35D4_CalibrateZero(32)`，
因此上电时应保持摆杆竖直。若ADC采样失败，则回退到默认零点 `1197`。

### 3.8 OLED显示 (`OLED/`)

SSD1306 I2C驱动，使用 `I2C1` (PB3=SDA, PA17=SCL):
- `Display_Init()` + `Display_Clear()` + `Display_ShowString(row, col, str)`

### 3.9 基础库 (`BasicMicroLib/`)

- **delay**: `delay_us()`, `delay_ms()` — 循环延时
- **getTime**: `getNowMs()`, `getTimeMs()` — SysTick扩展时间戳（防uint32回绕）
- **usart**: `USART_Init()`, `USART_SendData()`, `USART_WriteAsync()` — 串口；`UART_0` 为蓝牙串口，115200，TX 使用 `DMA_CH0` 搬运环形缓冲到 `UART0->TXDATA`
- **PID**: `PID_calculate()` — 通用PID计算（位置式，当前较少使用）

### 3.10 云台 (`Emm/`)

串口控制舵机协议，使用 `UART1`:
- `Emm_Init(addr)`: 使能指定地址舵机
- `Emm_Loc_Control(addr, ctrl)`: 位置/速度控制
- `Emm_Stop(addr)`: 停止

当前初始化代码在 `main.c` 中被注释。

## 4. 主循环执行时序

```
main()
  ├── SYSCFG_DL_init() 初始化SysConfig外设
  ├── WDD35D4_Init() → WDD35D4_CalibrateZero(32) 自动校准竖直零点
  ├── 电机、IMU、OLED、串口等模块初始化
  ├── 按键选择 commandList/Goal
  └── 进入 while(1):
       ├── 周期读取编码器增量 → NewMotorSpeedCtrl_UpdateByEncoderDelta() 更新速度闭环
       ├── 周期读取WDD35D4角度并刷新显示
       └── 每10ms读取 command[StageIndex] 执行阶段逻辑
            ├── Grayscale_Line() → NewMotorSpeedCtrl_SetTargetRobot() (循迹前进)
            ├── Grayscale_Cross() 判断路口触发阶段切换
            ├── _read_channel_stable() 判断旋转到位
            └── IMU_ReadAll() / MPU6050_ReadAllCalibrated() 用于姿态与转角控制
```

## 5. 中断服务映射

| ISR | 触发源 | 功能 |
|-----|--------|------|
| `GROUP1_IRQHandler` | GPIOA/GPIOB双边沿 | 编码器AB相脉冲计数 + 按键计数 |
| `UART_0_INST_IRQHandler` | UART0 RX / DMA_DONE_TX | 蓝牙串口数据接收；TX DMA完成后推进发送环形缓冲 |
| `UART_MAIXCAM_INST_IRQHandler` | UART3 RX | MaixCAM视觉模块数据接收 |

## 6. 关键数据流

```
灰度传感器(8ch) → grayscale[8] → Grayscale_Line() → 转向角速度(wz)
                                                      ↓
编码器脉冲 → motorLeftCount/RightCount → NewMotorSpeedCtrl_UpdateByEncoderDelta()
                                              ↓
目标速度(BaseSpeed) + 转向修正(wz) → SetTargetRobot() → 增量式PID → PWM输出

WDD35D4(PB17 ADC) → raw → 标定/零点/方向 → signed_angle_deg
                                              ↓
                                  StageStandUp倒立摆控制
                                              ↓
                                  小车前后目标速度 → 双轮速度闭环
```

## 7. 添加新功能指南

1. **新传感器驱动**: 在顶层新建独立文件夹（如 `NewSensor/`），配 `.h`+`.c`，在 `main.c` 中 include
2. **新阶段**: 在 `Stage.h` 的 `enum Stage` 中添加枚举值，在 `main.c` 的 switch 中添加 case 分支
3. **新命令序列**: 在 `Stage.h` 中定义新的 `commandN[]` 数组，加入 `commandList`
4. **修改PID参数**: 在 `main.c` 全局变量区（`BaseSpeed`, `RoundSpeed`, `grayscalePid`）或 `newmotor_speed_ctrl.c` 初始化参数中
5. **引脚变更**: 通过 CCS SysConfig 修改 `.syscfg` 文件，重新生成 `ti_msp_dl_config.c/h`

## 8. 注意事项

- `ti_msp_dl_config.c/h` 由 SysConfig 自动生成，**不应手动修改**
- `empty.syscfg` 已配置 `WDD35D4_ADC`：`PB17` / `ADC1` / `channel 4`。不要同时把PB17用于超声波Echo。
- `main.c` 当前已移除 `Ultrasonic_Init()` 调用；超声波源码保留但不是当前运行链路。
- WDD35D4的 `ReadSignedAngleDeg()` 输出才适合倒立摆控制，普通 `ReadAngleDeg()` 是单端角。
- `WDD35D4_DEFAULT_FILTER_ALPHA` 当前为 `0`，表示默认不启用软件低通滤波；如需抑制噪声，调用 `WDD35D4_ReadFilteredRaw(alpha, ...)` 并传入 `0<alpha<=1`。
- `StageStandUp` 当前分支读取 `WDD35D4_ReadData()` 的 `signed_angle_deg`，即相对竖直零点的有符号角。
- 编码器计数在 ISR 中累加，主循环原子读取后清零（临界区用 `__disable_irq()`/`__enable_irq()` 保护）
- 蓝牙串口 `UART_0` 波特率为 115200，时钟源为 BUSCLK，发送为软件环形缓冲 + `DMA_CH0`。`printf/fputc/write` 只入队，`USART_PollTx()` 只在DMA和UART都空闲时启动下一段传输，不再逐字节轮询写TX FIFO。
- `UART_0_INST_IRQHandler()` 必须保留 `DL_UART_IIDX_DMA_DONE_TX` 和 `DL_UART_IIDX_EOT_DONE` 分支，并调用 `USART_HandleTxInterrupt()`；`DMA_DONE_TX` 只代表数据搬入FIFO，`EOT_DONE` 才代表线上的最后一字节发完。
- 115200 下连续日志吞吐约 11KB/s；主循环调试 `printf` 必须限速，否则 DMA 环形缓冲会丢字节，蓝牙端表现为断行、错位或类似乱码。
- 当前 `StartTime`/`nowTime` 为全局变量，`TimeBase_Init()` 初始化 SysTick
- 四个命令序列 `command0~3` 按按键 `TextIndex` 选择，`TextIndex=3` 时额外选 Goal 终点路线
