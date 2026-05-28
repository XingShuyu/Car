这是一个TI MSPM0G3507的循迹小车项目
引脚定义在 引脚定义.md 中
各个模块的功能都要封装在对应的文件夹中
编译使用`D:\\ElecCompetation\\TI\\Softwares\\CCStudio\\ccs\\utils\\bin\\gmake`和`D:/ElecCompetation/TI/Softwares/CCStudio/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe`
代码要求写清注释，尽量保持模块功能解耦

---

# 项目代码定位与导航

## 1. 项目概览

- **硬件平台**: TI MSPM0G3507 (LQFP-64, ARM Cortex-M0+)
- **功能**: 循迹竞速小车，8路灰度传感器循迹 + MPU6050姿态感知 + 超声波测距 + 编码器速度闭环
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
│   ├── usart.h / usart.c           # UART初始化与发送 (USART_Init, USART_SendData)
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
├── ultrasonic/                     # 超声波测距
│   └── ultrasonic.h / .c           # HC-SR04驱动 (Ultrasonic_Init, Ultrasonic_GetDistance)
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

| 区域 | 行号范围 | 功能 |
|------|---------|------|
| 全局变量声明 | 18–87 | PID参数、速度、时间戳、阶段索引、编码器计数等 |
| `main()` 初始化段 | 89–168 | 外设初始化、MPU6050校准、OLED按键选模式 |
| 主循环 | 170–578 | 两段定时：30ms编码器速度更新 + 10ms阶段状态机 |
| 阶段switch-case | 201–517 | 根据 `command[StageIndex]` 执行对应阶段逻辑 |
| `GROUP1_IRQHandler()` | 583–644 | GPIOA/GPIOB 编码器脉冲计数 + 按键中断 |
| `UART_0_INST_IRQHandler()` | 647–674 | 串口0接收中断（蓝牙数据） |
| `UART_MAIXCAM_INST_IRQHandler()` | 676–692 | MaixCAM视觉模块串口接收 |
| `process_imu_for_horizontal_motion()` | 694–705 | IMU偏航角积分（当前未在主循环启用） |
| `buzzer_beep()` | 707–716 | 蜂鸣器鸣响3声 |

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

HC-SR04驱动，Trig=PB19, Echo=PB17:
- `Ultrasonic_Init()`: 配置GPIO
- `Ultrasonic_GetDistance()`: 返回距离(cm)，超时返回0

当前在 `Stageultrasonic` 中被注释未启用。

### 3.7 OLED显示 (`OLED/`)

SSD1306 I2C驱动，使用 `I2C1` (PB3=SDA, PA17=SCL):
- `Display_Init()` + `Display_Clear()` + `Display_ShowString(row, col, str)`

### 3.8 基础库 (`BasicMicroLib/`)

- **delay**: `delay_us()`, `delay_ms()` — 循环延时
- **getTime**: `getNowMs()`, `getTimeMs()` — SysTick扩展时间戳（防uint32回绕）
- **usart**: `USART_Init()`, `USART_SendData()` — 串口
- **PID**: `PID_calculate()` — 通用PID计算（位置式，当前较少使用）

### 3.9 云台 (`Emm/`)

串口控制舵机协议，使用 `UART1`:
- `Emm_Init(addr)`: 使能指定地址舵机
- `Emm_Loc_Control(addr, ctrl)`: 位置/速度控制
- `Emm_Stop(addr)`: 停止

当前初始化代码在 `main.c` 中被注释。

## 4. 主循环执行时序

```
main() → 初始化各外设 → 按键选命令序列 → 进入 while(1):
  ├── 每30ms: 读取编码器增量 → NewMotorSpeedCtrl_UpdateByEncoderDelta() 更新速度闭环
  └── 每10ms: 读取 command[StageIndex] 执行阶段逻辑
       ├── Grayscale_Line() → NewMotorSpeedCtrl_SetTargetRobot() (循迹前进)
       ├── Grayscale_Cross() 判断路口触发阶段切换
       ├── _read_channel_stable() 判断旋转到位
       └── MPU6050_ReadAllCalibrated() IMU角度控转
```

## 5. 中断服务映射

| ISR | 触发源 | 功能 |
|-----|--------|------|
| `GROUP1_IRQHandler` | GPIOA/GPIOB双边沿 | 编码器AB相脉冲计数 + 按键计数 |
| `UART_0_INST_IRQHandler` | UART0 RX | 蓝牙串口数据接收 |
| `UART_MAIXCAM_INST_IRQHandler` | UART3 RX | MaixCAM视觉模块数据接收 |

## 6. 关键数据流

```
灰度传感器(8ch) → grayscale[8] → Grayscale_Line() → 转向角速度(wz)
                                                      ↓
编码器脉冲 → motorLeftCount/RightCount → NewMotorSpeedCtrl_UpdateByEncoderDelta()
                                              ↓
目标速度(BaseSpeed) + 转向修正(wz) → SetTargetRobot() → 增量式PID → PWM输出
```

## 7. 添加新功能指南

1. **新传感器驱动**: 在顶层新建独立文件夹（如 `NewSensor/`），配 `.h`+`.c`，在 `main.c` 中 include
2. **新阶段**: 在 `Stage.h` 的 `enum Stage` 中添加枚举值，在 `main.c` 的 switch 中添加 case 分支
3. **新命令序列**: 在 `Stage.h` 中定义新的 `commandN[]` 数组，加入 `commandList`
4. **修改PID参数**: 在 `main.c` 全局变量区（`BaseSpeed`, `RoundSpeed`, `grayscalePid`）或 `newmotor_speed_ctrl.c` 初始化参数中
5. **引脚变更**: 通过 CCS SysConfig 修改 `.syscfg` 文件，重新生成 `ti_msp_dl_config.c/h`

## 8. 注意事项

- `ti_msp_dl_config.c/h` 由 SysConfig 自动生成，**不应手动修改**
- 编码器计数在 ISR 中累加，主循环原子读取后清零（临界区用 `__disable_irq()`/`__enable_irq()` 保护）
- 当前 `StartTime`/`nowTime` 为全局变量，`TimeBase_Init()` 初始化 SysTick
- 四个命令序列 `command0~3` 按按键 `TextIndex` 选择，`TextIndex=3` 时额外选 Goal 终点路线
