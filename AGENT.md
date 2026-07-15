这是一个TI MSPM0G3507的循迹小车项目
引脚定义在 引脚定义.md 中
各个模块的功能都要封装在对应的文件夹中
编译使用`D:\\ElecCompetation\\TI\\Softwares\\CCS\\ccs\\utils\\bin\\gmake.exe`和`D:/ElecCompetation/TI/Softwares/CCStudio/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe`
SDK位置`D:\ElecCompetation\TI\Softwares\MSPM0_SDK`
代码要求写清注释，尽量保持模块功能解耦

---

# 项目代码定位与导航

## 1. 项目概览

- **硬件平台**: TI MSPM0G3507 (LQFP-64, ARM Cortex-M0+)
- **功能**: 循迹小车，8路灰度传感器循迹 + IMU转角控制 + 编码器速度闭环 + OLED显示 + UART通信
- **开发环境**: TI CCS (Code Composer Studio), SysConfig 自动生成 `ti_msp_dl_config.c/h`
- **SDK**: mspm0_sdk@2.10.00.04
- **编译链**: tiarmclang (LLVM) + gmake
- **主控逻辑**: `main.c` 只保留系统初始化、启动按键选择、主循环调度；阶段业务在 `Stage/`，电机运行时在 `Motor/`

## 2. 目录结构

```
Main/
├── main.c                          # 程序入口：初始化 + 路线选择 + 主循环调度
├── 引脚定义.md                      # 完整引脚映射表
├── AGENT.md                        # 本文件
│
├── Stage/                          # 比赛阶段与阶段状态机
│   ├── Stage.h                     # 阶段枚举、命令结构体、commandList声明
│   ├── stage_commands.c            # command0~command3命令序列定义
│   └── stage_runner.h / .c         # 阶段switch-case执行器
│
├── Motor/                          # 电机驱动、编码器与速度闭环
│   ├── newmotor_driver.h / .c      # 底层PWM驱动
│   ├── newmotor_speed_ctrl.h / .c  # 双轮速度闭环PID控制器
│   ├── motor_encoder.h / .c        # 编码器中断计数与原子读取/清零
│   ├── motor_runtime.h / .c        # 主循环10ms闭环更新、里程累计、目标速度接口
│   └── motor_step_test.h / .c      # 可选电机闭环阶跃测试
│
├── Drivers/                        # 板级中断分发与简单外设
│   ├── board_isr.h / .c            # GROUP1/UART ISR入口与分发
│   ├── button_select.h / .c        # 按键路线/Goal选择计数
│   └── buzzer.h / .c               # 蜂鸣器三声提示
│
├── Communication/                  # 应用层串口接收缓冲
│   ├── bluetooth_serial.h / .c     # UART0蓝牙CRLF分帧与recv0缓冲
│   └── maixcam_serial.h / .c       # MaixCam UART DMA接收与CRLF分帧
│
├── BasicMicroLib/                  # 基础库
│   ├── delay.h / delay.c           # 微秒/毫秒延时
│   ├── getTime.h / getTime.c       # SysTick时间基准
│   ├── usart.h / usart.c           # UART0 DMA收发与printf输出
│   └── PID.h / PID.c               # 通用PID结构体与计算函数
│
├── GrayScale/                      # 灰度传感器
│   ├── grayscale_sensor.h / .c     # 8通道CD4051读取与稳定单通道读取
│   └── Grayscale_Scan.h / .c       # 循线PID、十字/直角判断
│
├── IMU/                            # IMU惯性测量
│   ├── imu.h / imu_data.h          # 统一IMU入口与标准数据结构
│   ├── HWT101/hwt101.h / .c        # HWT101驱动
│   ├── JY901S/jy901s.h / .c        # JY901S/WT901驱动
│   └── MPU6050/mpu6050.h / .c      # MPU6050驱动
│
├── wdd35d4/                        # WDD35D4角位移传感器驱动（当前主流程未调用）
├── ultrasonic/                     # HC-SR04旧驱动（当前主流程未调用）
├── OLED/                           # OLED底层驱动与显示封装
├── Emm/                            # 云台舵机串口协议
└── Debug/                          # CCS/SysConfig自动生成和构建产物，不手动维护
```

## 3. 主程序与阶段系统

### 3.1 `main.c`

`main.c` 当前只负责：

| 区域 | 功能 |
|------|------|
| 初始化段 | `SYSCFG_DL_init()`、通信/中断/OLED/计时器/云台/电机/IMU初始化 |
| 启动选择 | `ButtonSelect` 在开机窗口内选择 `commandList[index]`；当 `index=3` 时再选择 `Goal` |
| 主循环 | `USART_PollTx()`、`MaixCamSerial_Poll()`、`MotorRuntime_Update()`、`StageRunner_Update()` |

当前基础速度配置在 `main.c` 的 `stageConfig`：
- `base_speed_mmps = 300`
- `round_speed_mmps = 150`

### 3.2 `Stage/`

`Stage/Stage.h` 定义阶段枚举与命令结构，`Stage/stage_commands.c` 定义4套命令序列：

```c
extern const StageCommand *const commandList[STAGE_COMMAND_LIST_COUNT];
```

常用阶段：

```c
StageRush       // 猛冲
StageRight      // 循迹 + 右直角判断
StageRightRound // 右原地旋转
StageLeft       // 循迹 + 左直角判断
StageLeftRound  // 左原地旋转
StageCross      // 十字路口检测 + L1/L2测距
StageStartJudge // 起点判断
StageFinsih     // 终点停车（历史拼写保留）
StageBizz       // 停车鸣笛
StageFake       // 假线过滤
StageStop       // 倒车退线实验逻辑，目前仍为注释保留
StageTurn       // IMU控角度转弯
StageSkip       // 根据Goal跳转路线
StageForward    // 编码器定距前进
```

`StageStandUp` 和 `Stageultrasonic` 枚举保留，但当前 `StageRunner_Update()` 没有实际分支。

### 3.3 添加或修改阶段

1. 新增阶段枚举：改 `Stage/Stage.h`
2. 新增阶段执行逻辑：改 `Stage/stage_runner.c`
3. 新增命令序列或调整路线：改 `Stage/stage_commands.c`
4. 调整默认速度：改 `main.c` 的 `stageConfig`，或在命令中使用 `StageForwardData`

## 4. 电机与中断数据流

### 4.1 电机模块

- `Motor/motor_encoder.c`: ISR中累计编码器增量，`MotorEncoder_ReadAndClear()` 原子读取并清零。
- `Motor/motor_runtime.c`: 持有 `NewMotor_SpeedCtrl motor`，每约10ms读取编码器增量、累计左右里程、调用 `NewMotorSpeedCtrl_UpdateByEncoderDelta()`。
- `StageRunner` 不直接访问编码器计数或电机控制器，只通过 `MotorRuntime_SetTargetWheelMmps()`、`MotorRuntime_SetTargetRobot()`、`MotorRuntime_ResetDistance()`、`MotorRuntime_Get*DistanceMm()` 访问运动状态。

### 4.2 中断映射

| ISR | 所在文件 | 功能 |
|-----|----------|------|
| `GROUP1_IRQHandler` | `Drivers/board_isr.c` | 查询GPIOA/GPIOB pending并分发给编码器和按键模块 |
| `UART_0_INST_IRQHandler` | `Drivers/board_isr.c` | 调用 `USART_IRQHandler()` 处理UART0 DMA收发 |
| `UART_MAIXCAM_INST_IRQHandler` | `Drivers/board_isr.c` | 调用 `MaixCamSerial_IRQHandler()` |

### 4.3 主循环时序

```
main()
  ├── SYSCFG_DL_init()
  ├── MaixCamSerial_Init() / BluetoothSerial_Init()
  ├── BoardIrq_Enable() / USART_Init()
  ├── Display_Init() / TimeBase_Init()
  ├── Emm_Init(1/2)
  ├── MotorRuntime_Init()
  ├── IMU_Init()
  ├── ButtonSelect选择commandList与Goal
  ├── StageRunner_Init(command, goal, stageConfig)
  └── while (1)
       ├── nowTime = getNowMs()
       ├── USART_PollTx()
       ├── MaixCamSerial_Poll()
       ├── MotorRuntime_Update(nowTime, getNowUs())
       └── StageRunner_Update(nowTime)
```

## 5. 主要模块说明

### 5.1 灰度传感器 (`GrayScale/`)

- `Grayscale_Sensor_Read_All(bool[8])`: 读全部8通道
- `_read_channel_stable(ch)`: 单通道稳定读取
- `Grayscale_Line(pid, sensor_values)`: 返回循线转向修正
- `Grayscale_Cross(sensor_values, status)`: 判断十字/直角；`status=0` 十字/丁字，`1` 右直角，`2` 左直角

### 5.2 IMU (`IMU/`)

上层阶段逻辑通过统一入口：
- `IMU_Init()`
- `IMU_ZeroYaw()`
- `IMU_ReadAll(IMU_Data_t *)`

当前 `StageTurn` 和 `StageForward` 使用 `IMUData.yaw` 做转角/直行修正。

### 5.3 通信 (`Communication/` + `BasicMicroLib/usart.c`)

- `BasicMicroLib/usart.c` 是UART0底层DMA收发与 `printf` 输出。UART0 RX 使用DMA块缓冲并在主循环轮询DMA进度，避免短帧必须等DMA块满。
- `Communication/bluetooth_serial.c` 是UART0接收字节回调，保留原 `recv0` 缓冲语义；只有收到完整 `\r\n` 后才发布一帧，缓存内容不包含分隔符。
- `Communication/maixcam_serial.c` 是MaixCam UART DMA接收缓冲；同样使用 `\r\n` 作为分隔，主循环必须调用 `MaixCamSerial_Poll()` 及时处理未填满DMA块的短帧。
- SysConfig DMA通道分配如下：

| DMA Channel | 外设 | 方向 | 用途 |
|-------------|------|------|------|
| `DMA_CH0` | `UART0: UART_0` | TX | UART0/蓝牙发送、`printf` 输出 |
| `DMA_CH1` | `UART0: UART_0` | RX | UART0/蓝牙接收 |
| `DMA_CH2` | `UART3: UART_MAIXCAM` | RX | MaixCam接收 |
| `DMA_CH3` | 未配置 | - | 预留 |

SysConfig 的 Channel Overview 左侧 `Channel 0/1/2/3` 是DMA通道号，右侧 `UART0/UART3` 是UART外设实例号；UART0 同时启用TX DMA和RX DMA，所以会占用两个DMA channel。`UART3` 不是 `DMA_CH3`，它只是MaixCam所用的UART外设编号。

### 5.4 WDD35D4与超声波

- `wdd35d4/` 驱动保留，`empty.syscfg` 中仍有 `WDD35D4_ADC`：`PB17` / `ADC1` / `channel 4`。
- 当前 `main.c` 和 `StageRunner` 不初始化、不读取 WDD35D4。
- `ultrasonic/` 是旧HC-SR04驱动；PB17已被WDD35D4 ADC占用，不要同时作为超声波Echo使用。

## 6. 开发注意事项

- `ti_msp_dl_config.c/h` 由 SysConfig 自动生成，不应手动修改。
- `Debug/*.mk` 是 CCS managed build 自动生成文件，不作为源码维护；新增 `.c` 文件后通过 CCS Build 重新生成。
- 新模块必须放在对应文件夹中，避免继续把业务逻辑堆回 `main.c`。
- 头文件 include 优先使用项目根路径形式，例如 `Stage/Stage.h`、`Motor/motor_runtime.h`。
- 保留历史注释和实验代码；若注释过期，优先修正说明，不直接删除。
