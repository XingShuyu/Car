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
- **功能**: 循迹小车，8/12路灰度传感器循迹 + IMU转角控制 + 编码器速度闭环 + OLED显示 + UART通信；UART2 额外接入 Jibot 六轴机械臂的基础单轴位置控制。
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
├── Arm/                            # Jibot 机械臂基础驱动
│   ├── jibot_servo.h / .c           # UART2 位置帧发送、中心相对角到 PWM 换算
│   ├── arm_control.h / .c           # 六轴 PWM 状态读取、菜单手动示教测试
│   ├── arm_motion_state.h / .c      # 六轴二维 PWM 动作表的非阻塞状态机
│   ├── arm_ik_motion.h / .c         # 自动优先钩爪朝下逆解、标定映射与非阻塞执行
│   └── arm_target_test.h / .c        # 硬编码 IK 目标的最小测试状态机
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
│   ├── button_select.h / .c        # 按键 Next/Start 事件与防抖
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
├── GrayScale/                      # 灰度模块统一适配层
│   ├── grayscale_sensor.h / .c     # 自动选择、统一 P1~P12 传感器接口
│   ├── Grayscale_Scan.h / .c       # 统一循迹、路口和原地转弯接口
│   ├── GrayScale8/                 # 8路 CD4051 后端（保留原PID和规则）
│   └── GrayScale12/                # 12路 NCHD12/PCA9555 后端与独立PID
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
| 初始化段 | `SYSCFG_DL_init()`、通信/中断/OLED/计时器/云台/电机/IMU初始化；计时器初始化后发送一次 Jibot 夹爪 `+5°`、1000 ms 的上电小幅测试命令 |
| 启动选择 | 常驻两按键菜单：PB23切换地图/目标点候选项，PB26立即启动；地图4展开为4个Goal候选项 |
| 主循环 | `USART_PollTx()`、`MaixCamSerial_Poll()`、`MotorRuntime_Update()`、`StageRunner_Update()` |

当前基础速度配置在 `main.c` 的 `stageConfig`：
- `base_speed_mmps = 300`
- `round_speed_mmps = 150`

### 3.2 `Stage/`

`Stage/Stage.h` 定义阶段枚举与命令结构，`Stage/stage_commands.c` 定义5套命令序列：

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
   ├── Display_Init() / Grayscale_Init() / TimeBase_Init()
  ├── Emm_Init(1/2)
  ├── MotorRuntime_Init()
  ├── IMU_Init()
  ├── 启动菜单读取ButtonSelect事件并选择commandList与Goal
  ├── StageRunner_Init(command, goal, stageConfig)
  └── while (1)
       ├── nowTime = getNowMs()
       ├── USART_PollTx()
       ├── MaixCamSerial_Poll()
       ├── MotorRuntime_Update(nowTime, getNowUs())
       └── StageRunner_Update(nowTime)
```

## 5. 主要模块说明

### 5.1 灰度模块统一适配层 (`GrayScale/`)

- `Grayscale_Init()` 在启动时优先检测并初始化 NCHD12；未应答时固定回退到 8 路 CD4051。运行中 I2C 读失败不会切换模块，可通过 `Grayscale_LastReadOk()` 获取读取状态。
- `Grayscale_Sensor_Read_All(bool[12])` 统一输出 P1（右）至 P12（左）。12 路后端完整输出 12 路；8 路后端映射到 P3~P10，P1/P2/P11/P12 为 0。
- `Grayscale_Line()`、`Grayscale_Cross()`、`Grayscale_Zero()`、`Grayscale_OnlineNum()` 与 `Grayscale_GetTurnControl()` 均按活动后端自动分发。
- 8 路继续使用原有 `Track_PID` 参数与输出；12 路使用独立 PID 和可单独设置的 `Grayscale_Set12Pid()`、`Grayscale_Set12IrrScale()`。
- 12 路十字/直角使用完整 P1~P12：右直角 P1~P4，左直角 P9~P12，十字 P3~P10；P1 位于车体右侧。

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

### 5.4 Jibot 机械臂控制接口 (`Arm/`)

机械臂 UART2 的控制分为五层：基础单轴/组帧驱动、示教读取、PWM 动作表、自动朝下笛卡尔逆解和菜单测试。上层不得自行拼接 Jibot 协议帧，应按需求选用下列公开接口。

#### 5.4.1 轴号、协议范围与通信

| ID/宏 | 机械轴 | 说明 |
|---|---|---|
| `JIBOT_SERVO_ID_BASE` (`0`) | 底座 yaw | 车头正前方零位由 IK 标定定义 |
| `JIBOT_SERVO_ID_SHOULDER` (`1`) | 肩俯仰 | IK 平面关节 q1 |
| `JIBOT_SERVO_ID_ELBOW` (`2`) | 肘俯仰 | IK 平面关节 q2 |
| `JIBOT_SERVO_ID_WRIST_PITCH` (`3`) | 腕俯仰 | IK 平面关节 q3 |
| `JIBOT_SERVO_ID_WRIST_ORIENTATION` (`4`) | 腕旋转 | 当前 IK 固定为标定 PWM |
| `JIBOT_SERVO_ID_GRIPPER` (`5`) | 夹爪 | 当前 IK 固定为标定 PWM |

- 协议位置范围：`PWM 500~2500`，等效中心相对角 `-135°~+135°`，`0° = PWM 1500`，换算为 `round(1500 + angle_deg * 2000 / 270)`。正角只表示 PWM 增大，不等同于机械正方向。
- 到位时间 `time_ms` 为 `0~9999 ms`；任意完整 Jibot 命令的最小间隔是 `JIBOT_SERVO_COMMAND_INTERVAL_MS`（当前 1 ms）。
- UART2 SysConfig 实例为 `JibotArm`：`PB15` 是 MSPM0 TX（接控制器 RX），`PB16` 是 MSPM0 RX（接控制器 TX），配置为 `115200 8N1`、无流控，双方必须共地。
- UART2 当前只供 Jibot 使用。`Communication/cc2530_zigbee.c` 是历史源码，运行时禁止初始化或调用；恢复 DL-LN33 前必须重新规划 SysConfig 与中断分发。

#### 5.4.2 基础驱动：`Arm/jibot_servo.h`

| 接口 | 用途与行为 |
|---|---|
| `JibotServo_SetPwm(id, position_pwm, time_ms)` | 单轴原始 PWM 位置控制；参数非法或 UART 命令无法提交时返回 `false`。 |
| `JibotServo_SetPwmBatch(position_pwm, time_ms)` | 一次发送 ID0~5 的同步组位置帧；`position_pwm` 必须是 6 个合法 PWM。六轴同步动作应优先使用它。 |
| `JibotServo_SetAngle(id, angle_deg, time_ms)` | 单轴中心相对角控制；仅负责角度到 PWM 的协议换算，不负责机械零位、方向或碰撞限位。 |
| `JibotServo_ReleaseTorque(id)` | 发送 `#dddPULK!` 释放单轴扭力。释放前必须支撑机械臂。 |
| `JibotServo_RestoreTorque(id)` | 发送 `#dddPULR!` 恢复单轴扭力；只保持当前位置，不下发新位置。 |
| `JibotServo_ReadPosition(id, &pwm, timeout_ms)` | 发送 `#dddPRAD!` 并阻塞等待匹配位置应答；成功写入原始 PWM。必须在 `TimeBase_Init()` 后调用，常用超时 20~100 ms。 |

单轴位置帧格式为 `#dddPppppTtttt!`，六轴组帧格式为 `{G0000#000P...!#001P...!...#005P...!}`。驱动层只确认命令已交给 UART，不确认机械臂已实际到位。

#### 5.4.3 示教与 PWM 动作表

- `ArmControl_ReadAllPwm(ArmControl_PwmState *state, timeout_ms)`：依次阻塞读取 ID0~5；`validMask` 的位 n 表示 `pwm[n]` 有效。全部成功才返回 `true`。
- `ArmControl_RunTeachTest()`：菜单 `ARM TEACH TEST` 的交互入口。进入立即释放六轴；B2 在“读取 PWM 并恢复扭力”与“再次释放扭力”之间切换；B1 退出并尝试恢复六轴扭力。
- `ArmMotionState_Start(state, sequence, now_ms)`：启动非阻塞 PWM 动作表。动作表类型为 `uint16_t pwm[frameCount][JIBOT_SERVO_COUNT]`，可用 `ARM_MOTION_SEQUENCE(pwm_table)` 包装。
- `ArmMotionState_Update(state, now_ms)`：主循环持续调用；每 500 ms 查询六轴。任一轴未进入 `ARM_MOTION_STATE_PWM_TOLERANCE`（当前 50 PWM）或查询失败时，重发当前帧；全部到位后再下发下一帧。返回 `Idle/Running/Completed/Failed`。
- `ArmMotionState_Reset(state)`：丢弃动作状态，不发送停止命令。

阶段命令可用 `StageArmMotionData` 配合 `STAGE_CMD_ARM_MOTION(&data)` 运行 PWM 动作表；阶段执行时会先刹停小车。

#### 5.4.4 自动朝下笛卡尔控制：`Arm/arm_ik_motion.h`

```c
bool ArmIkMotion_Start(float yaw_deg, float x_mm, float y_mm);
ArmMotionState_Status ArmIkMotion_Update(void);
void ArmIkMotion_Reset(void);
```

- 坐标约定：`yaw=0` 指向车头前方，从车顶看逆时针为正；`x` 是底座偏航后的水平径向距离（mm，必须不小于 0）；`y` 是相对底座旋转中心向上的高度（mm）。
- 末端方向不再由调用方指定。求解器在有限解析候选中优先选择钩爪方向最接近竖直向下（模型方向 180°）的姿态；方向误差相同才比较三个平面关节与当前位置的等权平方移动量。
- 默认几何为 `h=91 mm`、`L1=104 mm`、`L2=74.5 mm`、`L3=174 mm`；q1~q3 默认模型限位均为 ±130°。有效限位还会与标定零位、轴方向、协议 ±135° 和轴 PWM 软件限位求交。
- `Start()` 会阻塞读取 ID1~3 的当前位置，以选择较小移动量的候选；成功后下发一帧六轴 PWM 并启动内部 `ArmMotionState`。ID4/ID5 使用 `fixedPwm`，默认均为 1500。
- `ArmIkMotion_LastStartWasReachabilityFailure()` 用于区分失败原因：目标无解、模型角/PWM 越界、NaN/无穷、`x<0` 或 yaw 无法映射时返回 `true`；忙状态、标定错误、位置读取或发送失败时返回 `false`。
- `ArmIkMotion_SetCalibration()` / `ArmIkMotion_GetCalibration()` 读写 `ArmIkMotion_Calibration`。只能在 IK 不运行时设置；可校准几何尺寸、q1~q3 模型限位、ID0~3 零位/方向/PWM 限位、ID4/5 固定 PWM 与位置读取超时。

阶段命令使用 `StageArmIkMotionData { yawDeg, xMm, yMm }` 和 `STAGE_CMD_ARM_IK_MOTION(&data)`。阶段开始会刹停小车；不可达显示 `CANT REACH`，其余启动或运行失败显示 `ARM IK FAIL`。

#### 5.4.5 菜单 IK 测试：`Arm/arm_target_test.h`

| 接口 | 用途 |
|---|---|
| `ArmTargetTest_SubmitTarget(yaw_deg, x_mm, y_mm)` | 空闲、完成或失败状态下提交一次自动朝下 IK 目标。 |
| `ArmTargetTest_Update()` | 推进并返回 `Idle/Moving/Completed/CannotReach/Failed`。 |
| `ArmTargetTest_Reset()` | 重置测试与 IK 状态，不发送停止命令。 |
| `ArmTargetTest_Run()` | 菜单 `ARM TARGET TEST` 入口；函数内硬编码 `yaw_deg/x_mm/y_mm`，提交后仅轮询状态，B1 退出。 |

测试 OLED 状态：`TARGET MOVING`、`TARGET REACHED`、`CANT REACH`、`TARGET START FAIL`。该测试不读取 MaixCam/蓝牙数据，也不使用路线动作表。

### 5.5 WDD35D4与超声波

- `wdd35d4/` 驱动保留，`empty.syscfg` 中仍有 `WDD35D4_ADC`：`PB17` / `ADC1` / `channel 4`。
- 当前 `main.c` 和 `StageRunner` 不初始化、不读取 WDD35D4。
- `ultrasonic/` 是旧HC-SR04驱动；PB17已被WDD35D4 ADC占用，不要同时作为超声波Echo使用。

## 6. 开发注意事项

- `ti_msp_dl_config.c/h` 由 SysConfig 自动生成，不应手动修改。
- `Debug/*.mk` 是 CCS managed build 自动生成文件，不作为源码维护；新增 `.c` 文件后通过 CCS Build 重新生成。
- 不要在工程目录中保存临时编译目录、`.o`、`.d` 或额外的 `main.o` 等对象文件。CCS 的递归扫描可能把它们加入链接，引发重复定义；临时构建产物应放在工程目录外。
- 修改 `empty.syscfg` 后，必须让 CCS/SysConfig 重新生成 `Debug/ti_msp_dl_config.*` 和构建清单；不要手改这些生成文件。
- 使用 Jibot 前，确保机械臂处于无障碍位置。基础驱动没有反馈和安全限位，首次调试应从小角度、长时间开始。
- 新模块必须放在对应文件夹中，避免继续把业务逻辑堆回 `main.c`。
- 头文件 include 优先使用项目根路径形式，例如 `Stage/Stage.h`、`Motor/motor_runtime.h`。
- 保留历史注释和实验代码；若注释过期，优先修正说明，不直接删除。
