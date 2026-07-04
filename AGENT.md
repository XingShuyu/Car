这是一个 TI MSPM0G3507 的电赛小车项目。
引脚定义在 `引脚定义.md` 中。
各个模块的功能尽量封装在对应文件夹中。
编译使用 `D:\ElecCompetation\TI\Softwares\CCStudio\ccs\utils\bin\gmake` 和 `D:/ElecCompetation/TI/Softwares/CCStudio/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe`。
代码要求写清注释，尽量保持模块功能解耦。

# 项目代码定位与导航

更新时间: 2026-07-04

## 1. 项目概览

- 硬件平台: TI MSPM0G3507, ARM Cortex-M0+
- 开发环境: TI CCS, SysConfig, tiarmclang 4.0.4 LTS
- SysConfig 输入: `empty.syscfg`
- SysConfig 生成物: `Debug/ti_msp_dl_config.c` 和 `Debug/ti_msp_dl_config.h`
- 当前主程序: `main.c`
- 当前默认控制模式: `BALANCE_DIRECT_CONTROL_ENABLE` 为 1, 主循环执行倒立摆 IMU 直控 PWM, 阶段循迹状态机不会运行
- 当前主要传感器链路: 编码器计数 + IMU roll/gx + OLED + UART0 日志
- 当前被初始化但未进入控制链路的数据源: WDD35D4 ADC, MaixCAM RX 缓冲, `Stage.h` 命令序列

## 2. 当前项目结构

```text
Main/
├── main.c                         # 入口: 初始化、倒立摆主循环、阶段状态机代码、ISR
├── Stage.h                        # 阶段枚举和 command0~3 命令表
├── empty.syscfg                   # SysConfig 外设/引脚配置源文件
├── 引脚定义.md                    # 人工维护的引脚说明
├── AGENT.md                       # 本文件
├── README.md / README.html        # 项目说明或导出的说明文件
├── LQR.md                         # LQR 调参/说明资料
├── lqr_from_csv.py                # 离线 LQR/CSV 分析脚本
├── serial_csv_receiver.py         # 串口 CSV 采集脚本
├── et --hard e99178a              # 误生成的 git log 输出文件, 可清理
│
├── BasicMicroLib/                 # 基础库
│   ├── delay.c / delay.h          # SysTick 忙等延时
│   ├── getTime.c / getTime.h      # SysTick 扩展时间戳
│   ├── PID.c / PID.h              # 通用 PID 计算
│   └── usart.c / usart.h          # UART0 DMA TX、printf 重定向、串口发送
│
├── Motor/                         # 电机底层和速度闭环
│   ├── newmotor_driver.c / .h     # PWM 输出、停止模式、编码器换算
│   └── newmotor_speed_ctrl.c / .h # 双轮速度增量式 PID
│
├── GrayScale/                     # 8 路灰度传感器
│   ├── grayscale_sensor.c / .h    # CD4051 通道选择和数字 OUT 读取
│   └── Grayscale_Scan.c / .h      # 循迹误差、直角/十字判断
│
├── IMU/                           # IMU 统一入口和子驱动
│   ├── imu.c / imu.h              # HWT101/JY901S/MPU6050 探测与数据合并
│   ├── imu_data.h                 # IMU_Data_t/JY901S_Data_t 数据结构
│   ├── HWT101/                    # WIT HWT101 单轴 I2C 驱动
│   ├── JY901S/                    # JY901S/WT901 I2C 驱动
│   └── MPU6050/                   # MPU6050 I2C 驱动、滤波、Kalman roll/pitch
│
├── wdd35d4/                       # WDD35D4 角位移传感器 ADC 驱动
├── OLED/                          # SSD1306 OLED 底层和文本显示封装
├── Emm/                           # Emm 串口舵机/云台协议
├── ultrasonic/                    # HC-SR04 旧超声波驱动, 当前 .c 整体注释
├── targetConfigs/                 # CCS 调试目标配置
├── Debug/                         # CCS 自动生成的构建产物, 不应手动维护
└── .settings / .theia             # IDE 配置
```

## 3. 当前实际运行路径

`main()` 启动顺序:

1. `SYSCFG_DL_init()` 初始化 SysConfig 生成的时钟、GPIO、PWM、I2C、UART、ADC、DMA。
2. 使能 GPIOA/GPIOB 编码器/按键中断, 初始化 UART0, OLED, MaixCAM UART3 中断和 SysTick 时间基准。
3. 初始化 WDD35D4 并固定零点为 `990`, 但当前主循环没有读取 `WDD35D4Data`。
4. 使能两个 Emm 舵机地址 1 和 2。
5. 启动左右电机 PWM 定时器, 初始化速度闭环结构体, 但当前平衡分支直接输出 PWM, 没有调用 `NewMotorSpeedCtrl_UpdateByEncoderDelta()`。
6. `IMU_Init()` 自动探测 HWT101/JY901S/MPU6050, OLED 显示探测结果, 然后 `IMU_ZeroYaw()`。
7. 1 秒按键选择 `TextIndex`, 如果选到 3 再进入 1 秒 `Goal` 选择。
8. 鸣笛后进入 `while (1)` 主循环。

主循环当前有效逻辑:

```text
while (1)
  ├── nowTime = getNowMs()
  ├── USART_PollTx() 尝试启动 UART0 TX DMA
  ├── 每约 5 ms:
  │   ├── 原子读取并清零左右编码器计数
  │   ├── 计算左右轮速度和平均速度
  │   ├── IMU_ReadAll() 读取 roll/gx
  │   ├── roll 超过 +/-30 度: 刹车并重置平衡目标
  │   └── roll 正常: 用 rollError、gx、speedError 直接计算 outTicks 并写 PWM
  ├── 每约 20 ms:
  │   └── Balance_UpdateTarget() 平滑目标速度/转向并更新目标平衡角
  └── 阶段状态机:
      └── 由于 BALANCE_DIRECT_CONTROL_ENABLE=1, 条件为 false, 当前不会运行
```

如果把 `BALANCE_DIRECT_CONTROL_ENABLE` 改为 0, `main.c` 里的 `switch (command[StageIndex])` 才会执行 `Stage.h` 命令序列。

## 4. 阶段系统

`Stage.h` 定义:

| 阶段 | 值 | 作用 |
| --- | ---: | --- |
| `StageRush` | 1 | 前冲固定距离并减速停车 |
| `StageRight` | 2 | 循迹前进, 遇右直角进入下一阶段 |
| `StageRightRound` | 3 | 原地右转, 灰度通道到位后停车 |
| `StageLeft` | 4 | 循迹前进, 遇左直角进入下一阶段 |
| `StageLeftRound` | 5 | 原地左转, 灰度通道到位后停车 |
| `StageCross` | 6 | 连续检测路口并记录两段距离 `distence[0/1]` |
| `Stageultrasonic` | 7 | 旧超声波避障阶段, 当前 case 内有效代码全注释 |
| `StageStartJudge` | 8 | 根据左右直角判断起点分支 |
| `StageFinsih` | 9 | 终点停车鸣笛, 名称拼写保留为代码现状 |
| `StageBizz` | 10 | 停车鸣笛并进入下一阶段 |
| `StageFake` | 11 | 假线过滤, 有线则回退两个阶段, 无线则前进 |
| `StageStop` | 12 | 倒车退线, 再次压线后停车鸣笛并结束运行 |
| `StageTurn145` | 13 | 通过 IMU yaw 左右约 135 度转向 |
| `StageSkip` | 14 | 按 `Goal` 计算偏移跳转 |
| `StageStandUp` | 15 | 枚举已定义, 当前 `main.c` 没有对应 case |

命令表:

- `command0[] = {}`: 空命令表。若阶段模式开启且默认 `TextIndex=0`, 会读取越界。
- `command1[]`: 从 `StageStartJudge` 开始的长路线。
- `command2[]`: 另一套长路线, 包含 `StageCross`。
- `command3[]`: 带 `StageSkip` 和 `StageTurn145` 的路线。
- `commandList[]`: 将四套命令表暴露给 `main.c`。

## 5. 函数索引

### 5.1 `main.c`

| 函数 | 作用 |
| --- | --- |
| `main()` | 系统入口。初始化外设、模块、IMU、显示和电机, 执行当前倒立摆主循环, 保留循迹阶段状态机代码。 |
| `GROUP1_IRQHandler()` | GPIOA/GPIOB 中断分发。读取编码器 A/B 相更新 `motorLeftCount`/`motorRightCount`, 按键 B23 增加 `TextIndex`。 |
| `UART_0_INST_IRQHandler()` | UART0 RX 接收缓冲, 处理 `DMA_DONE_TX` 和 `EOT_DONE` 以推进 UART0 TX DMA 队列。 |
| `UART_MAIXCAM_INST_IRQHandler()` | UART3 接收 MaixCAM 字符串, 遇 `\0` 或 `\n` 后置 `maixcam_flag`。当前主循环解析逻辑被注释。 |
| `process_imu_for_horizontal_motion(float dt)` | 旧 IMU 偏航积分辅助函数。当前只在注释代码中出现, 未被有效调用。 |
| `clampf_local(value, min, max)` | 本文件内部浮点限幅函数, 用于平衡目标更新。 |
| `slew_to_target(current, target, rate, dt)` | 按最大变化率平滑逼近目标值。 |
| `Balance_ResetTarget(reset_center)` | 重置平衡中心角、速度角、速度命令、转向命令和速度滤波状态。 |
| `Balance_UpdateTarget(dt, avg_speed_mmps)` | 20 ms 外环。平滑目标速度/转向, 根据速度误差计算目标倾角, 稳定且近停时缓慢学习中心角。 |
| `buzzer_beep()` | 蜂鸣器鸣响三声, 同时操作 GPIOA16 和 GPIOB22。 |

`MOTOR_STEP_TEST_ENABLE` 为 0 时, `MotorClosedLoopStepTest()` 和 `MotorStepTest_IsReached()` 只有条件编译内的声明, 没有有效定义或调用。

### 5.2 `BasicMicroLib/`

| 函数 | 作用 |
| --- | --- |
| `delay_us()` | 基于 SysTick 当前值忙等微秒延时。 |
| `delay_ms()` | 调用 `delay_us(ms * 1000)` 实现毫秒延时。 |
| `SysTick_Handler()` | SysTick 溢出中断, 累加 `s_systickOverflow` 以扩展 24 位计数器。 |
| `TimeBase_Init()` | 使能 SysTick 中断。 |
| `getExtendedTicks()` | 内部函数, 原子式读取溢出次数和当前 SysTick 值, 返回扩展 tick。 |
| `getNowUs()` | 返回当前微秒时间戳。 |
| `getNowMs()` | 返回当前毫秒时间戳。 |
| `getTimeUs(now, last)` | 返回两个 `uint32_t` 微秒时间戳差值, 利用无符号回绕。 |
| `getTimeMs(now, last)` | 返回两个 `uint32_t` 毫秒时间戳差值, 利用无符号回绕。 |
| `PID_calculate()` | 通用位置/增量混合 PID 计算, 当前主要被灰度旧代码路径引用。 |
| `USART_RestoreIrq()` | 内部函数, 根据保存的 PRIMASK 恢复中断状态。 |
| `USART_SendByte_Blocking()` | 内部函数, 对非 UART0 端口做阻塞单字节发送。 |
| `USART_EnqueueByte()` | 内部函数, 将 UART0 待发送字节写入环形缓冲。 |
| `USART_StartTxDmaIfIdle()` | 内部函数, UART0 空闲时启动下一段 DMA TX。 |
| `USART_Init()` | 初始化 UART0 FIFO、DMA TX 事件和 RX/DMA/EOT 中断。 |
| `USART_SendData()` | UART0 使用异步环形缓冲, 其他 UART 使用阻塞发送。 |
| `USART_WriteAsync()` | 将字符串逐字节写入 UART0 TX 环形缓冲。 |
| `USART_PollTx()` | 主循环轮询, 空闲时启动 UART0 TX DMA。 |
| `USART_HandleTxInterrupt()` | UART0 DMA/EOT 中断处理, 推进 TX 环形缓冲尾指针。 |
| `USART_GetDroppedTxBytes()` | 返回 UART0 TX 环形缓冲满时丢弃的字节数。 |
| `_sys_exit()` | 非微库场景下避免半主机退出, 进入低功耗等待中断死循环。 |
| `fputc()` | `printf` 输出重定向到 UART0 队列, `\n` 前补 `\r`。 |
| `putchar()` | 调用 `fputc()`。 |
| `write()` | tiarmclang/newlib 常见输出入口, 将缓冲写到 `stdout`。 |

当前 SysConfig 中串口波特率:

- `UART_0`: 921600, GPIOA10 TX, GPIOA11 RX, UART0, DMA TX。
- `Emm`: 115200, UART1。
- `UART_MAIXCAM`: 9600, UART3。

### 5.3 `Motor/`

| 函数 | 作用 |
| --- | --- |
| `nm_abs_i16()` | 内部函数, 取 int16 绝对值为 uint16。 |
| `nm_from_u16_with_limit()` | 内部函数, 兼容旧接口时将无符号 PWM 限幅后转 int16。 |
| `NewMotor_ClampSignedPwmTicks()` | 将 PWM ticks 限制到 `+/-NEWMOTOR_PWM_MAX_TICKS`。 |
| `NewMotor_ApplyLetfDeadzoneTicks()` | 左轮死区补偿。函数名中 `Letf` 为代码现状拼写。 |
| `NewMotor_ApplyRightDeadzoneTicks()` | 右轮死区补偿。 |
| `nm_write_left()` | 内部函数, 根据方向写左轮两个 PWM 通道。 |
| `nm_write_right()` | 内部函数, 根据方向写右轮两个 PWM 通道。 |
| `NewMotor_InitPwm()` | 启动 PWM 定时器并滑行停车。当前 `main.c` 手动启动定时器, 没直接调用此函数。 |
| `NewMotor_SetWheelPwmTicksRaw()` | 直接写左右轮 PWM, 不做死区补偿。 |
| `NewMotor_SetWheelPwmTicks()` | 做左右轮死区补偿后写 PWM。当前倒立摆分支直接调用它。 |
| `NewMotor_Stop()` | 滑行或电子刹车。 |
| `NewMotor_EncoderDeltaToDistanceMm()` | 编码器增量换算为轮子位移 mm。 |
| `NewMotor_EncoderDeltaToWheelSpeedMmps()` | 编码器增量和采样周期换算为轮速 mm/s。 |
| `NewMotor_LeftRightToLinearSpeedMmps()` | 左右轮速度换算为车体线速度。 |
| `NewMotor_LeftRightToYawRateRadps()` | 左右轮速度换算为车体偏航角速度。 |
| `nm_clampf()` | 速度闭环内部限幅。 |
| `nm_pid_reset()` | 清空单轮增量式 PID 状态。 |
| `nm_pid_incre_step()` | 单轮增量式 PID 步进并限幅输出。 |
| `NewMotorSpeedCtrl_Init()` | 初始化双轮速度闭环结构体和默认 PID 参数。 |
| `NewMotorSpeedCtrl_SetPid()` | 设置左右轮同一组速度 PID 参数。 |
| `NewMotorSpeedCtrl_SetOutputLimit()` | 设置速度 PID 输出限幅。 |
| `NewMotorSpeedCtrl_SetTargetWheelMmps()` | 设置左右轮目标速度。阶段状态机大量使用, 当前默认模式下不进入阶段。 |
| `NewMotorSpeedCtrl_SetTargetRobot()` | 差速模型, 车体线速度/角速度转左右轮目标。 |
| `NewMotorSpeedCtrl_UpdateByEncoderDelta()` | 根据编码器增量更新速度闭环并输出 PWM。当前 `main.c` 没有调用。 |
| `NewMotorSpeedCtrl_ResetAndStop()` | 清 PID 状态、禁用闭环并停车。 |
| `NewMotorSpeedCtrl_GetMeasuredWheelMmps()` | 读当前估计左右轮速度。 |
| `NewMotorSpeedCtrl_GetOutputPwmTicks()` | 读当前闭环输出 PWM。 |

条件编译关闭的兼容接口:

- `Init_Motor_PWM()`
- `PWM_Control_Car()`
- `Motor_Stop()`
- `L1_control()`
- `R1_control()`
- `Motor_Init()`
- `Motor_SetSpeed()`
- `Motor_Brake()`

这些函数只有把 `NEWMOTOR_ENABLE_BSP_COMPAT_API` 或 `NEWMOTOR_ENABLE_OLD_MOTOR_API` 改为 1 时才会编译。

注意: `newmotor_driver.h` 声明了 `NewMotor_ApplyDeadzoneTicks()` 但 `.c` 没有定义。当前没有调用者, 但这是一个潜在链接错误。

### 5.4 `GrayScale/`

| 函数 | 作用 |
| --- | --- |
| `_delay_us()` | 内部包装, 调用 `delay_us()`。 |
| `_select_channel()` | 写 AD0/AD1/AD2 选择 CD4051 通道。 |
| `Read_OUT_value()` | 读取 `GrayS_OUT` 数字输入。旧 ADC 读取方案已注释。 |
| `_read_channel_stable()` | 选通道后延时, 丢弃一次读数, 再返回稳定读数。阶段转弯代码直接调用。 |
| `Grayscale_Sensor_Init()` | 空函数, 目前依赖 SysConfig 初始化 GPIO。 |
| `Grayscale_Sensor_Read_All()` | 读取 8 路灰度, 当前按 `7 - i` 反向映射到数组。 |
| `Grayscale_Sensor_Read_Main()` | 只读中间 2 到 5 通道。当前没有有效调用。 |
| `Grayscale_Sensor_Read_Other()` | 只读两侧 0/1 和 6/7 通道。当前没有有效调用。 |
| `Grayscale_Sensor_Read_Single()` | 参数检查后读取单通道。当前没有有效调用。 |
| `Track_PID()` | 灰度循迹 PID, 使用 `IRTrack_Integral` 和 `error_last`。注意当前没有更新 `error_last`。 |
| `Grayscale_Line()` | 读取全部灰度并按手写逻辑生成循迹误差, 再经 `Track_PID()` 返回转向输出。 |
| `Grayscale_Cross()` | 读取全部灰度并判断左直角、右直角或十字/T 字。 |
| `Grayscale_Zero()` | 初始化灰度数组为中线状态, 清误差和积分。 |
| `Grayscale_OnlineNum()` | 读取全部灰度并统计在线通道数量。当前没有有效调用。 |

### 5.5 `IMU/imu.c`

| 函数 | 作用 |
| --- | --- |
| `IMU_Init()` | 探测 HWT101/JY901S/MPU6050, 初始化可用设备, 设置 MPU6050 滤波和零偏, 返回 ready mask。 |
| `IMU_GetReadyMask()` | 返回当前探测到的 IMU 设备位图。 |
| `IMU_IsDeviceReady()` | 查询指定 IMU 是否就绪。 |
| `read_mpu6050()` | 内部函数, 读取 MPU6050 校准数据并积分 yaw。 |
| `fill_from_mpu6050()` | 内部函数, 把 MPU6050 数据拷贝进 merged 数据。 |
| `fill_from_jy901s()` | 内部函数, 以 JY901S 全量数据覆盖 merged 数据, 保留 source mask。 |
| `fill_from_hwt101()` | 内部函数, 只用 HWT101 的 gy/gz/yaw 覆盖 merged 对应字段。 |
| `IMU_ReadAll()` | 依次读可用 MPU6050/JY901S/HWT101 并合并为 `IMU_Data_t`。 |
| `IMU_ZeroYaw()` | 对 HWT101/JY901S 发送清 yaw 命令, 对 MPU6050 清软件积分 yaw。 |

### 5.6 `IMU/MPU6050/`

| 函数 | 作用 |
| --- | --- |
| `delay_loop()` | 内部 NOP 延时。 |
| `make_i16()` | 高低字节合成有符号 16 位值。 |
| `reset_kalman_state()` | 清 roll/pitch Kalman 状态和上次时间。 |
| `accel_scale_from_fs()` | 根据加速度满量程返回 LSB 到 g 的比例。 |
| `gyro_scale_from_fs()` | 根据陀螺仪满量程返回 LSB 到 dps 的比例。 |
| `reset_filter_state()` | 清低通滤波历史和 Kalman 状态。 |
| `kalman_update()` | 一维 Kalman 更新角度和 bias。 |
| `update_kalman_angles()` | 用加速度角和陀螺角速度更新 roll/pitch。 |
| `i2c_wait_idle()` | 等待 MPU6050 I2C 总线空闲。 |
| `i2c_wait_stop_or_nack()` | 等待 STOP 或 NACK 并返回是否成功。 |
| `write_reg()` | 写单个 MPU6050 寄存器。 |
| `read_regs()` | 从指定寄存器连续读取数据。 |
| `apply_gyro_offset()` | 扣除陀螺仪零偏。 |
| `apply_accel_offset()` | 扣除加速度零偏。 |
| `apply_deadzone()` | 小于阈值时输出 0。 |
| `apply_filter()` | 对加速度/角速度做一阶低通滤波。 |
| `MPU6050_WhoAmI()` | 读取 WHO_AM_I。 |
| `MPU6050_Reset()` | 软件复位 MPU6050 并清滤波状态。 |
| `MPU6050_SetGyroFS()` | 设置陀螺仪量程并更新比例。 |
| `MPU6050_SetAccelFS()` | 设置加速度量程并更新比例。 |
| `MPU6050_Init()` | 使用默认配置初始化。 |
| `MPU6050_InitWithConfig()` | 按指定配置初始化电源、DLPF、采样率和量程。 |
| `MPU6050_ReadAccelRaw()` | 读取原始加速度寄存器。 |
| `MPU6050_ReadGyroRaw()` | 读取原始陀螺仪寄存器。 |
| `MPU6050_ReadTempRaw()` | 读取原始温度寄存器。 |
| `MPU6050_ReadAll()` | 一次读取 14 字节并换算为 g、dps、摄氏度。 |
| `MPU6050_CalibrateGyro()` | 静止采样陀螺仪和加速度零偏。 |
| `MPU6050_GetGyroZero()` | 读取当前陀螺仪零偏。 |
| `MPU6050_SetGyroZero()` | 手动设置陀螺仪零偏并清滤波状态。 |
| `MPU6050_SetFilter()` | 设置低通滤波系数、陀螺死区和加速度死区。 |
| `MPU6050_SetFilterParam()` | 旧接口, 调用 `MPU6050_SetFilter(alpha, deadzone, 0)`。 |
| `MPU6050_ReadAllCalibrated()` | 读取、扣零偏、死区、滤波, 并更新 roll/pitch。 |

### 5.7 `IMU/HWT101/`

| 函数 | 作用 |
| --- | --- |
| `make_i16_le()` | 小端低高字节合成 int16。 |
| `delay_loop()` | NOP 延时。 |
| `clear_i2c_state()` | 复位 HWT101 I2C 传输状态和 FIFO。 |
| `wait_idle()` | 等待 I2C controller idle。 |
| `wait_stop()` | 等待 STOP, 检查 NACK/仲裁丢失。 |
| `wait_tx_done()` | 等待 TX_DONE, 检查异常。 |
| `read_rx_fifo()` | 从 RX FIFO 读指定长度。 |
| `enable_i2c_pullups()` | 重新配置 I2C 引脚上拉。 |
| `recover_bus()` | 通过 SCL 脉冲尝试恢复 I2C 总线。 |
| `prepare_transfer()` | 等待空闲或恢复总线, 清状态。 |
| `read_accel_detect_regs()` | 读取加速度寄存器用于识别模块类型。 |
| `looks_like_full_attitude_module()` | 根据加速度判断是否像完整姿态模块, 用于区分 HWT101 和 JY901S。 |
| `HWT101_ReadBytes()` | 带重试的寄存器连续读取。 |
| `HWT101_WriteRegister()` | 带重试的寄存器写入。 |
| `HWT101_ProbeAddress()` | 探测指定 I2C 地址是否 ACK。 |
| `HWT101_ScanFirstAddress()` | 扫描第一个 ACK 地址。当前没有有效调用。 |
| `HWT101_ReadRegister()` | 读取 16 位寄存器。 |
| `HWT101_IsConnected()` | 判断 HWT101 是否连接且不像完整姿态模块。 |
| `HWT101_Init()` | 配上拉、恢复总线并检查连接。 |
| `HWT101_ZeroYaw()` | 解锁后发送清 yaw 命令。 |
| `HWT101_ReadGyroRaw()` | 读 gy/gz 原始值。 |
| `HWT101_ReadYawRaw()` | 读 yaw 原始值。 |
| `HWT101_ReadRaw()` | 读 gy/gz/yaw/version 原始数据。 |
| `HWT101_ConvertRaw()` | 原始数据换算为 `IMU_Data_t`。 |
| `HWT101_ReadAll()` | 读取并换算 HWT101 数据。 |

### 5.8 `IMU/JY901S/`

| 函数 | 作用 |
| --- | --- |
| `make_i16_le()` | 小端低高字节合成 int16。 |
| `delay_loop()` | NOP 延时。 |
| `clear_i2c_state()` | 复位 JY901S I2C 传输状态和 FIFO。 |
| `wait_idle()` | 等待 I2C controller idle。 |
| `wait_stop()` | 等待 STOP, 检查 NACK/仲裁丢失。 |
| `wait_tx_done()` | 等待 TX_DONE, 检查异常。 |
| `read_rx_fifo()` | 从 RX FIFO 读指定长度。 |
| `enable_i2c_pullups()` | 重新配置 I2C 引脚上拉。 |
| `recover_bus()` | 通过 SCL 脉冲尝试恢复 I2C 总线。 |
| `prepare_transfer()` | 等待空闲或恢复总线, 清状态。 |
| `JY901S_ReadBytes()` | 带重试的寄存器连续读取。 |
| `JY901S_WriteRegister()` | 带重试的寄存器写入。 |
| `JY901S_ProbeAddress()` | 探测指定 I2C 地址是否 ACK。 |
| `JY901S_ScanFirstAddress()` | 扫描第一个 ACK 地址。当前没有有效调用。 |
| `JY901S_ReadRegister()` | 读取 16 位寄存器。当前只有定义和声明, 没有有效业务调用。 |
| `JY901S_IsConnected()` | 判断 JY901S 是否连接。 |
| `JY901S_Init()` | 配上拉、恢复总线并检查连接。 |
| `JY901S_ZeroYaw()` | 解锁后发送清 yaw 命令。 |
| `JY901S_ReadAccelRaw()` | 读加速度原始向量。 |
| `JY901S_ReadGyroRaw()` | 读角速度原始向量。 |
| `JY901S_ReadMagRaw()` | 读磁场原始向量。 |
| `JY901S_ReadAngleRaw()` | 读 roll/pitch/yaw 原始角度。 |
| `JY901S_ReadRaw()` | 一次读取 AX 到 TEMP 的 26 字节原始数据。 |
| `JY901S_ConvertRaw()` | 原始数据换算为 `JY901S_Data_t`。 |
| `JY901S_ReadAll()` | 读取并换算 JY901S 数据。 |

### 5.9 `wdd35d4/`

| 函数 | 作用 |
| --- | --- |
| `WDD35D4_Init()` | 重置内部标定/零点/方向/滤波状态并使能 ADC。 |
| `WDD35D4_SetCalibration()` | 设置原始 ADC 到角度的有效范围。 |
| `WDD35D4_ResetCalibration()` | 恢复默认 0 到 4095 标定范围。 |
| `WDD35D4_GetCalibration()` | 读取当前标定范围。 |
| `WDD35D4_CalibrateZero()` | 多次采样当前 ADC 平均值作为零点。当前 `main.c` 没调用, 改为固定 `990`。 |
| `WDD35D4_SetZeroRaw()` | 设置竖直零点 ADC 原始值。当前启动时设为 `990`。 |
| `WDD35D4_GetZeroRaw()` | 读取零点。 |
| `WDD35D4_SetDirection()` | 设置角度方向正反。 |
| `WDD35D4_GetDirection()` | 读取角度方向。 |
| `WDD35D4_ReadRaw()` | 触发 ADC 转换并等待 MEM0 结果。 |
| `WDD35D4_ReadFilteredRaw()` | 读取 ADC 并可选一阶低通。默认 alpha 为 0, 直通。 |
| `WDD35D4_ReadVoltage()` | 读取并换算电压。 |
| `WDD35D4_ReadAngleDeg()` | 读取并换算单端角。 |
| `WDD35D4_ReadSignedAngleDeg()` | 读取并换算相对零点有符号角。 |
| `WDD35D4_ReadData()` | 一次输出 raw、电压、单端角、有符号角。 |
| `WDD35D4_RawToVoltage()` | ADC 原始值换算电压。 |
| `WDD35D4_RawToAngleDeg()` | ADC 原始值换算单端角。 |
| `WDD35D4_RawToSignedAngleDeg()` | ADC 原始值换算相对零点有符号角。 |
| `WDD35D4_ClampRaw()` | 内部函数, 将 raw 限制在标定范围内。 |
| `WDD35D4_ReadProcessedRaw()` | 内部函数, 读取滤波 raw 并四舍五入为 uint16。 |

### 5.10 `OLED/`

| 函数 | 作用 |
| --- | --- |
| `i2c_wait_idle()` | OLED I2C 等待总线空闲。 |
| `i2c_wait_stop_or_nack()` | OLED I2C 等待 STOP/NACK。 |
| `i2c_write()` | OLED I2C 写不超过 FIFO 大小的数据。 |
| `write_command()` | 写 SSD1306 命令。 |
| `write_data()` | 写 SSD1306 数据块。 |
| `set_page_address()` | 设置 SSD1306 page 和 column。 |
| `set_pixel_i()` | 带 int 边界检查的像素写入。 |
| `probe_address()` | 探测 OLED 地址。 |
| `select_oled_address()` | 在 0x3C/0x3D 中选择可用地址。 |
| `send_init_sequence()` | 发送 SSD1306 初始化命令并清屏刷新。 |
| `OLED_Init()` | 初始化 OLED 底层驱动。 |
| `OLED_IsReady()` | 返回 OLED 是否初始化成功。 |
| `OLED_Clear()` | 清 framebuffer。 |
| `OLED_UpdatePage()` | 刷新单个 page。 |
| `OLED_Update()` | 刷新全部 page。 |
| `OLED_SetPixel()` | 设置 framebuffer 单像素。 |
| `OLED_DrawPoint()` | 点亮单像素。 |
| `OLED_ClearPoint()` | 清除单像素。 |
| `OLED_DrawLine()` | Bresenham 画线。当前没有业务调用。 |
| `OLED_DrawCircle()` | 中点圆算法画圆。当前没有业务调用。 |
| `OLED_GetBuffer()` | 返回 framebuffer 指针。当前没有业务调用。 |
| `printable_char_index()` | 显示层内部函数, 非 ASCII 可显示字符转空格。 |
| `draw_char()` | 显示层内部函数, 将 6x8 字模写入 framebuffer。 |
| `Display_Init()` | 初始化 OLED。 |
| `Display_Clear()` | 清屏并刷新。 |
| `Display_Update()` | 全量刷新显示。当前没有业务调用。 |
| `Display_ShowString()` | 在指定行列显示字符串, 局部刷新后清 framebuffer。 |
| `Display_ShowSpeed()` | 格式化速度并显示。当前没有业务调用。 |
| `Display_ShowValue()` | 格式化标签和值并显示。当前没有业务调用。 |

### 5.11 `Emm/`

| 函数 | 作用 |
| --- | --- |
| `Emm_SendData()` | 按 Emm 协议通过 `Emm_INST` UART 发送地址、功能码、数据和校验。 |
| `Emm_Loc_Control()` | 把角度/速度/加速度/模式打包为 0xFD 位置控制指令。当前没有业务调用。 |
| `Emm_Stop()` | 发送 0xFE 停止指令。当前没有业务调用。 |
| `Emm_Init()` | 发送 0xF3 使能指令。当前启动时调用地址 1 和 2。 |

### 5.12 `ultrasonic/`

`ultrasonic.h` 声明:

- `Ultrasonic_Init()`
- `Ultrasonic_GetDistance()`

但 `ultrasonic.c` 当前全部有效实现都被注释, 编译时只生成空对象。`main.c` 没有包含 `ultrasonic.h`, `Stageultrasonic` case 内调用也全是注释, 当前没有链接风险。若以后恢复超声波, 需要先处理 PB17 与 WDD35D4 ADC 的引脚冲突。

## 6. 循环索引

说明: 以下只列有效编译代码中的循环。注释块里的旧循环不参与编译, 不列入有效控制流。

### 6.1 `main.c`

| 位置 | 循环 | 作用 |
| --- | --- | --- |
| `main.c:230` | `while (getTimeMs(getNowMs(), startTime) < 1000)` | 开机 1 秒显示并允许按键选择 `TextIndex`。 |
| `main.c:239` | `while (getTimeMs(getNowMs(), startTime) < 1000)` | `TextIndex == 3` 时, 1 秒显示并允许选择 `Goal`。 |
| `main.c:253` | `while (1)` | 主循环, 执行 UART TX 轮询、倒立摆控制和可选阶段状态机。 |
| `main.c:766` | `while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST))` | UART0 RX 中断中清空 RX FIFO。 |
| `main.c:911` | `for (int i = 0; i < 3; i++)` | 蜂鸣器响三声。 |

### 6.2 `BasicMicroLib/`

| 位置 | 循环 | 作用 |
| --- | --- | --- |
| `delay.c:20` | `while (1)` | 轮询 SysTick 当前值直到达到目标 tick 数。 |
| `getTime.c:25` | `do ... while (overflow1 != overflow2)` | 双读溢出计数, 保证扩展 tick 快照一致。 |
| `usart.c:27` | `while (DL_UART_isBusy(...))` | 非 UART0 阻塞发送前等待 UART 空闲, 带超时。 |
| `usart.c:149` | `while (*str != '\0')` | 异步写字符串时逐字节入队。 |
| `usart.c:211` | `while (1)` | `_sys_exit()` 永久等待中断。 |
| `usart.c:242` | `for (i = 0; i < count; i++)` | `write()` 将缓冲逐字节输出到 `stdout`。 |

### 6.3 `Emm/`

| 位置 | 循环 | 作用 |
| --- | --- | --- |
| `Emm.c:10` | `for (int i = 0; i < dataSize; i++)` | 发送 Emm 指令数据区。 |
| `Emm.c:41` | `for (i = 0; i < 4; i++)` | 将 32 位角度按大端顺序写入指令。 |

### 6.4 `GrayScale/`

| 位置 | 循环 | 作用 |
| --- | --- | --- |
| `grayscale_sensor.h:31` | `do ... while (0)` | GPIO 写宏的多语句安全封装。 |
| `grayscale_sensor.c:72` | `for i = 0..7` | 读取全部 8 路灰度。 |
| `grayscale_sensor.c:80` | `for i = 2..5` | 读取中间 4 路灰度。 |
| `grayscale_sensor.c:88` | `for i = 0..1` | 读取左侧两路灰度。 |
| `grayscale_sensor.c:91` | `for i = 6..7` | 读取右侧两路灰度。 |
| `Grayscale_Scan.c:213` | `for i = 0..7` | 统计在线灰度传感器数量。 |

### 6.5 `Motor/`

当前 `Motor/` 的有效函数内部没有显式 `for`/`while` 循环。速度闭环每次调用只做一次增量 PID 计算。

### 6.6 `wdd35d4/`

| 位置 | 循环 | 作用 |
| --- | --- | --- |
| `wd35d4.c:67` | `for i = 0..sample_count-1` | 零点校准时多次采样取平均。 |
| `wd35d4.c:112` | `while ADC MEM0 未完成且未超时` | 等待 ADC 转换完成, 防止异常时卡死。 |

### 6.7 `IMU/MPU6050/`

| 位置 | 循环 | 作用 |
| --- | --- | --- |
| `mpu6050.c:59` | `while (loops-- > 0)` | NOP 延时。 |
| `mpu6050.c:211` | `while BUSY_BUS` | 等待 I2C 总线空闲。 |
| `mpu6050.c:233` | `do ... while (status == 0)` | 等待 STOP 或 NACK。 |
| `mpu6050.c:308` | `for i = 0..length-1` | 按长度读取 RX FIFO。 |
| `mpu6050.c:310` | `while RXFIFO empty` | 等待当前字节进入 RX FIFO, 带超时。 |
| `mpu6050.c:585` | `for i = 0..samples-1` | 陀螺仪零偏采样。 |
| `mpu6050.c:597` | `for i = 0..samples-1` | 加速度零偏采样。 |

### 6.8 `IMU/HWT101/`

| 位置 | 循环 | 作用 |
| --- | --- | --- |
| `hwt101.c:34` | `while (loops-- > 0)` | NOP 延时。 |
| `hwt101.c:51` | `while controller 非 idle` | 等待 I2C 空闲。 |
| `hwt101.c:80` | `do ... while STOP 未到` | 等待 STOP, 检查 NACK/仲裁丢失。 |
| `hwt101.c:106` | `do ... while TX_DONE 未到` | 等待发送完成。 |
| `hwt101.c:115` | `for i = 0..length-1` | 逐字节读取 RX FIFO。 |
| `hwt101.c:117` | `while RXFIFO empty` | 等待 RX FIFO 数据, 带异常检测。 |
| `hwt101.c:181` | `for i = 0..RECOVERY_PULSES-1` | I2C 总线恢复时发送 SCL 脉冲。 |
| `hwt101.c:256` | `for retry = 0..RETRY_COUNT-1` | 读取寄存器重试。 |
| `hwt101.c:306` | `for retry = 0..RETRY_COUNT-1` | 写寄存器重试。 |
| `hwt101.c:340` | `for addr = 0x08..0x77` | 扫描 I2C 地址。 |

### 6.9 `IMU/JY901S/`

| 位置 | 循环 | 作用 |
| --- | --- | --- |
| `jy901s.c:35` | `while (loops-- > 0)` | NOP 延时。 |
| `jy901s.c:52` | `while controller 非 idle` | 等待 I2C 空闲。 |
| `jy901s.c:81` | `do ... while STOP 未到` | 等待 STOP, 检查 NACK/仲裁丢失。 |
| `jy901s.c:107` | `do ... while TX_DONE 未到` | 等待发送完成。 |
| `jy901s.c:116` | `for i = 0..length-1` | 逐字节读取 RX FIFO。 |
| `jy901s.c:118` | `while RXFIFO empty` | 等待 RX FIFO 数据, 带异常检测。 |
| `jy901s.c:182` | `for i = 0..RECOVERY_PULSES-1` | I2C 总线恢复时发送 SCL 脉冲。 |
| `jy901s.c:222` | `for retry = 0..RETRY_COUNT-1` | 读取寄存器重试。 |
| `jy901s.c:272` | `for retry = 0..RETRY_COUNT-1` | 写寄存器重试。 |
| `jy901s.c:306` | `for addr = 0x08..0x77` | 扫描 I2C 地址。 |

### 6.10 `OLED/`

| 位置 | 循环 | 作用 |
| --- | --- | --- |
| `oled.c:26` | `while BUSY_BUS` | 等待 OLED I2C 总线空闲。 |
| `oled.c:49` | `do ... while (status == 0)` | 等待 STOP 或 NACK。 |
| `oled.c:77` | `while (sent < length)` | 将待发数据逐字节填入 TX FIFO。 |
| `oled.c:141` | `for each address` | 探测 0x3C 和 0x3D OLED 地址。 |
| `oled.c:172` | `for each init command` | 发送 SSD1306 初始化命令。 |
| `oled.c:186` | `for volatile i < 10000` | OLED 上电后短延时。 |
| `oled.c:216` | `while (column < OLED_WIDTH)` | 分块刷新一个 page。 |
| `oled.c:236` | `for page = 0..PAGE_COUNT-1` | 刷新全部 page。 |
| `oled.c:282` | `while (true)` | Bresenham 画线, 到终点后 break。 |
| `oled.c:308` | `while (x <= y)` | 中点圆算法八分对称画圆。 |
| `display.c:129` | `for col = 0..FONT_WIDTH-1` | 绘制字符的列。 |
| `display.c:131` | `for bit = 0..FONT_HEIGHT-1` | 绘制字符列内的每个像素。 |
| `display.c:179` | `while (*str != '\0')` | 逐字符显示字符串, 必要时换行。 |
| `display.c:194` | `for page = firstPage..lastPage` | 刷新字符串覆盖的 page。 |

## 7. 无用代码和可删除性判断

### 7.1 可以安全清理

这些清理不改变当前固件行为:

- `Debug/` 目录下的 `.o`、`.d`、`.out`、`.map`、`*_linkInfo.xml`、`.mk`、clangd cache 等构建产物。`.gitignore` 已忽略 `Debug/` 和这些文件, CCS 可重新生成。
- 根目录 `et --hard e99178a`。内容是带 ANSI 颜色码的 `git log` 输出, 文件名像误输入 `git reset --hard e99178a` 时生成的残留, 不参与构建。
- `main.c` 里已经注释的大块旧逻辑: 基础循迹、超声波测距、MaixCAM 解析、OLED 速度显示、旧 `Display_WheelSpeeds()` 实现等。它们不参与编译。
- `ultrasonic/ultrasonic.c` 当前有效代码为空, 且没有有效调用者。若确认不再使用超声波, 可以删除 `ultrasonic/` 并从工程重新生成 makefile。
- `main.c` 中 `Display_WheelSpeeds()` 的前置声明和底部注释实现可以删除。当前没有有效定义和有效调用。
- `MOTOR_STEP_TEST_ENABLE` 为 0 时的 `MotorClosedLoopStepTest()`/`MotorStepTest_IsReached()` 声明块可以删除。当前没有定义, 打开宏也无法链接。

### 7.2 当前未用, 但不建议直接删

这些接口当前业务路径没有直接调用, 但属于模块公共 API、调试接口或未来功能入口。删除前应先确认比赛功能和调试方式:

- `NewMotorSpeedCtrl_UpdateByEncoderDelta()`、`NewMotorSpeedCtrl_ResetAndStop()`、`NewMotorSpeedCtrl_GetMeasuredWheelMmps()`、`NewMotorSpeedCtrl_GetOutputPwmTicks()`。当前倒立摆直控绕过速度闭环, 但阶段循迹代码仍按速度闭环接口组织。
- `NewMotor_LeftRightToLinearSpeedMmps()`、`NewMotor_LeftRightToYawRateRadps()`。当前未用, 但属于底盘运动学辅助。
- `Grayscale_Sensor_Read_Main()`、`Grayscale_Sensor_Read_Other()`、`Grayscale_Sensor_Read_Single()`、`Grayscale_OnlineNum()`。当前未用, 但调试或优化灰度算法时有价值。
- `IMU_GetReadyMask()`、`IMU_IsDeviceReady()`、`HWT101_ScanFirstAddress()`、`JY901S_ScanFirstAddress()`、各 IMU raw read 函数。当前主业务只用 `IMU_Init()`、`IMU_ReadAll()`、`IMU_ZeroYaw()`, 但这些函数适合排障。
- `MPU6050_ReadAccelRaw()`、`MPU6050_ReadGyroRaw()`、`MPU6050_ReadTempRaw()`、`MPU6050_InitWithConfig()`、`MPU6050_SetFilterParam()` 等配置/原始读取接口。当前由 `IMU_Init()` 间接使用一部分 MPU6050 接口, 原始接口适合调试。
- OLED 图形接口 `OLED_DrawLine()`、`OLED_DrawCircle()`、`OLED_GetBuffer()`、`Display_ShowSpeed()`、`Display_ShowValue()`。当前主程序只显示字符串, 但这些是显示模块公共能力。
- WDD35D4 的标定、读取电压、读取角度等 API。当前只初始化并设置零点, 没有进入控制环。若后续不用 WDD35D4, 可以整体移除；如果要做角位移倒立摆, 这些必须保留。
- `Emm_Loc_Control()` 和 `Emm_Stop()`。当前只 `Emm_Init()` 使能舵机, 位置控制和停止接口未用, 但属于云台协议核心功能。
- `Stage.h` 中的阶段枚举和命令表。当前默认平衡模式不跑阶段状态机；若只做倒立摆, 可以移除阶段系统和灰度路线逻辑。若还要循迹比赛, 不能删。

### 7.3 建议修正而不是删除

- `newmotor_driver.h` 中 `NewMotor_ApplyDeadzoneTicks()` 只有声明没有定义。应删除这个声明, 或补一个实现, 或改成声明现有的 `NewMotor_ApplyLetfDeadzoneTicks()`/`NewMotor_ApplyRightDeadzoneTicks()`。
- `NewMotor_ApplyLetfDeadzoneTicks()` 拼写错误。若没有外部依赖, 建议重命名为 `NewMotor_ApplyLeftDeadzoneTicks()`。
- `StageFinsih` 拼写错误。若没有外部依赖, 建议统一改为 `StageFinish`。
- `StageStandUp` 只有枚举没有 `main.c` case。要么补实现, 要么删枚举。
- `command0[] = {}` 为空。阶段模式下默认 `TextIndex=0` 会导致 `command[StageIndex]` 越界。要么给 `command0` 合法内容, 要么默认 `TextIndex` 指向有效命令, 要么阶段模式前检查长度。
- `lastStageTime = getNowMs() + 5000` 配合 `getTimeMs(now, last)` 会因为无符号减法在 `lastStageTime` 位于未来时得到很大的差值, 阶段模式下不会实现 5 秒延时。若要延时, 应记录 `stageEnableTime` 或先判断绝对时间到达。
- `Track_PID()` 当前计算了 `error_last` 但没有更新它, D 项实际不按预期工作。若继续用灰度 PID, 应在返回前更新 `error_last = error`。

## 8. 当前不能删除的核心链路

在保持当前倒立摆 IMU 直控运行的前提下, 以下内容不能直接删除:

- `main.c` 中初始化、编码器 ISR、UART0 ISR、平衡主循环、`Balance_*` 函数、`buzzer_beep()`。
- `BasicMicroLib/delay.*`, `getTime.*`, `usart.*`。
- `Motor/newmotor_driver.*` 中 PWM 输出、停车、编码器换算。
- `IMU/imu.*`, `IMU/imu_data.h`, 以及当前可能接入的 HWT101/JY901S/MPU6050 子驱动。
- `OLED/display.*` 和 `OLED/oled.*`, 因为启动过程依赖显示初始化和字符串提示。
- `Emm/Emm.*`, 因为启动过程会调用 `Emm_Init(1)` 和 `Emm_Init(2)`。
- `empty.syscfg` 和 SysConfig 生成的 `ti_msp_dl_config.*`。生成物在 `Debug/` 下, 不手动维护, 但编译需要。

## 9. 维护注意事项

- 当前 `main.c` 处于未提交修改状态, 修改前先看 `git status` 和 diff。
- `Debug/` 是生成目录, 不要在里面手改业务逻辑。
- `BALANCE_DIRECT_CONTROL_ENABLE=1` 时, 阶段循迹代码是运行时不可达路径；调试循迹前先改这个宏并处理 `command0` 越界风险。
- 当前 UART0 是 921600, 旧文档中“蓝牙 115200”的说法已过时。Emm UART 是 115200, MaixCAM UART 是 9600。
- PB17 当前用于 WDD35D4 ADC, 旧超声波 Echo 也写 PB17。两者不能同时使用。
- `printf` 已走 UART0 DMA 环形缓冲, 高频日志仍可能填满缓冲, 可用 `USART_GetDroppedTxBytes()` 排查丢字节。
- 编码器计数在 ISR 里更新, 主循环读取并清零时必须保留临界区。
- 如果要把倒立摆控制改回速度闭环, 需要恢复周期性调用 `NewMotorSpeedCtrl_UpdateByEncoderDelta()` 或重新设计直立环输出到速度闭环的接口。
