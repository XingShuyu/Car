#ifndef MOTOR_STEP_TEST_H
#define MOTOR_STEP_TEST_H

/**
 * @brief 运行交互式双轮 PID 阶跃测试。
 *
 * B1 退出并刹停；B2 复位后重新加速到 target_mmps。
 */
void MotorStepTest_Run(int target_mmps);

#endif
