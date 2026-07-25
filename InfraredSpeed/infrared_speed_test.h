/** @file infrared_speed_test.h @brief OLED 交互式测速测试程序。 */

#ifndef INFRARED_SPEED_TEST_H
#define INFRARED_SPEED_TEST_H

/**
 * 在 OLED 上运行测速测试：B1 重新布防，B2 返回启动菜单。
 * 小球依次经过任意两个不同测速门后显示速度（m/s）。
 */
void InfraredSpeedTest_Run(void);

#endif /* INFRARED_SPEED_TEST_H */
