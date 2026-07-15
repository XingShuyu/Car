#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/getTime.h"
#include "BasicMicroLib/usart.h"
#include "Communication/bluetooth_serial.h"
#include "Communication/maixcam_serial.h"
#include "Drivers/board_isr.h"
#include "Drivers/button_select.h"
#include "Drivers/buzzer.h"
#include "Emm/Emm.h"
#include "IMU/imu.h"
#include "Motor/motor_runtime.h"
#include "Motor/motor_step_test.h"
#include "OLED/display.h"
#include "Stage/Stage.h"
#include "Stage/stage_runner.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>

static const StageRunner_Config stageConfig = {
	.base_speed_mmps = 300,
	.round_speed_mmps = 150,
};

static uint8_t App_SelectIndex(uint32_t wait_ms, const char *label)
{
	ButtonSelect_Reset();
	startTime = getNowMs();
	while (getTimeMs(getNowMs(), startTime) < wait_ms) {
		char str[12];
		uint8_t index = ButtonSelect_GetIndex();
		snprintf(str, sizeof(str), "%s: %d", label, index);
		Display_ShowString(0, 0, str);
	}

	return ButtonSelect_GetIndex();
}

int main(void)
{
	uint8_t textIndex;
	int goal = 0;
	const StageCommand *command;

	//--------------------------------------
	//                初始化
	//--------------------------------------
	SYSCFG_DL_init(); // 由SysConfig自动生成的初始化函数
	//---------------中断使能----------------

	MaixCamSerial_Init();
	BluetoothSerial_Init();
	BoardIrq_Enable();
	USART_Init(); // 使能UART中断（接收依赖此步骤）
	setvbuf(stdout, NULL, _IONBF, 0);

	Display_Init(); // 初始化显示屏

	TimeBase_Init(); // 初始化计时器

	// 使能云台
	Emm_Init(1);
	delay_ms(10);
	Emm_Init(2);
	delay_ms(10);

	MotorRuntime_Init();
	printf("OK");

#if MOTOR_STEP_TEST_ENABLE
	MotorStepTest_Run();
#endif

	// IMU初始化
	{
		int temp = 0;
		temp = IMU_Init();
		if (temp) {
			if (temp == 1) {
				Display_ShowString(0, 0, "HWT101 Ready");
			} else if (temp == 2) {
				Display_ShowString(0, 0, "JY901S Ready");
			} else if (temp == 3) {
				Display_ShowString(0, 0, "MPU6050 Ready");
			}
		} else {
			Display_ShowString(0, 0, "IMU Faid");
		}
		delay_ms(1000);
	}

	// 获取启动时间tick
	Display_ShowString(0, 0, "Car Ready"); // 可选：开机显示欢迎信息
	delay_ms(2000);
	Display_Clear();
	IMU_ZeroYaw();

	textIndex = App_SelectIndex(1000, "set");
	command = commandList[textIndex];
	if (textIndex == 3) {
		goal = App_SelectIndex(3000, "goal");
	}

	startTime = getNowMs();
	Buzzer_Beep();
	StageRunner_Init(command, goal, &stageConfig);
	MotorRuntime_SetTargetWheelMmps(stageConfig.base_speed_mmps,
								   stageConfig.base_speed_mmps);

	while (1) {
		// 更新当前时间
		nowTime = getNowMs();
		USART_PollTx();
		MaixCamSerial_Poll();
		// 每10ms获取电机运行圈数
		MotorRuntime_Update(nowTime, getNowUs());

		if (!StageRunner_Update(nowTime)) {
			break;
		}
	}
}
