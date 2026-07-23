#include "Arm/jibot_servo.h"
#include "Arm/arm_control.h"
#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/getTime.h"
#include "BasicMicroLib/usart.h"
#include "Communication/maixcam_serial.h"
#include "Drivers/board_isr.h"
#include "Drivers/button_select.h"
#include "Drivers/buzzer.h"
#include "Emm/Emm.h"
#include "GrayScale/grayscale_sensor.h"
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

typedef enum AppMenuAction {
	AppMenuActionRoute = 0,
	AppMenuActionArmTeach,
} AppMenuAction;

typedef struct AppRouteOption {
	AppMenuAction action;
	uint8_t commandIndex;
	int goal;
} AppRouteOption;

/*
 * 菜单显示的地图编号从 1 开始；goal 仍保持阶段机原有的 0~3 语义。
 * 地图 4 的四个目标点展开为独立候选项，使两按键即可完成选择和启动。
 */
static const AppRouteOption appRouteOptions[] = {
	{AppMenuActionRoute, 0U, 0},
	{AppMenuActionRoute, 1U, 0},
	{AppMenuActionRoute, 2U, 0},
	{AppMenuActionRoute, 3U, 0},
	{AppMenuActionRoute, 3U, 1},
	{AppMenuActionRoute, 3U, 2},
	{AppMenuActionRoute, 3U, 3},
	{AppMenuActionRoute, 4U, 0},
	{AppMenuActionArmTeach, 0U, 0},
};

#define APP_ROUTE_OPTION_COUNT \
	((uint8_t)(sizeof(appRouteOptions) / sizeof(appRouteOptions[0])))

static void App_ShowRouteMenu(uint8_t optionIndex)
{
	const AppRouteOption *option = &appRouteOptions[optionIndex];
	char line[21];

	Display_Clear();
	snprintf(line, sizeof(line), "MAP SELECT %u/%u",
			 (unsigned int)(optionIndex + 1U),
			 (unsigned int)APP_ROUTE_OPTION_COUNT);
	Display_ShowString(0, 0, line);

	if (option->action == AppMenuActionArmTeach) {
		snprintf(line, sizeof(line), "ARM TEACH TEST");
	} else if (option->commandIndex == 3U) {
		snprintf(line, sizeof(line), "MAP 4 TARGET %d", option->goal + 1);
	} else {
		snprintf(line, sizeof(line), "MAP %u",
			 (unsigned int)(option->commandIndex + 1U));
	}
	Display_ShowString(2, 0, line);
	Display_ShowString(5, 0, "B1 NEXT");
	Display_ShowString(6, 0,
				   (option->action == AppMenuActionArmTeach) ?
					   "B2 ENTER" :
					   "B2 START");
}

static const AppRouteOption *App_SelectRoute(void)
{
	uint8_t optionIndex = 0U;

	ButtonSelect_ResetEvents();
	App_ShowRouteMenu(optionIndex);

	while (true) {
		ButtonSelect_Event event = ButtonSelect_TakeEvent();

		if (event == ButtonSelectEventNext) {
			optionIndex++;
			if (optionIndex >= APP_ROUTE_OPTION_COUNT) {
				optionIndex = 0U;
			}
			App_ShowRouteMenu(optionIndex);
		} else if (event == ButtonSelectEventStart) {
			Display_Clear();
			Display_ShowString(2, 0, "STARTING...");
			return &appRouteOptions[optionIndex];
		}

		/* 启动菜单停留期间继续维护通信接收，不执行路线或电机控制。 */
		USART_PollTx();
		MaixCamSerial_Poll();
	}
}

int main(void) {
	const AppRouteOption *routeOption;
	const StageCommand *command;

	//--------------------------------------
	//                初始化
	//--------------------------------------
	SYSCFG_DL_init(); // 由SysConfig自动生成的初始化函数
	//---------------中断使能----------------

	MaixCamSerial_Init();
	BoardIrq_Enable();
	USART_Init(); // 使能UART中断（接收依赖此步骤）
	setvbuf(stdout, NULL, _IONBF, 0);

	Display_Init(); // 初始化显示屏
	(void)Grayscale_Init();
	Display_ShowString(0, 0,
				   (Grayscale_GetActiveDriver() == GrayscaleDriver12) ?
					   "Gray12 Ready" :
					   "Gray8 Ready");

	TimeBase_Init(); // 初始化计时器

	// /* UART2 已分配给 Jibot；首次上电仅小幅测试夹爪。 */
	// (void)JibotServo_SetAngle(3, -90.0F, 1000U);
	// (void)JibotServo_SetAngle(2, -90.0F, 1000U);

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

	/* 选择期间确保车辆不会因初始化残留的目标值而移动。 */
	MotorRuntime_SetTargetWheelMmps(0, 0);
	MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);

	Display_ShowString(0, 0, "Car Ready"); // 可选：开机显示欢迎信息
	delay_ms(2000);
	/* ARM TEACH 是菜单内的独立测试项；退出后回到地图选择。 */
	while (true) {
		routeOption = App_SelectRoute();
		if (routeOption->action != AppMenuActionArmTeach) {
			break;
		}
		ArmControl_RunTeachTest();
	}
	command = commandList[routeOption->commandIndex];

	/* 选择等待时间不应影响出发时的航向零点。 */
	IMU_ZeroYaw();
	startTime = getNowMs();
	Buzzer_Beep();
	StageRunner_Init(command, routeOption->goal, &stageConfig);
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
