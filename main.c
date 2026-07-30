#include "Arm/jibot_servo.h"
#include "Arm/arm_control.h"
#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/getTime.h"
#include "BasicMicroLib/usart.h"
#include "Communication/bluetooth_serial.h"
#include "Communication/dl_ln33.h"
#include "Communication/maixcam_serial.h"
#include "Drivers/board_isr.h"
#include "Drivers/button_select.h"
#include "Drivers/buzzer.h"
#include "GrayScale/grayscale_sensor.h"
#include "IMU/imu.h"
#include "InfraredSpeed/infrared_speed.h"
#include "Motor/motor_runtime.h"
#include "Motor/motor_step_test.h"
#include "OLED/display.h"
#include "Stage/Stage.h"
#include "dl_ln33_stage_machine/car_sync.h"
#include "dl_ln33_stage_machine/car_sync_setup.h"
#include "Stage/stage_runner.h"
#include "wdd35d4/wd35d4.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>

static const StageRunner_Config stageConfig = {
	.base_speed_mmps = 400,
	.round_speed_mmps = 150,
	.sync_role = CarSyncRoleSolo,
	.sync_peer_address = 0U,
	.sync_run_id = CARSYNC_CROSS_RUN_ID,
};

typedef enum AppMenuAction {
	AppMenuActionRoute = 0,
	AppMenuActionArmTeach,
	AppMenuActionMotorPidTest,
} AppMenuAction;

typedef struct AppRouteOption {
	AppMenuAction action;
	uint8_t commandIndex;
	int goal;
	CarSyncRole syncRole;
} AppRouteOption;

/*
 * 菜单显示的地图编号从 1 开始；goal 仍保持阶段机原有的 0~3 语义。
 * 地图 4 的四个目标点展开为独立候选项，使两按键即可完成选择和启动。
 */
static const AppRouteOption appRouteOptions[] = {
	{AppMenuActionRoute, 0U, 0, CarSyncRoleSolo},
	{AppMenuActionRoute, 1U, 0, CarSyncRoleSolo},
	{AppMenuActionRoute, 2U, 0, CarSyncRoleSolo},
	{AppMenuActionRoute, 3U, 0, CarSyncRoleSolo},
	{AppMenuActionRoute, 3U, 1, CarSyncRoleSolo},
	{AppMenuActionRoute, 3U, 2, CarSyncRoleSolo},
	{AppMenuActionRoute, 3U, 3, CarSyncRoleSolo},
	{AppMenuActionRoute, 4U, 0, CarSyncRoleSolo},
	{AppMenuActionRoute, 5U, 0, CarSyncRoleLeader},
	{AppMenuActionRoute, 5U, 0, CarSyncRoleFollower},
	{AppMenuActionRoute, 6U, 0, CarSyncRoleSolo},
	{AppMenuActionArmTeach, 0U, 0, CarSyncRoleSolo},
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
	} else if (option->syncRole == CarSyncRoleLeader) {
		snprintf(line, sizeof(line), "CROSS LEADER");
	} else if (option->syncRole == CarSyncRoleFollower) {
		snprintf(line, sizeof(line), "CROSS FOLLOWER");
	} else if (option->commandIndex == 6U) {
		snprintf(line, sizeof(line), "ARM GRAB TEST");
	} else if (option->commandIndex == 3U) {
		snprintf(line, sizeof(line), "MAP 4 TARGET %d", option->goal + 1);
	} else {
		snprintf(line, sizeof(line), "MAP %u",
			 (unsigned int)(option->commandIndex + 1U));
	}
	Display_ShowString(2, 0, line);
	Display_ShowString(5, 0, "B1 NEXT");
	Display_ShowString(6, 0,
				   (option->action == AppMenuActionRoute) ?
					   "B2 START" :
					   "B2 ENTER");
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
		DLLN33_Poll();
		MaixCamSerial_Poll();
	}
}

int main(void) {
	const AppRouteOption *routeOption;
	const StageCommand *command;
	StageRunner_Config runConfig;

	//--------------------------------------
	//                初始化
	//--------------------------------------
	SYSCFG_DL_init(); // 由SysConfig自动生成的初始化函数
	/* WDD35D4使用PB17/ADC1 CH4，必须在SysConfig完成外设配置后初始化。 */
	WDD35D4_Init();
	WDD35D4_CalibrateZero(10);
	TimeBase_Init(); // 红外测速中断需要微秒级时间基准
	DLLN33_Init();
	InfraredSpeed_Init();
	//---------------中断使能----------------

	MaixCamSerial_Init();
	BoardIrq_Enable();
	USART_Init(); // 使能UART中断（接收依赖此步骤）
	BluetoothSerial_Init();
	setvbuf(stdout, NULL, _IONBF, 0);

	Display_Init(); // 初始化显示屏
	(void)Grayscale_Init();
	Display_ShowString(0, 0,
				   (Grayscale_GetActiveDriver() == GrayscaleDriver12) ?
					   "Gray12 Ready" :
					   "Gray8 Ready");

	// /* UART0 已分配给 Jibot；首次上电仅小幅测试夹爪。 */
	// (void)JibotServo_SetAngle(3, -90.0F, 1000U);
	// (void)JibotServo_SetAngle(2, -90.0F, 1000U);

	MotorRuntime_Init();
	printf("OK");

	// IMU初始化
	{
		int temp = 0;
		temp = IMU_Init();
		if (temp) {
			if (temp == 1) {
				Display_ShowString(0, 0, "HWT101 Ready");
			} else if (temp == 2) {
				Display_ShowString(0, 0, "JY901S Ready");
				JY901S_ZeroAxAy();
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
	/* 所有独立测试项退出后均回到地图选择。 */
	while (true) {
		routeOption = App_SelectRoute();
		if (routeOption->action == AppMenuActionRoute &&
			CarSync_SetupNetwork(routeOption->syncRole)) {
			break;
		}
		if (routeOption->action == AppMenuActionArmTeach) {
			ArmControl_RunTeachTest();
		} else if (routeOption->action == AppMenuActionMotorPidTest) {
			MotorStepTest_Run(stageConfig.base_speed_mmps);
		}
	}
	command = commandList[routeOption->commandIndex];
	runConfig = stageConfig;
	runConfig.sync_role = routeOption->syncRole;
	runConfig.sync_peer_address = CarSync_GetPeerAddress(routeOption->syncRole);
	runConfig.sync_run_id = CARSYNC_CROSS_RUN_ID;

	/* 选择等待时间不应影响出发时的航向零点。 */
	IMU_ZeroYaw();
	startTime = getNowMs();
	Buzzer_Beep();
	StageRunner_Init(command, routeOption->goal, &runConfig);
	if (runConfig.sync_role == CarSyncRoleFollower) {
		MotorRuntime_SetTargetWheelMmps(0, 0);
		MotorRuntime_Stop(NEWMOTOR_STOP_BRAKE);
	}

	while (1) {
		// 更新当前时间
		nowTime = getNowMs();
		USART_PollTx();
		DLLN33_Poll();
		MaixCamSerial_Poll();
		// 每10ms获取电机运行圈数
		MotorRuntime_Update(nowTime, getNowUs());

		if (!StageRunner_Update(nowTime)) {
			break;
		}
	}
}
