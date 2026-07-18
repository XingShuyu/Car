#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/getTime.h"
#include "BasicMicroLib/usart.h"
#include "Communication/bluetooth_serial.h"
#include "Communication/dl_ln33.h"
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

/*
 * 首次配置某一块 DL-LN33 时，临时设为 1，并为每个节点填写唯一地址。
 * 配置成功后模块会保存参数并重启；日常运行请保持为 0，避免每次上电重启组网模块。
 */
#define DL_LN33_AUTO_CONFIG_ON_BOOT 0U
#define DL_LN33_NODE_ADDRESS         0x0002U
#define DL_LN33_NETWORK_ID           0x1988U
#define DL_LN33_CHANNEL              0x0FU

/*
 * 联调角色：本机地址为 0x0002。收到 0x0001 从 A0 端口发来的 "PING" 后，
 * 立即从 A0 端口回复 "PONG"。该测试会取走 DL-LN33 接收队列中的帧，
 * 因此接入正式无线业务前请改为 0。
 */
#define DL_LN33_PINGPONG_TEST_ENABLE 1U
#define DL_LN33_PINGPONG_PORT        0xA0U
#define DL_LN33_PINGPONG_PEER        0x0001U

#if DL_LN33_AUTO_CONFIG_ON_BOOT
static const DLLN33_NetworkConfig dlLn33NetworkConfig = {
	.address = DL_LN33_NODE_ADDRESS,
	.network_id = DL_LN33_NETWORK_ID,
	.channel = DL_LN33_CHANNEL,
};
#endif

static const StageRunner_Config stageConfig = {
	.base_speed_mmps = 300,
	.round_speed_mmps = 150,
};

#if DL_LN33_PINGPONG_TEST_ENABLE
static void App_DLN33PingPongPoll(void)
{
	DLLN33_Frame frame;
	static const uint8_t pong[4U] = {'P', 'O', 'N', 'G'};

	while (DLLN33_TryReceive(&frame)) {
		if ((frame.remote_address != DL_LN33_PINGPONG_PEER) ||
			(frame.destination_port != DL_LN33_PINGPONG_PORT) ||
			(frame.payload_length != 4U) ||
			(frame.payload[0] != 'P') || (frame.payload[1] != 'I') ||
			(frame.payload[2] != 'N') || (frame.payload[3] != 'G')) {
			continue;
		}

		/* 回复到发送方的源端口；本测试中 0x0001 也使用 A0 端口。 */
		(void)DLLN33_Send(frame.remote_address, DL_LN33_PINGPONG_PORT,
						frame.source_port, pong, sizeof(pong));
	}
}
#endif

static uint8_t App_SelectIndex(uint32_t wait_ms, const char *label) {
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

int main(void) {
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
	DLLN33_Init();
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

#if DL_LN33_AUTO_CONFIG_ON_BOOT
	/* 异步写入地址、网络 ID、信道和默认 115200 波特率，完成后模块自动重启。 */
	(void)DLLN33_BeginNetworkSetup(&dlLn33NetworkConfig);
#endif

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
		DLLN33_Poll();
#if DL_LN33_PINGPONG_TEST_ENABLE
		App_DLN33PingPongPoll();
#endif
		// 每10ms获取电机运行圈数
		MotorRuntime_Update(nowTime, getNowUs());

		if (!StageRunner_Update(nowTime)) {
			break;
		}
	}
}
