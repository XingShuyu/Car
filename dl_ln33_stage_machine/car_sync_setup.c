#include "dl_ln33_stage_machine/car_sync_setup.h"

#include <stdio.h>

#include "BasicMicroLib/delay.h"
#include "BasicMicroLib/usart.h"
#include "Communication/dl_ln33.h"
#include "Communication/maixcam_serial.h"
#include "OLED/display.h"

bool CarSync_SetupNetwork(CarSyncRole role)
{
	DLLN33_NetworkConfig networkConfig;
	DLLN33_NetworkSetupState setupState;
	char line[21];

	if (role == CarSyncRoleSolo) {
		return true;
	}

	networkConfig.address = CarSync_GetLocalAddress(role);
	networkConfig.network_id = CARSYNC_NETWORK_ID;
	networkConfig.channel = CARSYNC_CHANNEL;

	Display_Clear();
	Display_ShowString(0, 0, "ZB SETUP");
	if (!DLLN33_BeginNetworkSetup(&networkConfig)) {
		Display_ShowString(1, 0, "ZB BEGIN FAIL");
		delay_ms(1200);
		return false;
	}

	while (true) {
		USART_PollTx();
		DLLN33_Poll();
		MaixCamSerial_Poll();
		setupState = DLLN33_GetNetworkSetupState();
		if (setupState == DLLN33_NETWORK_SETUP_COMPLETE) {
			Display_ShowString(1, 0, "ZB OK");
			delay_ms(500);
			return true;
		}
		if (setupState == DLLN33_NETWORK_SETUP_FAILED) {
			snprintf(line, sizeof(line), "ZB FAIL %02X",
					 (unsigned int)DLLN33_GetNetworkSetupError());
			Display_ShowString(1, 0, line);
			delay_ms(1500);
			return false;
		}
	}
}
