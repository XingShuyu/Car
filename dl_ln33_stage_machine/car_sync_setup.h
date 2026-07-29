#ifndef CAR_SYNC_SETUP_H
#define CAR_SYNC_SETUP_H

#include <stdbool.h>

#include "dl_ln33_stage_machine/car_sync.h"

/* Configure the selected multi-car role on the DL-LN33 module. */
bool CarSync_SetupNetwork(CarSyncRole role);

#endif
