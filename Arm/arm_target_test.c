#include "Arm/arm_target_test.h"

#include "Arm/arm_ik_motion.h"
#include "Drivers/button_select.h"
#include "OLED/display.h"

static ArmTargetTest_Status armTargetTestStatus = ArmTargetTestIdle;

static void ArmTargetTest_ShowStatus(ArmTargetTest_Status status)
{
	Display_Clear();
	Display_ShowString(0U, 0U, "ARM TARGET TEST");
	if (status == ArmTargetTestMoving) {
		Display_ShowString(2U, 0U, "TARGET MOVING");
	} else if (status == ArmTargetTestCompleted) {
		Display_ShowString(2U, 0U, "TARGET REACHED");
	} else if (status == ArmTargetTestCannotReach) {
		Display_ShowString(2U, 0U, "CANT REACH");
	} else if (status == ArmTargetTestFailed) {
		Display_ShowString(2U, 0U, "TARGET START FAIL");
	}
	Display_ShowString(6U, 0U, "B1 EXIT");
}

bool ArmTargetTest_SubmitTarget(float yaw_deg, float x_mm, float y_mm)
{
	if (armTargetTestStatus == ArmTargetTestMoving) {
		return false;
	}

	if (!ArmIkMotion_Start(yaw_deg, x_mm, y_mm)) {
		armTargetTestStatus =
			ArmIkMotion_LastStartWasReachabilityFailure() ?
				ArmTargetTestCannotReach : ArmTargetTestFailed;
		return false;
	}

	armTargetTestStatus = ArmTargetTestMoving;
	return true;
}

ArmTargetTest_Status ArmTargetTest_Update(void)
{
	if (armTargetTestStatus == ArmTargetTestMoving) {
		ArmMotionState_Status motionStatus = ArmIkMotion_Update();

		if (motionStatus == ArmMotionStateCompleted) {
			armTargetTestStatus = ArmTargetTestCompleted;
		} else if (motionStatus == ArmMotionStateFailed) {
			armTargetTestStatus = ArmTargetTestFailed;
		}
	}

	return armTargetTestStatus;
}

void ArmTargetTest_Reset(void)
{
	ArmIkMotion_Reset();
	armTargetTestStatus = ArmTargetTestIdle;
}

void ArmTargetTest_Run(void)
{
	float yaw_deg = 45.0F;
	float x_mm = 200.0F;
	float y_mm = 200.0F;
	ArmTargetTest_Status lastStatus = ArmTargetTestIdle;

	ArmTargetTest_Reset();
	ButtonSelect_ResetEvents();
	(void)ArmTargetTest_SubmitTarget(yaw_deg, x_mm, y_mm);
	while (true) {
		ArmTargetTest_Status status;

		if (ButtonSelect_TakeEvent() == ButtonSelectEventNext) {
			ArmTargetTest_Reset();
			Display_Clear();
			Display_ShowString(2U, 0U, "ARM TEST EXIT");
			return;
		}

		status = ArmTargetTest_Update();
		if (status != lastStatus) {
			ArmTargetTest_ShowStatus(status);
			lastStatus = status;
		}
	}
}
