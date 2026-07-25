#include "Arm/arm_ik_motion.h"

#include "Arm/jibot_servo.h"
#include "BasicMicroLib/getTime.h"

#include <float.h>
#include <math.h>

#define ARM_IK_MOTION_PI_F                 (3.14159265358979323846F)
#define ARM_IK_MOTION_DEG_TO_RAD           (ARM_IK_MOTION_PI_F / 180.0F)
#define ARM_IK_MOTION_RAD_TO_DEG           (180.0F / ARM_IK_MOTION_PI_F)
#define ARM_IK_MOTION_CENTER_PWM           (1500.0F)
#define ARM_IK_MOTION_PWM_PER_DEGREE       (2000.0F / 270.0F)
#define ARM_IK_MOTION_REACH_EPSILON        (1.0e-5F)
#define ARM_IK_MOTION_POSITION_EPSILON_MM  (0.1F)
#define ARM_IK_MOTION_ANGLE_EPSILON_RAD    (1.0e-5F)
#define ARM_IK_MOTION_SCORE_EPSILON        (1.0e-6F)
#define ARM_IK_MOTION_TWO_PI_F             (2.0F * ARM_IK_MOTION_PI_F)
#define ARM_IK_MOTION_DEFAULT_READ_TIMEOUT (20U)

/*
 * 默认标定：ID0 的 1500 PWM 指向车头正前方；ID1~3 的模型正方向分别
 * 映射为 -/+/- 的 PWM 正方向。所有零位均为 1500 PWM，ID4/5 固定居中。
 * 实机标定后可通过 ArmIkMotion_SetCalibration() 覆盖这些默认值。
 */
static ArmIkMotion_Calibration armIkMotionCalibration = {
	91.0F,
	104.0F,
	74.5F,
	174.0F,
	{-130.0F, -130.0F, -130.0F},
	{130.0F, 130.0F, 130.0F},
	{0.0F, 0.0F, 0.0F, 0.0F},
	{1, -1, 1, -1},
	{JIBOT_SERVO_MIN_PWM, JIBOT_SERVO_MIN_PWM,
	 JIBOT_SERVO_MIN_PWM, JIBOT_SERVO_MIN_PWM},
	{JIBOT_SERVO_MAX_PWM, JIBOT_SERVO_MAX_PWM,
	 JIBOT_SERVO_MAX_PWM, JIBOT_SERVO_MAX_PWM},
	{1500U, 1500U},
	ARM_IK_MOTION_DEFAULT_READ_TIMEOUT,
};

/* 单帧静态缓冲在整个状态机运行期间有效。 */
static uint16_t armIkMotionPwmFrame[1][JIBOT_SERVO_COUNT];
static ArmMotionState armIkMotionState;
static bool armIkMotionLastStartReachabilityFailure = false;

typedef struct ArmIkMotion_JointLimits {
	float minimumRad[3];
	float maximumRad[3];
} ArmIkMotion_JointLimits;

typedef struct ArmIkMotion_BestCandidate {
	bool found;
	float qRad[3];
	float directionError;
	float movementScore;
} ArmIkMotion_BestCandidate;

typedef struct ArmIkMotion_Circle {
	float centerX;
	float centerY;
	float radius;
} ArmIkMotion_Circle;

static bool ArmIkMotion_IsFinite(float value)
{
	return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static float ArmIkMotion_Clamp(float value, float minimum, float maximum)
{
	if (value < minimum) {
		return minimum;
	}
	if (value > maximum) {
		return maximum;
	}
	return value;
}

static float ArmIkMotion_WrapPi(float value)
{
	while (value > ARM_IK_MOTION_PI_F) {
		value -= 2.0F * ARM_IK_MOTION_PI_F;
	}
	while (value <= -ARM_IK_MOTION_PI_F) {
		value += 2.0F * ARM_IK_MOTION_PI_F;
	}
	return value;
}

static bool ArmIkMotion_IsValidPwm(uint16_t pwm)
{
	return (pwm >= JIBOT_SERVO_MIN_PWM) &&
		   (pwm <= JIBOT_SERVO_MAX_PWM);
}

static bool ArmIkMotion_IsCalibrationValid(
	const ArmIkMotion_Calibration *calibration)
{
	uint8_t axis;

	if ((calibration == NULL) ||
		!ArmIkMotion_IsFinite(calibration->baseHeightMm) ||
		!ArmIkMotion_IsFinite(calibration->link1Mm) ||
		!ArmIkMotion_IsFinite(calibration->link2Mm) ||
		!ArmIkMotion_IsFinite(calibration->link3Mm) ||
		(calibration->baseHeightMm < 0.0F) ||
		(calibration->link1Mm <= 0.0F) ||
		(calibration->link2Mm <= 0.0F) ||
		(calibration->link3Mm <= 0.0F)) {
		return false;
	}

	for (axis = 0U; axis < 4U; axis++) {
		if (!ArmIkMotion_IsFinite(calibration->axisZeroDeg[axis]) ||
			((calibration->axisDirection[axis] != 1) &&
			 (calibration->axisDirection[axis] != -1)) ||
			!ArmIkMotion_IsValidPwm(calibration->axisMinPwm[axis]) ||
			!ArmIkMotion_IsValidPwm(calibration->axisMaxPwm[axis]) ||
			(calibration->axisMinPwm[axis] >
			 calibration->axisMaxPwm[axis])) {
			return false;
		}
	}

	for (axis = 0U; axis < 3U; axis++) {
		if (!ArmIkMotion_IsFinite(calibration->jointMinDeg[axis]) ||
			!ArmIkMotion_IsFinite(calibration->jointMaxDeg[axis]) ||
			(calibration->jointMinDeg[axis] >
			 calibration->jointMaxDeg[axis])) {
			return false;
		}
	}

	return ArmIkMotion_IsValidPwm(calibration->fixedPwm[0]) &&
		   ArmIkMotion_IsValidPwm(calibration->fixedPwm[1]);
}

static bool ArmIkMotion_ModelDegToPwm(uint8_t axis, float modelDeg,
						  uint16_t *pwm)
{
	float servoDeg;
	float pwmValue;

	if ((axis >= 4U) || (pwm == NULL) || !ArmIkMotion_IsFinite(modelDeg)) {
		return false;
	}

	servoDeg = armIkMotionCalibration.axisZeroDeg[axis] +
		((float)armIkMotionCalibration.axisDirection[axis] * modelDeg);
	if (!ArmIkMotion_IsFinite(servoDeg) ||
		(servoDeg < JIBOT_SERVO_MIN_ANGLE_DEG) ||
		(servoDeg > JIBOT_SERVO_MAX_ANGLE_DEG)) {
		return false;
	}

	pwmValue = ARM_IK_MOTION_CENTER_PWM +
		(servoDeg * ARM_IK_MOTION_PWM_PER_DEGREE);
	if (!ArmIkMotion_IsFinite(pwmValue) ||
		(pwmValue < (float)JIBOT_SERVO_MIN_PWM) ||
		(pwmValue > (float)JIBOT_SERVO_MAX_PWM)) {
		return false;
	}

	*pwm = (uint16_t)(pwmValue + 0.5F);
	return (*pwm >= armIkMotionCalibration.axisMinPwm[axis]) &&
		   (*pwm <= armIkMotionCalibration.axisMaxPwm[axis]);
}

static bool ArmIkMotion_ReadCurrentJointDeg(float currentDeg[3])
{
	uint8_t joint;

	if (currentDeg == NULL) {
		return false;
	}

	for (joint = 0U; joint < 3U; joint++) {
		uint16_t pwm;
		float servoDeg;

		if (!JibotServo_ReadPosition((uint8_t)(JIBOT_SERVO_ID_SHOULDER +
												 joint),
										&pwm,
										armIkMotionCalibration.positionReadTimeoutMs)) {
			return false;
		}

		servoDeg = ((float)pwm - ARM_IK_MOTION_CENTER_PWM) /
			ARM_IK_MOTION_PWM_PER_DEGREE;
		currentDeg[joint] =
			(servoDeg - armIkMotionCalibration.axisZeroDeg[joint + 1U]) /
			(float)armIkMotionCalibration.axisDirection[joint + 1U];
	}

	return true;
}

/*
 * 将模型限位与实际可映射到的 PWM 范围求交集。自动求解以该范围作为
 * q1/q2/q3 的边界，因此即使零位或软件 PWM 限位收窄，也不会漏掉次优解。
 */
static bool ArmIkMotion_GetEffectiveJointLimits(
	ArmIkMotion_JointLimits *limits)
{
	uint8_t joint;

	if (limits == NULL) {
		return false;
	}

	for (joint = 0U; joint < 3U; joint++) {
		uint8_t axis = (uint8_t)(joint + 1U);
		float servoMinimum;
		float servoMaximum;
		float modelMinimum;
		float modelMaximum;
		float effectiveMinimum;
		float effectiveMaximum;

		servoMinimum = ((float)armIkMotionCalibration.axisMinPwm[axis] -
			ARM_IK_MOTION_CENTER_PWM) / ARM_IK_MOTION_PWM_PER_DEGREE;
		servoMaximum = ((float)armIkMotionCalibration.axisMaxPwm[axis] -
			ARM_IK_MOTION_CENTER_PWM) / ARM_IK_MOTION_PWM_PER_DEGREE;
		servoMinimum = ArmIkMotion_Clamp(servoMinimum,
			JIBOT_SERVO_MIN_ANGLE_DEG, JIBOT_SERVO_MAX_ANGLE_DEG);
		servoMaximum = ArmIkMotion_Clamp(servoMaximum,
			JIBOT_SERVO_MIN_ANGLE_DEG, JIBOT_SERVO_MAX_ANGLE_DEG);

		if (armIkMotionCalibration.axisDirection[axis] > 0) {
			modelMinimum = servoMinimum -
				armIkMotionCalibration.axisZeroDeg[axis];
			modelMaximum = servoMaximum -
				armIkMotionCalibration.axisZeroDeg[axis];
		} else {
			modelMinimum = armIkMotionCalibration.axisZeroDeg[axis] -
				servoMaximum;
			modelMaximum = armIkMotionCalibration.axisZeroDeg[axis] -
				servoMinimum;
		}

		effectiveMinimum = armIkMotionCalibration.jointMinDeg[joint];
		if (modelMinimum > effectiveMinimum) {
			effectiveMinimum = modelMinimum;
		}
		effectiveMaximum = armIkMotionCalibration.jointMaxDeg[joint];
		if (modelMaximum < effectiveMaximum) {
			effectiveMaximum = modelMaximum;
		}

		if (!ArmIkMotion_IsFinite(effectiveMinimum) ||
			!ArmIkMotion_IsFinite(effectiveMaximum) ||
			(effectiveMinimum > effectiveMaximum)) {
			return false;
		}

		limits->minimumRad[joint] =
			effectiveMinimum * ARM_IK_MOTION_DEG_TO_RAD;
		limits->maximumRad[joint] =
			effectiveMaximum * ARM_IK_MOTION_DEG_TO_RAD;
	}

	return true;
}

/* 已知二连杆末端位置，返回两组 (q1, q2) 解析解。坐标采用 e=[sin, cos]。 */
static bool ArmIkMotion_SolveTwoLink(float targetX, float targetY,
	float link1, float link2, float qPair[2][2])
{
	float d;
	float sineElbow;
	float bearing;
	uint8_t branch;

	if ((qPair == NULL) || !ArmIkMotion_IsFinite(targetX) ||
		!ArmIkMotion_IsFinite(targetY) || !ArmIkMotion_IsFinite(link1) ||
		!ArmIkMotion_IsFinite(link2) || (link1 <= 0.0F) ||
		(link2 <= 0.0F)) {
		return false;
	}

	d = (targetX * targetX + targetY * targetY - link1 * link1 -
		link2 * link2) / (2.0F * link1 * link2);
	if (!ArmIkMotion_IsFinite(d) ||
		(d > 1.0F + ARM_IK_MOTION_REACH_EPSILON) ||
		(d < -1.0F - ARM_IK_MOTION_REACH_EPSILON)) {
		return false;
	}

	d = ArmIkMotion_Clamp(d, -1.0F, 1.0F);
	sineElbow = sqrtf(ArmIkMotion_Clamp(1.0F - d * d, 0.0F, 1.0F));
	bearing = atan2f(targetX, targetY);

	for (branch = 0U; branch < 2U; branch++) {
		float sign = (branch == 0U) ? 1.0F : -1.0F;
		float q2 = atan2f(sign * sineElbow, d);
		float shoulderOffset = atan2f(link2 * sign * sineElbow,
			link1 + link2 * d);

		qPair[branch][0] = ArmIkMotion_WrapPi(bearing - shoulderOffset);
		qPair[branch][1] = q2;
	}

	return true;
}

/* 仅保留真实到达目标、处于有效限位且可映射为 PWM 的候选姿态。 */
static void ArmIkMotion_TryCandidate(const float qRad[3], float targetX,
	float targetY, const float currentDeg[3],
	const ArmIkMotion_JointLimits *limits,
	ArmIkMotion_BestCandidate *best)
{
	float phi;
	float calculatedX;
	float calculatedY;
	float directionError;
	float movementScore = 0.0F;
	uint8_t joint;

	if ((qRad == NULL) || (currentDeg == NULL) || (limits == NULL) ||
		(best == NULL)) {
		return;
	}

	for (joint = 0U; joint < 3U; joint++) {
		uint16_t pwm;

		if (!ArmIkMotion_IsFinite(qRad[joint]) ||
			(qRad[joint] < limits->minimumRad[joint] -
				ARM_IK_MOTION_ANGLE_EPSILON_RAD) ||
			(qRad[joint] > limits->maximumRad[joint] +
				ARM_IK_MOTION_ANGLE_EPSILON_RAD) ||
			!ArmIkMotion_ModelDegToPwm((uint8_t)(joint + 1U),
				qRad[joint] * ARM_IK_MOTION_RAD_TO_DEG, &pwm)) {
			return;
		}
	}

	phi = qRad[0] + qRad[1] + qRad[2];
	calculatedX = armIkMotionCalibration.link1Mm * sinf(qRad[0]) +
		armIkMotionCalibration.link2Mm * sinf(qRad[0] + qRad[1]) +
		armIkMotionCalibration.link3Mm * sinf(phi);
	calculatedY = armIkMotionCalibration.baseHeightMm +
		armIkMotionCalibration.link1Mm * cosf(qRad[0]) +
		armIkMotionCalibration.link2Mm * cosf(qRad[0] + qRad[1]) +
		armIkMotionCalibration.link3Mm * cosf(phi);
	if ((fabsf(calculatedX - targetX) > ARM_IK_MOTION_POSITION_EPSILON_MM) ||
		(fabsf(calculatedY - targetY) > ARM_IK_MOTION_POSITION_EPSILON_MM)) {
		return;
	}

	directionError = fabsf(ArmIkMotion_WrapPi(phi - ARM_IK_MOTION_PI_F));
	for (joint = 0U; joint < 3U; joint++) {
		float error = ArmIkMotion_WrapPi(qRad[joint] -
			currentDeg[joint] * ARM_IK_MOTION_DEG_TO_RAD);
		movementScore += error * error;
	}

	if (!best->found ||
		(directionError < best->directionError -
			ARM_IK_MOTION_SCORE_EPSILON) ||
		((fabsf(directionError - best->directionError) <=
			ARM_IK_MOTION_SCORE_EPSILON) &&
		 (movementScore < best->movementScore -
			ARM_IK_MOTION_SCORE_EPSILON))) {
		for (joint = 0U; joint < 3U; joint++) {
			best->qRad[joint] = qRad[joint];
		}
		best->directionError = directionError;
		best->movementScore = movementScore;
		best->found = true;
	}
}

/* 对一个 q1/q2 解，枚举 atan2 主值相差 2pi 的全部可行 q3 表示。 */
static void ArmIkMotion_TryDirectionCandidates(float q1, float q2, float phi,
	float targetX, float targetY, const float currentDeg[3],
	const ArmIkMotion_JointLimits *limits,
	ArmIkMotion_BestCandidate *best)
{
	int8_t turn;

	for (turn = -1; turn <= 1; turn++) {
		float qRad[3];

		qRad[0] = q1;
		qRad[1] = q2;
		qRad[2] = phi + (float)turn * ARM_IK_MOTION_TWO_PI_F - q1 - q2;
		ArmIkMotion_TryCandidate(qRad, targetX, targetY, currentDeg,
			limits, best);
	}
}

static void ArmIkMotion_AddWristPointCandidates(float wristX, float wristY,
	float targetX, float targetY, const float currentDeg[3],
	const ArmIkMotion_JointLimits *limits,
	ArmIkMotion_BestCandidate *best)
{
	float qPair[2][2];
	float phi;
	uint8_t branch;

	if (!ArmIkMotion_SolveTwoLink(wristX,
		wristY - armIkMotionCalibration.baseHeightMm,
		armIkMotionCalibration.link1Mm, armIkMotionCalibration.link2Mm,
		qPair)) {
		return;
	}

	phi = atan2f(targetX - wristX, targetY - wristY);
	if (!ArmIkMotion_IsFinite(phi)) {
		return;
	}

	for (branch = 0U; branch < 2U; branch++) {
		ArmIkMotion_TryDirectionCandidates(qPair[branch][0], qPair[branch][1],
			phi, targetX, targetY, currentDeg, limits, best);
	}
}

/* 目标圆与一个 S03 边界圆的零、一或两个交点。 */
static void ArmIkMotion_AddCircleIntersectionCandidates(
	const ArmIkMotion_Circle *boundary, float targetX, float targetY,
	const float currentDeg[3], const ArmIkMotion_JointLimits *limits,
	ArmIkMotion_BestCandidate *best)
{
	float deltaX;
	float deltaY;
	float distance;
	float along;
	float perpendicularSquared;
	float perpendicular;
	float normalX;
	float normalY;
	float wristX;
	float wristY;

	if ((boundary == NULL) || (currentDeg == NULL) || (limits == NULL) ||
		(best == NULL)) {
		return;
	}

	deltaX = boundary->centerX - targetX;
	deltaY = boundary->centerY - targetY;
	distance = sqrtf(deltaX * deltaX + deltaY * deltaY);
	if (!ArmIkMotion_IsFinite(distance) ||
		(distance <= ARM_IK_MOTION_REACH_EPSILON) ||
		(distance > armIkMotionCalibration.link3Mm + boundary->radius +
			ARM_IK_MOTION_REACH_EPSILON) ||
		(distance < fabsf(armIkMotionCalibration.link3Mm - boundary->radius) -
			ARM_IK_MOTION_REACH_EPSILON)) {
		return;
	}

	along = (armIkMotionCalibration.link3Mm *
		armIkMotionCalibration.link3Mm - boundary->radius * boundary->radius +
		distance * distance) / (2.0F * distance);
	perpendicularSquared = armIkMotionCalibration.link3Mm *
		armIkMotionCalibration.link3Mm - along * along;
	if (perpendicularSquared < -ARM_IK_MOTION_REACH_EPSILON) {
		return;
	}
	perpendicular = sqrtf(ArmIkMotion_Clamp(perpendicularSquared, 0.0F,
		FLT_MAX));
	normalX = deltaX / distance;
	normalY = deltaY / distance;
	wristX = targetX + along * normalX - perpendicular * normalY;
	wristY = targetY + along * normalY + perpendicular * normalX;
	ArmIkMotion_AddWristPointCandidates(wristX, wristY, targetX, targetY,
		currentDeg, limits, best);

	if (perpendicular > ARM_IK_MOTION_REACH_EPSILON) {
		wristX = targetX + along * normalX + perpendicular * normalY;
		wristY = targetY + along * normalY - perpendicular * normalX;
		ArmIkMotion_AddWristPointCandidates(wristX, wristY, targetX, targetY,
			currentDeg, limits, best);
	}
}

static void ArmIkMotion_AddStrictDownCandidates(float targetX, float targetY,
	const float currentDeg[3], const ArmIkMotion_JointLimits *limits,
	ArmIkMotion_BestCandidate *best)
{
	float qPair[2][2];
	uint8_t branch;

	if (!ArmIkMotion_SolveTwoLink(targetX,
		targetY + armIkMotionCalibration.link3Mm -
			armIkMotionCalibration.baseHeightMm,
		armIkMotionCalibration.link1Mm, armIkMotionCalibration.link2Mm,
		qPair)) {
		return;
	}

	for (branch = 0U; branch < 2U; branch++) {
		ArmIkMotion_TryDirectionCandidates(qPair[branch][0], qPair[branch][1],
			ARM_IK_MOTION_PI_F, targetX, targetY, currentDeg, limits, best);
	}
}

/* q1/q2 限位及 q2=0 奇异弧，与目标圆的有限交点候选。 */
static void ArmIkMotion_AddS03BoundaryCandidates(float targetX, float targetY,
	const float currentDeg[3], const ArmIkMotion_JointLimits *limits,
	ArmIkMotion_BestCandidate *best)
{
	ArmIkMotion_Circle boundary;
	float q1Limit;
	float q2Limit;
	uint8_t index;

	if ((limits->minimumRad[1] <= ARM_IK_MOTION_ANGLE_EPSILON_RAD) &&
		(limits->maximumRad[1] >= -ARM_IK_MOTION_ANGLE_EPSILON_RAD)) {
		boundary.centerX = 0.0F;
		boundary.centerY = armIkMotionCalibration.baseHeightMm;
		boundary.radius = armIkMotionCalibration.link1Mm +
			armIkMotionCalibration.link2Mm;
		ArmIkMotion_AddCircleIntersectionCandidates(&boundary, targetX,
			targetY, currentDeg, limits, best);
	}

	for (index = 0U; index < 2U; index++) {
		q1Limit = (index == 0U) ? limits->minimumRad[0] :
			limits->maximumRad[0];
		boundary.centerX = armIkMotionCalibration.link1Mm * sinf(q1Limit);
		boundary.centerY = armIkMotionCalibration.baseHeightMm +
			armIkMotionCalibration.link1Mm * cosf(q1Limit);
		boundary.radius = armIkMotionCalibration.link2Mm;
		ArmIkMotion_AddCircleIntersectionCandidates(&boundary, targetX,
			targetY, currentDeg, limits, best);
	}

	for (index = 0U; index < 2U; index++) {
		q2Limit = (index == 0U) ? limits->minimumRad[1] :
			limits->maximumRad[1];
		boundary.centerX = 0.0F;
		boundary.centerY = armIkMotionCalibration.baseHeightMm;
		boundary.radius = sqrtf(armIkMotionCalibration.link1Mm *
			armIkMotionCalibration.link1Mm + armIkMotionCalibration.link2Mm *
			armIkMotionCalibration.link2Mm + 2.0F *
			armIkMotionCalibration.link1Mm * armIkMotionCalibration.link2Mm *
			cosf(q2Limit));
		ArmIkMotion_AddCircleIntersectionCandidates(&boundary, targetX,
			targetY, currentDeg, limits, best);
	}
}

/* q3 到达上下限时，将 L2/L3 合并为等效连杆后求解。 */
static void ArmIkMotion_AddQ3LimitCandidates(float targetX, float targetY,
	const float currentDeg[3], const ArmIkMotion_JointLimits *limits,
	ArmIkMotion_BestCandidate *best)
{
	uint8_t index;

	for (index = 0U; index < 2U; index++) {
		float q3 = (index == 0U) ? limits->minimumRad[2] :
			limits->maximumRad[2];
		float equivalentLength = sqrtf(armIkMotionCalibration.link2Mm *
			armIkMotionCalibration.link2Mm + armIkMotionCalibration.link3Mm *
			armIkMotionCalibration.link3Mm + 2.0F *
			armIkMotionCalibration.link2Mm * armIkMotionCalibration.link3Mm *
			cosf(q3));
		float equivalentOffset = atan2f(armIkMotionCalibration.link3Mm *
			sinf(q3), armIkMotionCalibration.link2Mm +
				armIkMotionCalibration.link3Mm * cosf(q3));
		float qPair[2][2];
		uint8_t branch;

		if (!ArmIkMotion_SolveTwoLink(targetX,
			targetY - armIkMotionCalibration.baseHeightMm,
			armIkMotionCalibration.link1Mm, equivalentLength, qPair)) {
			continue;
		}

		for (branch = 0U; branch < 2U; branch++) {
			float qRad[3];

			qRad[0] = qPair[branch][0];
			qRad[1] = ArmIkMotion_WrapPi(qPair[branch][1] -
				equivalentOffset);
			qRad[2] = q3;
			ArmIkMotion_TryCandidate(qRad, targetX, targetY, currentDeg,
				limits, best);
		}
	}
}

static bool ArmIkMotion_SolveAutoDown(float targetX, float targetY,
	const float currentDeg[3], const ArmIkMotion_JointLimits *limits,
	float outDeg[3])
{
	ArmIkMotion_BestCandidate best = {false, {0.0F, 0.0F, 0.0F},
		FLT_MAX, FLT_MAX};
	uint8_t joint;

	if ((currentDeg == NULL) || (limits == NULL) || (outDeg == NULL)) {
		return false;
	}

	ArmIkMotion_AddStrictDownCandidates(targetX, targetY, currentDeg, limits,
		&best);
	if (!best.found ||
		(best.directionError > ARM_IK_MOTION_SCORE_EPSILON)) {
		ArmIkMotion_AddS03BoundaryCandidates(targetX, targetY, currentDeg,
			limits, &best);
		ArmIkMotion_AddQ3LimitCandidates(targetX, targetY, currentDeg,
			limits, &best);
	}

	if (!best.found) {
		return false;
	}

	for (joint = 0U; joint < 3U; joint++) {
		outDeg[joint] = best.qRad[joint] * ARM_IK_MOTION_RAD_TO_DEG;
	}
	return true;
}

bool ArmIkMotion_SetCalibration(const ArmIkMotion_Calibration *calibration)
{
	if ((armIkMotionState.status == ArmMotionStateRunning) ||
		!ArmIkMotion_IsCalibrationValid(calibration)) {
		return false;
	}

	armIkMotionCalibration = *calibration;
	return true;
}

void ArmIkMotion_GetCalibration(ArmIkMotion_Calibration *calibration)
{
	if (calibration != NULL) {
		*calibration = armIkMotionCalibration;
	}
}

bool ArmIkMotion_Start(float yaw_deg, float x_mm, float y_mm)
{
	float currentDeg[3];
	float solvedDeg[3];
	ArmIkMotion_JointLimits limits;
	ArmMotionState_Sequence sequence;
	uint8_t joint;

	armIkMotionLastStartReachabilityFailure = false;
	if ((armIkMotionState.status == ArmMotionStateRunning) ||
		!ArmIkMotion_IsCalibrationValid(&armIkMotionCalibration)) {
		return false;
	}
	if (!ArmIkMotion_IsFinite(yaw_deg) || !ArmIkMotion_IsFinite(x_mm) ||
		!ArmIkMotion_IsFinite(y_mm) || (x_mm < 0.0F)) {
		armIkMotionLastStartReachabilityFailure = true;
		return false;
	}
	if (!ArmIkMotion_ModelDegToPwm(JIBOT_SERVO_ID_BASE, yaw_deg,
			&armIkMotionPwmFrame[0][JIBOT_SERVO_ID_BASE])) {
		armIkMotionLastStartReachabilityFailure = true;
		return false;
	}
	if (!ArmIkMotion_GetEffectiveJointLimits(&limits)) {
		return false;
	}
	if (!ArmIkMotion_ReadCurrentJointDeg(currentDeg)) {
		return false;
	}
	if (!ArmIkMotion_SolveAutoDown(x_mm, y_mm, currentDeg, &limits,
			solvedDeg)) {
		armIkMotionLastStartReachabilityFailure = true;
		return false;
	}

	for (joint = 0U; joint < 3U; joint++) {
		if (!ArmIkMotion_ModelDegToPwm((uint8_t)(joint + 1U),
				solvedDeg[joint],
				&armIkMotionPwmFrame[0][joint + 1U])) {
			armIkMotionLastStartReachabilityFailure = true;
			return false;
		}
	}

	armIkMotionPwmFrame[0][JIBOT_SERVO_ID_WRIST_ORIENTATION] =
		armIkMotionCalibration.fixedPwm[0];
	armIkMotionPwmFrame[0][JIBOT_SERVO_ID_GRIPPER] =
		armIkMotionCalibration.fixedPwm[1];
	sequence.pwmData = &armIkMotionPwmFrame[0][0];
	sequence.frameCount = 1U;

	return ArmMotionState_Start(&armIkMotionState, &sequence, getNowMs());
}

bool ArmIkMotion_LastStartWasReachabilityFailure(void)
{
	return armIkMotionLastStartReachabilityFailure;
}

ArmMotionState_Status ArmIkMotion_Update(void)
{
	return ArmMotionState_Update(&armIkMotionState, getNowMs());
}

void ArmIkMotion_Reset(void)
{
	ArmMotionState_Reset(&armIkMotionState);
	armIkMotionLastStartReachabilityFailure = false;
}
