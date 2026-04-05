#include "motor.h"
#include "GrayScale/grayscale_sensor.h"
#include "ti_msp_dl_config.h"

// 宏定义：你的配置文件里设置的 PWM 周期是 3200
#define PWM_PERIOD 3200

// 目标转速
int32_t leftTargetSpeed = 0, rightTargetSpeed = 0;
// 基础转速
int32_t leftBaseSpeed = 0, rightBaseSpeed = 0;
// 真实输入转速
int32_t leftRealSpeed = 0, rightRealSpeed = 0;

void Motor_Init(void) {
	// PWM 已在 SysConfig 中初始化，这里确保电机初始处于停止状态
	Motor_Brake();
}

void Motor_SetSpeed(int16_t left_speed, int16_t right_speed) {
	uint32_t left_pwm = 0;
	uint32_t right_pwm = 0;

	/*----------------- 处理左轮速度 -----------------*/
	// 强制使用快衰竭
	DL_TimerG_setCaptureCompareValue(MotorLeft_INST, 0, GPIO_MotorLeft_C0_IDX);
	DL_TimerG_setCaptureCompareValue(MotorLeft_INST, 0, GPIO_MotorLeft_C1_IDX);
	if (left_speed > 0) {
		// 正转：方向引脚 (PB21, 即 C0通道) 输出高电平 -> 占空比设为最大值
		left_pwm = (uint32_t)left_speed;
		// 限幅保护
		if (left_pwm > MOTOR_MAX_SPEED)
			left_pwm = MOTOR_MAX_SPEED;
		// 设置左轮速度PWM (PB11, 即 C1通道)
		DL_TimerG_setCaptureCompareValue(MotorLeft_INST, left_pwm,
										 GPIO_MotorLeft_C1_IDX);
	} else if (left_speed < 0) {
		// 反转：方向引脚 (PB21, 即 C0通道) 输出低电平 -> 占空比设为 0
		left_pwm = (uint32_t)-left_speed;
		// 限幅保护
		if (left_pwm > MOTOR_MAX_SPEED)
			left_pwm = MOTOR_MAX_SPEED;
		// 设置左轮速度PWM (PB11, 即 C1通道)
		DL_TimerG_setCaptureCompareValue(MotorLeft_INST, left_pwm,
										 GPIO_MotorLeft_C0_IDX);
	} else {
		// 速度为0：方向引脚置低
		DL_TimerG_setCaptureCompareValue(MotorLeft_INST, 0,
										 GPIO_MotorLeft_C0_IDX);
	}

	/*----------------- 处理右轮速度 -----------------*/
	// 强制使用快衰竭
	DL_TimerG_setCaptureCompareValue(MotorRight_INST, 0,
									 GPIO_MotorRight_C0_IDX);
	DL_TimerG_setCaptureCompareValue(MotorRight_INST, 0,
									 GPIO_MotorRight_C1_IDX);
	if (right_speed > 0) {
		// 正转：方向引脚 (PB21, 即 C0通道) 输出高电平 -> 占空比设为最大值
		right_pwm = (uint32_t)right_speed;
		// 限幅保护
		if (right_pwm > MOTOR_MAX_SPEED)
			right_pwm = MOTOR_MAX_SPEED;
		// 设置左轮速度PWM (PB11, 即 C1通道)
		DL_TimerG_setCaptureCompareValue(MotorRight_INST, right_pwm,
										 GPIO_MotorRight_C1_IDX);
	} else if (right_speed < 0) {
		// 反转：方向引脚 (PB10, 即 C0通道) 输出低电平 -> 占空比设为 0
		right_pwm = (uint32_t)-right_speed;
		// 限幅保护
		if (right_pwm > MOTOR_MAX_SPEED)
			right_pwm = MOTOR_MAX_SPEED;
		// 设置左轮速度PWM (PB11, 即 C1通道)
		DL_TimerG_setCaptureCompareValue(MotorRight_INST, right_pwm,
										 GPIO_MotorRight_C0_IDX);
	} else {
		// 速度为0：方向引脚置低
		DL_TimerG_setCaptureCompareValue(MotorRight_INST, 0,
										 GPIO_MotorRight_C0_IDX);
	}
}

void Motor_Brake(void) {
	/*
	 * 注意：根据 AT8236 手册的慢衰减逻辑：
	 * IN1=0, IN2=0 是 滑行（自由停止）
	 * IN1=1, IN2=1 是 刹车（短路制动，迅速停止）
	 *
	 * 这里默认采用“滑行”模式（全部设为0）。如果你想要急刹车，
	 * 请把下面所有的 0 改成 PWM_PERIOD。
	 */

	// 左轮滑行停止
	DL_TimerG_setCaptureCompareValue(MotorLeft_INST, 0,
									 GPIO_MotorLeft_C0_IDX); // 方向脚低电平
	DL_TimerG_setCaptureCompareValue(MotorLeft_INST, 0,
									 GPIO_MotorLeft_C1_IDX); // 速度脚低电平

	// 右轮滑行停止
	DL_TimerG_setCaptureCompareValue(MotorRight_INST, 0,
									 GPIO_MotorRight_C0_IDX); // 方向脚低电平
	DL_TimerG_setCaptureCompareValue(MotorRight_INST, 0,
									 GPIO_MotorRight_C1_IDX); // 速度脚低电平
}

void Motor_SetAccuSpeed(int32_t left_speed, int32_t right_speed) {
	leftBaseSpeed = left_speed;
	leftTargetSpeed = left_speed;
	rightBaseSpeed = right_speed;
	rightTargetSpeed = right_speed;
	return;
}

// pid纠正速度
void Motor_PidSpeed(PID *motorPID, int32_t leftSpeed, int32_t rightSpeed) {
	int32_t leftBias, rightBias; // 定义相关变量
	static int32_t leftLast_bias,rightLast_bias,leftPrev_bias,rightPrev_bias; // 静态变量，函数调用结束后其值依然存在
	leftBias = leftTargetSpeed - leftSpeed;	   // 求速度偏差
	rightBias = rightTargetSpeed - rightSpeed; // 求速度偏差
	leftRealSpeed += PID_calculate(motorPID, leftBias, leftLast_bias,leftPrev_bias);
	rightRealSpeed += PID_calculate(motorPID, rightBias, rightLast_bias,rightPrev_bias);
	leftPrev_bias = leftLast_bias;
	rightPrev_bias = rightLast_bias;
	leftLast_bias = leftBias;
	rightLast_bias = rightBias;
	Motor_SetSpeed(leftRealSpeed/100 , rightRealSpeed/100);
}

// 根据pid返回值修改目标速度
void Motor_FixError(float error) {
	leftTargetSpeed = leftBaseSpeed * (1 - error*0.5);
	rightTargetSpeed = rightBaseSpeed * (1 + error*0.5);
}

void Rush(void){
	Motor_SetAccuSpeed(5000,5000);
	delay_ms(500);
	Motor_SetAccuSpeed(0,0);
}

void RightRound(void){
	// Rush();
	Motor_SetAccuSpeed(5000, -5000);
	while(_read_channel_stable(4)==false)delay_ms(50);
	Motor_SetAccuSpeed(0,0);
}