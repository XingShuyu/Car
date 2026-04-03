#include "motor.h"
#include "ti_msp_dl_config.h"

// 宏定义：你的配置文件里设置的 PWM 周期是 3200
#define PWM_PERIOD 3200

//目标转速
int32_t leftTargetSpeed=0,rightTargetSpeed=0;

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
	leftTargetSpeed = left_speed;
	rightTargetSpeed = right_speed;
	return;
}

//pid纠正速度
void Motor_PidSpeed(PID *motorPID, int32_t leftSpeed, int32_t rightSpeed) {
	int32_t leftBias, rightBias; // 定义相关变量
	static int32_t leftControlVelocity, leftLast_bias, rightControlVelocity,
		rightLast_bias; // 静态变量，函数调用结束后其值依然存在
	leftBias = leftTargetSpeed - leftSpeed;	   // 求速度偏差
	rightBias = rightTargetSpeed - rightSpeed; // 求速度偏差
	leftControlVelocity +=
		motorPID->p * (leftBias - leftLast_bias) +
		motorPID->i * leftBias; // 增量式PI控制器
								// Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
								// Velcity_Ki*Bias 速度控制值由Bias不断积分得到
								// 偏差越大加速度越大
	rightControlVelocity +=
		motorPID->p * (rightBias - rightLast_bias) + motorPID->i * rightBias;
	leftLast_bias = leftBias;
	rightLast_bias = rightBias;
	Motor_SetSpeed(leftControlVelocity, rightControlVelocity);
}

//根据pid返回值修改目标速度
void Motor_FixError(float error){
	leftTargetSpeed-=error;
	rightTargetSpeed+=error;
}