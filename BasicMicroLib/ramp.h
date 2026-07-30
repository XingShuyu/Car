#ifndef RAMP_H
#define RAMP_H

/**
 * @brief 以单次最大步长使数值向目标值靠近，不会越过目标。
 *
 * 该函数与单位无关；速度控制中可传入 mm/s 和 mm/s/调用周期。
 * @param current 当前值。
 * @param target 目标值。
 * @param max_step 本次调用允许的最大增量，必须为正数。
 * @return 向 target 靠近后的值；max_step 非正时保持 current。
 */
static inline float RampTo(float current, float target, float max_step)
{
	float error = target - current;

	if (max_step <= 0.0f) {
		return current;
	}
	if (error > max_step) {
		return current + max_step;
	}
	if (error < -max_step) {
		return current - max_step;
	}
	return target;
}

#endif
