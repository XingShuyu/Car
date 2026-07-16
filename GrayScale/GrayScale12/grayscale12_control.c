/**
 * @file grayscale12_control.c
 * @brief NCHD12 归一化位置与 PID 转向修正计算。
 */

#include "GrayScale/GrayScale12/grayscale12_control.h"
#include <stddef.h>

#define GRAYSCALE12_DEFAULT_PERIOD_S (0.01f)

static float clamp_symmetric(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float get_period_s(const PID *pid)
{
    if ((pid == NULL) || (pid->t <= 0.0f)) {
        return GRAYSCALE12_DEFAULT_PERIOD_S;
    }

    return pid->t / 1000.0f;
}

static float update_pid(Grayscale12_LineController_t *controller, float error)
{
    PID *pid = controller->pid;
    float periodS = get_period_s(pid);
    float derivative = 0.0f;
    float integralTerm;

    if (controller->hasPreviousError) {
        derivative = (error - controller->previousError) / periodS;
    }

    controller->integral += error * periodS;
    integralTerm = pid->i * controller->integral;
    if (pid->i_Max > 0.0f) {
        integralTerm = clamp_symmetric(integralTerm, pid->i_Max);

        /* 反算累计值，避免积分项触及限幅后持续累积。 */
        if (pid->i != 0.0f) {
            controller->integral = integralTerm / pid->i;
        }
    }

    pid->saved_i = integralTerm;
    controller->previousError = error;
    controller->hasPreviousError = true;
    controller->lastPosition = error;
    controller->hasPosition = true;
    controller->lastOutput = pid->p * error + integralTerm + pid->d * derivative;

    return controller->lastOutput;
}

bool Grayscale12_LineController_Init(Grayscale12_LineController_t *controller,
                                     PID *pid)
{
    if ((controller == NULL) || (pid == NULL)) {
        return false;
    }

    controller->pid = pid;
    Grayscale12_LineController_Reset(controller);
    return true;
}

void Grayscale12_LineController_Reset(Grayscale12_LineController_t *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->integral = 0.0f;
    controller->previousError = 0.0f;
    controller->lastPosition = 0.0f;
    controller->lastOutput = 0.0f;
    controller->hasPreviousError = false;
    controller->hasPosition = false;

    if (controller->pid != NULL) {
        controller->pid->saved_i = 0.0f;
    }
}

bool Grayscale12_NormalizeRaw(uint16_t raw12, float *position)
{
    float weightedSum = 0.0f;
    uint8_t activeCount = 0u;

    if (position == NULL) {
        return false;
    }

    raw12 &= GRAYSCALE12_RAW_MASK;
    for (uint8_t i = 0u; i < GRAYSCALE12_CHANNELS; i++) {
        if ((raw12 & ((uint16_t)1u << i)) != 0u) {
            float channelPosition = -1.0f +
                (2.0f * (float)i / (float)(GRAYSCALE12_CHANNELS - 1u));

            weightedSum += channelPosition;
            activeCount++;
        }
    }

    if (activeCount == 0u) {
        return false;
    }

    *position = weightedSum / (float)activeCount;
    return true;
}

float Grayscale12_LineFromRaw(Grayscale12_LineController_t *controller,
                              uint16_t raw12, bool *lineDetected)
{
    float position;

    if ((controller == NULL) || (controller->pid == NULL) ||
        !Grayscale12_NormalizeRaw(raw12, &position)) {
        if (lineDetected != NULL) {
            *lineDetected = false;
        }
        return (controller != NULL) ? controller->lastOutput : 0.0f;
    }

    if (lineDetected != NULL) {
        *lineDetected = true;
    }
    return update_pid(controller, position);
}

float Grayscale12_Line(Grayscale12_LineController_t *controller,
                       bool sensorValues[GRAYSCALE12_CHANNELS],
                       bool *lineDetected)
{
    uint16_t raw12;

    if (!Grayscale12_ReadRaw(&raw12)) {
        if (sensorValues != NULL) {
            for (uint8_t i = 0u; i < GRAYSCALE12_CHANNELS; i++) {
                sensorValues[i] = false;
            }
        }
        if (lineDetected != NULL) {
            *lineDetected = false;
        }
        return (controller != NULL) ? controller->lastOutput : 0.0f;
    }

    if (sensorValues != NULL) {
        for (uint8_t i = 0u; i < GRAYSCALE12_CHANNELS; i++) {
            sensorValues[i] = ((raw12 & ((uint16_t)1u << i)) != 0u);
        }
    }

    return Grayscale12_LineFromRaw(controller, raw12, lineDetected);
}
