#include "Drivers/board_isr.h"

#include "BasicMicroLib/usart.h"
#include "Communication/dl_ln33.h"
#include "Communication/maixcam_serial.h"
#include "Drivers/button_select.h"
#include "InfraredSpeed/infrared_speed.h"
#include "Motor/motor_encoder.h"
#include "ti_msp_dl_config.h"

void BoardIrq_Enable(void)
{
	// 开启 GPIOA 和 GPIOB 的全局中断 (因为编码器引脚跨越了这两个端口)
	NVIC_EnableIRQ(MotorMonitor_GPIOA_INT_IRQN);
	NVIC_EnableIRQ(GPIOB_INT_IRQn);
	NVIC_EnableIRQ(UART_MAIXCAM_INST_INT_IRQN); // 初始化maixcam
	NVIC_EnableIRQ(DL_LN33_INST_INT_IRQN);
}

// MSPM0 的 GPIOA/GPIOB 外部中断属于 GROUP1 向量，
// 这里做一次分发，避免中断落入默认处理函数导致“卡死”。
void GROUP1_IRQHandler(void)
{
	int gpioA_iidx, gpioB_iidx;

	// 分别查询两个 PORT 的待处理中断
	gpioA_iidx = DL_GPIO_getPendingInterrupt(GPIOA);
	gpioB_iidx = DL_GPIO_getPendingInterrupt(GPIOB);

	(void)MotorEncoder_HandleGpioAInterrupt(gpioA_iidx);
	(void)MotorEncoder_HandleGpioBInterrupt(gpioB_iidx);
	(void)ButtonSelect_HandleGpioBInterrupt(gpioB_iidx);
	(void)InfraredSpeed_HandleGpioBInterrupt(gpioB_iidx);
}

// 串口的中断服务函数
void Bluetooth_INST_IRQHandler(void)
{
	USART_IRQHandler();
}

// DL-LN33 的 UART2 接收与错误中断。
void DL_LN33_INST_IRQHandler(void)
{
	DLLN33_IRQHandler();
}

// maixcam的串口中断服务
void UART_MAIXCAM_INST_IRQHandler(void)
{
	MaixCamSerial_IRQHandler();
}
