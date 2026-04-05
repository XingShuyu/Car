#include "grayscale_sensor.h"

static void _delay_us(volatile uint32_t us) { delay_us(us); }

// 选择传感器通道 select sensor channel
static void _select_channel(uint8_t channel) {
	SENSOR_AD0_WRITE((channel >> 0) & 0x01);
	SENSOR_AD1_WRITE((channel >> 1) & 0x01);
	SENSOR_AD2_WRITE((channel >> 2) & 0x01);
}

// 读取OUT引脚的值 read the value of OUT pin
bool Read_OUT_value(void) {
	unsigned int gAdcResult = 0;
	bool result;
	uint32_t timeout = 200000U;

	// 使能ADC转换
	DL_ADC12_enableConversions(ADC12_0_INST);
	// 软件触发ADC开始转换
	DL_ADC12_startConversion(ADC12_0_INST);

	// 如果当前状态 不是 空闲状态
	while (
		(DL_ADC12_getStatus(ADC12_0_INST) != DL_ADC12_STATUS_CONVERSION_IDLE) &&
		(timeout > 0U)) {
		timeout--;
	}

	// 清除触发转换状态
	DL_ADC12_stopConversion(ADC12_0_INST);
	// 失能ADC转换
	DL_ADC12_disableConversions(ADC12_0_INST);

	// 防止ADC状态异常导致主循环永久卡住
	if (timeout == 0U) {
		return 0;
	}

	// 获取数据
	gAdcResult = DL_ADC12_getMemResult(ADC12_0_INST, ADC12_0_ADCMEM_Grayscale);
    //归一化处理
	if (gAdcResult <= THRESHOLD)
		result = false;
	else
		result = true;
	return result;
}

// 切换通道后进行稳定读取：丢弃首个样本，减少上一通道残留影响
bool _read_channel_stable(uint8_t channel) {
	_select_channel(channel);
	_delay_us(50);
	Read_OUT_value();

	_select_channel(channel);
	_delay_us(50);

	// // ADC采样保持电容在通道切换后可能残留上一通道电压，首样本不用于控制
	// (void)Read_OUT_value();
	// _delay_us(10);

	return Read_OUT_value();
}

// 初始化灰度传感器所需的GPIO / init GPIO for grayscale sensor
void Grayscale_Sensor_Init(void) {}

// 读取所有8个通道的灰度值 read all 8 channels grayscale values
void Grayscale_Sensor_Read_All(bool *sensor_values) {
	uint8_t i;
	for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
		sensor_values[i] = _read_channel_stable(i);
	}
}

// 读取中间四个通道灰度值
void Grayscale_Sensor_Read_Main(bool *sensor_values) {
	uint8_t i;
	for (i = 2; i < 6; i++) {
		sensor_values[i] = _read_channel_stable(i);
	}
}

// 读取两边四个通道灰度值
void Grayscale_Sensor_Read_Other(bool *sensor_values) {
	uint8_t i;
	for (i = 0; i < 2; i++) {
		sensor_values[i] = _read_channel_stable(i);
	}
	for (i = 6; i < 8; i++) {
		sensor_values[i] = _read_channel_stable(i);
	}
}

// 读取单个指定通道的灰度值 read single specified channel grayscale value
bool Grayscale_Sensor_Read_Single(uint8_t channel) {
	if (channel >= GRAYSCALE_SENSOR_CHANNELS) {
		return 0; // 无效通道 // Invalid channel
	}
	return _read_channel_stable(channel);
}
