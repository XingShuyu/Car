#include "Grayscale_Scan.h"

float eOld = 0.0f;

// 中间四个光电返回直线pid结果
float Grayscale_Line(bool *sensor_values, PID *pid) {
	// static float last_calculated_sensor_values = 0;
	// float now_calculated_sensor_values = 0;
	// float out;
	// Grayscale_Sensor_Read_Main(sensor_values);
	// now_calculated_sensor_values =
	//     -2.0f * sensor_values[2] - 1.0f * sensor_values[3] +
	//     1.0f * sensor_values[4] + 2.0f * sensor_values[5];
	// out = PID_calculate(pid, now_calculated_sensor_values,
	//                     last_calculated_sensor_values);
	// last_calculated_sensor_values = now_calculated_sensor_values;

	// Grayscale_Sensor_Read_Main(sensor_values);
	Grayscale_Sensor_Read_All(sensor_values);
	// 重心归一化
	float eRow = (-4.0*sensor_values[0]-3.0*sensor_values[1]-2.0*sensor_values[2]-1.0*sensor_values[3]+1.0*sensor_values[4]+2.0*sensor_values[5]+3.0*sensor_values[6]+4.0*sensor_values[7])/(sensor_values[0]+sensor_values[1]+sensor_values[2]+sensor_values[3]+sensor_values[4]+sensor_values[5]+sensor_values[6]+sensor_values[7]+0.00000001);
	float eNew = A*eOld+(1-A)*eRow;
	//计算转向参数
	float w = pid->p*eNew+pid->d*(eNew-eOld)/0.01;
	eOld = eNew;
	return w/4.0f;
}

/*十字路口/直角弯道判断
 * @param sensor_values 传感器状态数组
 * @param status 2为左直角，1为右直角，0为十字路口/丁字
 * @return 是否符合
 */
bool Grayscale_Cross(bool *sensor_values, int status) {
	Grayscale_Sensor_Read_Other(sensor_values);
	if (status == 0) {
		return (sensor_values[0] > 0 && sensor_values[1] > 0 &&
				sensor_values[6] > 0 && sensor_values[7] > 0);
	} else if (status == 1) {
		return (sensor_values[0] > 0 && sensor_values[1] > 0 &&
				sensor_values[6] == 0 && sensor_values[7] == 0);
	} else if (status == 2) {
		return (sensor_values[0] == 0 && sensor_values[1] == 0 &&
				sensor_values[6] > 0 && sensor_values[7] > 0);
	} else {
		return false;
	}
}