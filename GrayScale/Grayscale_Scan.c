#include "BasicMicroLib/PID.h"
#include "grayscale_sensor.h"

// 中间四个光电返回直线pid结果
static float Grayscale_Line(uint16_t *sensor_values, PID *pid) {
  static float last_calculated_sensor_values = 0;
  float now_calculated_sensor_values = 0;
  float out;
  Grayscale_Sensor_Read_Main(sensor_values);
  now_calculated_sensor_values =
      -2.0f * sensor_values[2] - 1.0f * sensor_values[3] +
      1.0f * sensor_values[4] + 2.0f * sensor_values[5];
  out = PID_calculate(pid, now_calculated_sensor_values,
                      last_calculated_sensor_values);
  last_calculated_sensor_values = now_calculated_sensor_values;
  return out;
}

/*十字路口/直角弯道判断
 * @param sensor_values 传感器状态数组
 * @param threshold 判断阈值
 * @param status 1为左直角，2为右直角，0为十字路口/丁字
 * @return 是否符合
 */
static bool Grayscale_Cross(uint16_t *sensor_values, uint16_t threshold,
                            int status) {
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