#include "Grayscale_Scan.h"

float eOld = 0.0f;
float sensor_weight[8] = {-3.0,-2.7,-2.5,-2.0,2.0,2.5,2.7,3.0};


// 中间四个光电返回直线pid结果
float Grayscale_Line(PID *pid,bool *sensor_values) {
	// static float wNow, eOld, ePrev;
	// error>0 左转
	static float error,eOld,eNew;
	float eRow,weightedSum;
	int activeSensor;
	Grayscale_Sensor_Read_All(sensor_values);
	weightedSum=0;
	activeSensor=0;
	// 重心归一化
	for(int i = 0;i<8;i++){
		if (sensor_values[i]) {
			weightedSum+=sensor_weight[i];
			activeSensor++;
		}
	}
	if (activeSensor==0) {
		eRow = eNew;
	}
	else {
		eRow=weightedSum/(float)activeSensor;
	}
	eNew = A * eNew + (1 - A) * eRow;
	error = pid->p*eNew+pid->d*(eNew-eOld)/pid->t;
	// // 计算转向参数
	// if (sensor_values[3] == 1 && sensor_values[4] == 1) {
	// 	error = 0;
	// }
	// if (sensor_values[3] == 1 && sensor_values[4] == 0) {
	// 	error = -0.05;
	// }
	// if (sensor_values[3] == 0 && sensor_values[4] == 1) {
	// 	error = 0.05;
	// }
	// if (sensor_values[5] == 1 && sensor_values[4] == 1) {
	// 	error = 0.1;
	// }
	// if (sensor_values[2] == 1 && sensor_values[3] == 1) {
	// 	error = -0.1;
	// }
	// if (sensor_values[2] == 1 && sensor_values[3] == 0) {
	// 	error = -0.2;
	// }
	// if (sensor_values[5] == 1 && sensor_values[4] == 0) {
	// 	error = 0.2;
	// }
	// if (sensor_values[1] == 1) {
	// 	error = -0.25;
	// }
	// if (sensor_values[0] == 1) {
	// 	error = -0.3;
	// }
	// if (sensor_values[6] == 1) {
	// 	error = 0.25;
	// }
	// if (sensor_values[7] == 1) {
	// 	error = 0.3;
	// }
	eOld = eNew;
	// printf("Back:%.2f\r\n",error);
	return error;

	// // wNow = pid->p * eNew + pid->d * (eNew - eOld);
	// wNow += PID_calculate(pid, eRow, eOld, ePrev);
	// ePrev = eOld;
	// eOld = eRow;
	// printf("Back:%.2f\r\n", eRow);
	// return eRow/3.0;
}

/*十字路口/直角弯道判断
 * @param sensor_values 传感器状态数组
 * @param status 2为左直角，1为右直角，0为十字路口/丁字
 * @return 是否符合
 */
bool Grayscale_Cross(bool *sensor_values, int status) {
	Grayscale_Sensor_Read_All(sensor_values);
	if (status == 2) {
		return (sensor_values[0] == 0 && sensor_values[6] > 0 &&
				sensor_values[4] > 0 && sensor_values[5] > 0);
	} else if (status == 1) {
		return (sensor_values[3] > 0 && sensor_values[1] > 0 && sensor_values[2] > 0 &&
				sensor_values[7] == 0);
	} else if (status == 0) {
		return (sensor_values[2] > 0 && sensor_values[1] > 0 &&
				sensor_values[6] > 0 && sensor_values[5] > 0);
	} else {
		return false;
	}
}

void Grayscale_Zero(bool *sensor_values){
	sensor_values[0]=0;
	sensor_values[1]=0;
	sensor_values[2]=0;
	sensor_values[3]=1;
	sensor_values[4]=1;
	sensor_values[5]=0;
	sensor_values[6]=0;
	sensor_values[7]=0;

}