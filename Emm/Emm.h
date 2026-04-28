#ifndef EMM_H_
#define EMM_H_
#include <stdio.h>

typedef struct Emm_Data{
    int addr;
    int funcCode;
    unsigned char * data;
    int dataSize;
    int checkSum;

} Emm_Data;

typedef struct LocControl{
    //方向:0/1正反
    int dirction;
    int speed;
    int accu;
    float angle;
    int mode;

} LocControl;


/**
 * @brief 向电机发送指令
 *
 * @param Emm_Data 电机指令结构体
 */
void Emm_SendData(Emm_Data * emm_Data);

/**
 * @brief 电机转角度
 *
 * @param emm_Data 电机指令结构体
 * @param locControl 电机位置结构体
 */
void Emm_Loc_Control(int Addr,LocControl* locControl);

/**
 * @brief 电机停转
 *
 * @param Emm_Data 电机指令结构体
 */
void Emm_Stop(int Addr);

/**
 * @brief 使能电机
 */
void Emm_Init(int Addr);
#endif