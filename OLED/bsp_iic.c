#include "bsp_iic.h"




 
void OLED_WR_Byte(uint8_t addr, uint8_t dat) {
    uint8_t temp[2];
    temp[0] = addr;
    temp[1] = dat;
    DL_I2C_fillControllerTXFIFO(OLED_IIC_INST, temp, 2);
    while (!(DL_I2C_getControllerStatus(OLED_IIC_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
 
    DL_I2C_startControllerTransfer(OLED_IIC_INST, OLED_IIC_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(OLED_IIC_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
   
		while (!(DL_I2C_getControllerStatus(OLED_IIC_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_flushControllerTXFIFO(OLED_IIC_INST);
}


uint8_t OLED_WR_ReadByte(uint8_t addr) {
    uint8_t data;
 
    DL_I2C_fillControllerTXFIFO(OLED_IIC_INST, &addr, 1);
	
		DL_I2C_disableInterrupt(OLED_IIC_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
	
    while (!(DL_I2C_getControllerStatus(OLED_IIC_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(OLED_IIC_INST, OLED_IIC_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(OLED_IIC_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    while (!(DL_I2C_getControllerStatus(OLED_IIC_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
//    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
 
    DL_I2C_startControllerTransfer(OLED_IIC_INST, OLED_IIC_ADDR, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (DL_I2C_getControllerStatus(OLED_IIC_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    while (!(DL_I2C_getControllerStatus(OLED_IIC_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    //while (DL_I2C_isControllerRXFIFOEmpty(OLED_IIC_INST));
    data = DL_I2C_receiveControllerData(OLED_IIC_INST);
 
    return data;
}