#include "oledfont.h"
#include "oled.h"

u8 OLED_GRAM[144][4]; // OLED显存缓冲区 OLED display buffer

/**
 * @brief 反显设置 模式选择 0正常显示 1反色显示
 * @param i Display mode selection 0:Normal 1:Inverse
 */
void OLED_ColorTurn(u8 i)
{
    if(i == 0) {
        OLED_WR_Byte(0xA6, OLED_CMD); // 正常显示 Normal display
    } 
    if(i == 1) {
        OLED_WR_Byte(0xA7, OLED_CMD); // 反色显示 Inverse display
    }
}

/**
 * @brief 屏幕旋转设置 模式选择 0正常方向 1旋转180度
 * @param i Rotation mode selection 0:Normal 1:Rotated 180 degrees
 */
void OLED_DisplayTurn(u8 i)
{
    if(i == 0) {
        OLED_WR_Byte(0xC8, OLED_CMD); // 正常方向 Normal orientation
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    if(i == 1) {
        OLED_WR_Byte(0xC0, OLED_CMD); // 反转方向 Rotated orientation
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

/**
 * @brief I2C延时函数 约4us延时
 * @note Approximately 4us delay for I2C timing
 */
void IIC_delay(void)
{
    delay_us(3); 
}

/**
 * @brief I2C起始信号 I2C start condition
 */
void I2C_Start(void)
{
    OLED_SDA_Set();
    OLED_SCL_Set();
    IIC_delay();
    OLED_SDA_Clr();
    IIC_delay();
    OLED_SCL_Clr();
    IIC_delay();
}

/**
 * @brief I2C停止信号 I2C stop condition
 */
void I2C_Stop(void)
{
    OLED_SDA_Clr();
    OLED_SCL_Set();
    IIC_delay();
    OLED_SDA_Set();
}

/**
 * @brief 等待应答信号 检测数据信号电平
 * @note Wait for ACK signal by checking data line level
 */
void I2C_WaitAck(void)
{
    OLED_SDA_Set();
    IIC_delay();
    OLED_SCL_Set();
    IIC_delay();
    OLED_SCL_Clr();
    IIC_delay();
}

/**
 * @brief 发送一个字节 将数据从最高位依次写入
 * @param dat 要发送的数据 Data to send
 */
void Send_Byte(u8 dat)
{
    u8 i;
    for(i = 0; i < 8; i++) {
        if(dat & 0x80) { // 发送最高位 Send MSB first
            OLED_SDA_Set();
        } else {
            OLED_SDA_Clr();
        }
        IIC_delay();
        OLED_SCL_Set();
        IIC_delay();
        OLED_SCL_Clr(); // 时钟下降沿锁存数据 Latch data on falling edge
        dat <<= 1;
    }
}

/**
 * @brief 向OLED写入一个字节 数据或命令
 * @param dat 要写入的数据 Data to write
 * @param mode 写入模式 0命令 1数据
 *        Write mode 0:Command 1:Data
 */
void OLED_WR_Byte(u8 dat, u8 mode)
{
    I2C_Start();
    Send_Byte(0x78); // OLED设备地址 OLED device address
    I2C_WaitAck();
    if(mode) {
        Send_Byte(0x40); // 数据模式 Data mode
    } else {
        Send_Byte(0x00); // 命令模式 Command mode
    }
    I2C_WaitAck();
    Send_Byte(dat);
    I2C_WaitAck();
    I2C_Stop();
}

/**
 * @brief 开启OLED显示 Turn on OLED display
 */
void OLED_DisPlay_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD); // 电荷泵使能 Enable charge pump
    OLED_WR_Byte(0x14, OLED_CMD); // 开启电荷泵 Charge pump on
    OLED_WR_Byte(0xAF, OLED_CMD); // 点亮屏幕 Display on
}

/**
 * @brief 关闭OLED显示 Turn off OLED display
 */
void OLED_DisPlay_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD); // 电荷泵使能 Enable charge pump
    OLED_WR_Byte(0x10, OLED_CMD); // 关闭电荷泵 Charge pump off
    OLED_WR_Byte(0xAE, OLED_CMD); // 关闭屏幕 Display off
}

/**
 * @brief 更新显存到OLED 刷新显示
 * @note Refresh display from GRAM buffer
 */
void OLED_Refresh(void)
{
    u8 i, n;
    for(i = 0; i < 4; i++) { // 4页(32行) 4 pages (32 rows)
        OLED_WR_Byte(0xB0 + i, OLED_CMD); // 设置页地址 Set page address
        OLED_WR_Byte(0x00, OLED_CMD);    // 设置列低地址 Set column low address
        OLED_WR_Byte(0x10, OLED_CMD);    // 设置列高地址 Set column high address
        
        I2C_Start();
        Send_Byte(0x78);     // OLED地址 OLED address
        I2C_WaitAck();
        Send_Byte(0x40);     // 数据模式 Data mode
        I2C_WaitAck();
        
        for(n = 0; n < 128; n++) { // 128列 128 columns
            Send_Byte(OLED_GRAM[n][i]); // 发送显存数据 Send GRAM data
            I2C_WaitAck();
        }
        I2C_Stop();
    }
}

/**
 * @brief 清空显存并刷新显示 Clear GRAM and refresh display
 */
void OLED_Clear(void)
{
    u8 i, n;
    for(i = 0; i < 4; i++) {
        for(n = 0; n < 128; n++) {
            OLED_GRAM[n][i] = 0; // 清除所有数据 Clear all data
        }
    }
    OLED_Refresh(); // 更新显示 Refresh display
}

/**
 * @brief 画点函数 在指定坐标绘制点
 * @param x X坐标(0~127) X coordinate
 * @param y Y坐标(0~63) Y coordinate
 * @param t 模式 1画点 0清点
 *        Mode 1:Draw point 0:Clear point
 */
void OLED_DrawPoint(u8 x, u8 y, u8 t)
{
    u8 i, m, n;
    i = y / 8;      // 计算页号 Calculate page index
    m = y % 8;      // 计算位偏移 Calculate bit offset
    n = 1 << m;     // 位掩码 Bit mask
    
    if(t) {
        OLED_GRAM[x][i] |= n;  // 设置点 Set point
    } else {
        OLED_GRAM[x][i] &= ~n; // 清除点 Clear point
    }
}

/**
 * @brief 画线函数 在两点之间绘制直线
 * @param x1,y1 起点坐标 Start point coordinates
 * @param x2,y2 终点坐标 End point coordinates
 * @param mode 模式 1画线 0清除
 *        Mode 1:Draw line 0:Clear line
 */
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2, u8 mode)
{
    u16 t; 
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    
    delta_x = x2 - x1; // 计算坐标增量 Calculate coordinate increment
    delta_y = y2 - y1;
    uRow = x1; // 画线起点坐标 Line start coordinate
    uCol = y1;
    
    // 确定X方向步进 Determine X direction step
    if(delta_x > 0) incx = 1; 
    else if (delta_x == 0) incx = 0; // 垂直线 Vertical line
    else {incx = -1; delta_x = -delta_x;}
    
    // 确定Y方向步进 Determine Y direction step
    if(delta_y > 0) incy = 1;
    else if (delta_y == 0) incy = 0; // 水平线 Horizontal line
    else {incy = -1; delta_y = -delta_y;}
    
    // 确定基本增量轴 Determine basic increment axis
    if(delta_x > delta_y) distance = delta_x; 
    else distance = delta_y;
    
    // Bresenham算法绘制直线 Bresenham algorithm for line drawing
    for(t = 0; t < distance + 1; t++)
    {
        OLED_DrawPoint(uRow, uCol, mode); // 绘制点 Draw point
        xerr += delta_x;
        yerr += delta_y;
        if(xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if(yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}



/**
 * @brief 显示字符函数 在指定位置显示单个字符
 * @param x,y 起始坐标 Start coordinates
 * @param chr 要显示的字符 Character to display
 * @param size1 字体大小 Font size
 * @param mode 模式 0反色显示 1正常显示
 *        Mode 0:Inverse display 1:Normal display
 */
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size1, u8 mode)
{
    u8 i, m, temp, size2, chr1;
    u8 x0 = x, y0 = y;
    
    // 计算字体数据大小 Calculate font data size
    if(size1 == 8) size2 = 6;
    else size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
    
    chr1 = chr - ' '; // 计算字库偏移 Calculate font offset
    
    for(i = 0; i < size2; i++)
    {
        // 获取字体数据 Get font data
        if(size1 == 8) temp = asc2_0806[chr1][i];
        else if(size1 == 12) temp = asc2_1206[chr1][i];
        else if(size1 == 16) temp = asc2_1608[chr1][i];
        else if(size1 == 24) temp = asc2_2412[chr1][i];
        else return;
        
        // 逐位绘制 Draw bit by bit
        for(m = 0; m < 8; m++)
        {
            if(temp & 0x01) OLED_DrawPoint(x, y, mode);
            else OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        // 换行处理 Line break handling
        if((size1 != 8) && ((x - x0) == size1 / 2))
        {
            x = x0;
            y0 = y0 + 8;
        }
        y = y0;
    }
}

/**
 * @brief 显示字符串函数 在指定位置显示字符串
 * @param x,y 起始坐标 Start coordinates
 * @param chr 字符串指针 String pointer
 * @param size1 字体大小 Font size
 * @param mode 模式 0反色显示 1正常显示
 *        Mode 0:Inverse display 1:Normal display
 */
void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size1, u8 mode)
{
    // 遍历字符串直到结束符 Traverse string until null terminator
    while((*chr >= ' ') && (*chr <= '~'))
    {
        OLED_ShowChar(x, y, *chr, size1, mode);
        // 计算下一个字符位置 Calculate next character position
        if(size1 == 8) x += 6;
        else x += size1 / 2;
        chr++;
    }
}

/**
 * @brief 幂函数计算 计算m的n次方
 * @param m 底数 Base
 * @param n 指数 Exponent
 * @return 计算结果 Calculation result
 */
u32 OLED_Pow(u8 m, u8 n)
{
    u32 result = 1;
    while(n--)
    {
        result *= m;
    }
    return result;
}

/**
 * @brief 显示数字函数 在指定位置显示数字
 * @param x,y 起始坐标 Start coordinates
 * @param num 要显示的数字 Number to display
 * @param len 数字位数 Number length
 * @param size1 字体大小 Font size
 * @param mode 模式 0反色显示 1正常显示
 *        Mode 0:Inverse display 1:Normal display
 */
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size1, u8 mode)
{
    u8 t, temp, m = 0;
    if(size1 == 8) m = 2; // 小字体额外间距 Extra spacing for small font
    
    for(t = 0; t < len; t++)
    {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10; // 提取每位数字 Extract each digit
        if(temp == 0)
        {
            OLED_ShowChar(x + (size1 / 2 + m) * t, y, '0', size1, mode);
        }
        else 
        {
            OLED_ShowChar(x + (size1 / 2 + m) * t, y, temp + '0', size1, mode);
        }
    }
}



/**
 * @brief OLED初始化函数 配置控制器并清屏
 * @note Initialize OLED controller and clear screen
 */
void OLED_Init(void)
{
    delay_ms(200); // 延时等待OLED上电稳定 Delay for OLED power stabilization
    
    // OLED控制器初始化命令序列 OLED controller initialization sequence
    OLED_WR_Byte(0xAE, OLED_CMD);  // 关闭显示避免闪烁 Display off to avoid flicker
    OLED_WR_Byte(0x00, OLED_CMD);  // 设置列地址低4位 Set lower column address
    OLED_WR_Byte(0x10, OLED_CMD);  // 设置列地址高4位 Set higher column address
    OLED_WR_Byte(0x00, OLED_CMD);  // 设置显示起始行 Set display start line
    OLED_WR_Byte(0xB0, OLED_CMD);  // 设置页地址 Set page address
    
    OLED_WR_Byte(0x81, OLED_CMD);  // 对比度控制 Contrast control
    OLED_WR_Byte(0xFF, OLED_CMD);  // 对比度值128 Contrast value 128
    
    OLED_WR_Byte(0xA1, OLED_CMD);  // 段重映射水平翻转 Segment remap horizontal flip
    OLED_WR_Byte(0xA6, OLED_CMD);  // 正常显示非反色 Normal display not inverse
    
    OLED_WR_Byte(0xA8, OLED_CMD);  // 设置多路复用比率 Set multiplex ratio
    OLED_WR_Byte(0x1F, OLED_CMD);  // 设置为1/32占空比 Set duty 1/32
    
    OLED_WR_Byte(0xC8, OLED_CMD);  // COM扫描方向垂直翻转 COM scan direction vertical flip
    
    OLED_WR_Byte(0xD3, OLED_CMD);  // 设置显示偏移 Set display offset
    OLED_WR_Byte(0x00, OLED_CMD);  // 无偏移 No offset
    
    OLED_WR_Byte(0xD5, OLED_CMD);  // 设置显示时钟分频 Set oscillator division
    OLED_WR_Byte(0x80, OLED_CMD);  // 默认值 Default value
    
    OLED_WR_Byte(0xD9, OLED_CMD);  // 设置预充电周期 Set pre-charge period
    OLED_WR_Byte(0x1F, OLED_CMD);  // 推荐值 Recommended value
    
    OLED_WR_Byte(0xDA, OLED_CMD);  // 设置COM引脚配置 Set COM pins configuration
    OLED_WR_Byte(0x00, OLED_CMD);  // 默认配置 Default configuration
    
    OLED_WR_Byte(0xDB, OLED_CMD);  // 设置VCOMH电压 Set VCOMH deselect level
    OLED_WR_Byte(0x40, OLED_CMD);  // 约0.77xVCC Approx 0.77xVCC
    
    OLED_WR_Byte(0x8D, OLED_CMD);  // 电荷泵设置 Charge pump setting
    OLED_WR_Byte(0x14, OLED_CMD);  // 启用电荷泵 Enable charge pump
    

    OLED_WR_Byte(0xAF, OLED_CMD);  // 开启显示 Display ON
    
    // 显示欢迎信息 Display welcome message
    OLED_ShowString(10, 40, (uint8_t *)"Hello,Yahboom!", 16, 1);
    OLED_Refresh();  // 更新显示 Refresh display
		delay_ms(200);
		OLED_Clear();  // 清空显存 Clear display buffer
    
}

/**
 * @brief 写入一行字符 在指定行显示字符串
 * @param data 字符串指针 String pointer
 * @param line 行号(1-4) Line number (1-4)
 * @param clear 是否清屏 Whether to clear screen
 * @param refresh 是否立即刷新 Whether to refresh immediately
 */
void OLED_Draw_Line(char *data, uint8_t line, bool clear, bool refresh)
{
    if (line > 0 && line <= 4) {
        if(clear == true) {
            OLED_Clear();
        }
        
        OLED_ShowString(0, (line-1)*8, (uint8_t *)data, 8, 1);
        
        if(refresh == true) {
            OLED_Refresh();
        }
    }
}