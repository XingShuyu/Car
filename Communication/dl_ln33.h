#ifndef DL_LN33_H
#define DL_LN33_H

#include <stdbool.h>
#include <stdint.h>

/*
 * DL-LN33 <-> MSPM0G3507 接线：
 *   PB15 / UART2_TX -> DL-LN33 RX
 *   PB16 / UART2_RX <- DL-LN33 TX
 *   两端必须共地，模块供电为 3.3V。
 *
 * UART2 由 SysConfig 配置为 115200, 8N1, 无校验、无硬件流控。
 */

#define DLLN33_DEFAULT_BAUD_RATE      115200U
#define DLLN33_DEFAULT_BAUD_CODE      0x08U
#define DLLN33_MAX_LOGICAL_LENGTH     63U
#define DLLN33_MAX_PAYLOAD_LENGTH     59U
#define DLLN33_APPLICATION_PORT_MIN   0x80U
#define DLLN33_CONFIG_SOURCE_PORT     0x90U

#define DLLN33_PORT_LED               0x20U
#define DLLN33_PORT_MANAGEMENT        0x21U
#define DLLN33_PORT_ERROR_REPORT      0x22U

/* 0x21 基本信息管理端口命令。 */
#define DLLN33_CMD_READ_ADDRESS       0x01U
#define DLLN33_CMD_READ_NETWORK_ID    0x02U
#define DLLN33_CMD_READ_CHANNEL       0x03U
#define DLLN33_CMD_READ_BAUD          0x04U
#define DLLN33_CMD_RESTART            0x10U
#define DLLN33_CMD_SET_ADDRESS        0x11U
#define DLLN33_CMD_SET_NETWORK_ID     0x12U
#define DLLN33_CMD_SET_CHANNEL        0x13U
#define DLLN33_CMD_SET_BAUD           0x14U

/* 0x21 管理端口应答状态。 */
#define DLLN33_MANAGEMENT_OK          0x00U
#define DLLN33_MANAGEMENT_REMOTE_DENY 0xF0U
#define DLLN33_MANAGEMENT_BAD_COMMAND 0xF8U
#define DLLN33_MANAGEMENT_BAD_LENGTH  0xF9U
#define DLLN33_MANAGEMENT_BAD_VALUE   0xFAU

typedef struct {
	uint8_t source_port;
	uint8_t destination_port;
	uint16_t remote_address;
	uint8_t payload_length;
	uint8_t payload[DLLN33_MAX_PAYLOAD_LENGTH];
} DLLN33_Frame;

typedef struct {
	uint16_t address;
	uint16_t network_id;
	uint8_t channel;
} DLLN33_NetworkConfig;

typedef enum {
	DLLN33_NETWORK_SETUP_IDLE = 0,
	DLLN33_NETWORK_SETUP_SET_ADDRESS,
	DLLN33_NETWORK_SETUP_WAIT_ADDRESS,
	DLLN33_NETWORK_SETUP_SET_NETWORK_ID,
	DLLN33_NETWORK_SETUP_WAIT_NETWORK_ID,
	DLLN33_NETWORK_SETUP_SET_CHANNEL,
	DLLN33_NETWORK_SETUP_WAIT_CHANNEL,
	DLLN33_NETWORK_SETUP_SET_BAUD,
	DLLN33_NETWORK_SETUP_WAIT_BAUD,
	DLLN33_NETWORK_SETUP_RESTART,
	DLLN33_NETWORK_SETUP_WAIT_RESTART_TX,
	DLLN33_NETWORK_SETUP_COMPLETE,
	DLLN33_NETWORK_SETUP_FAILED,
} DLLN33_NetworkSetupState;

typedef struct {
	uint32_t received_frames;
	uint32_t dropped_received_frames;
	uint32_t frame_errors;
	uint32_t uart_errors;
	uint32_t dropped_tx_frames;
} DLLN33_Statistics;

/*
 * 在 SYSCFG_DL_init() 之后、打开 DL_LN33 UART2 NVIC 之前调用。
 * 初始化帧解析器、UART2 RX 中断和非阻塞发送队列。
 */
void DLLN33_Init(void);

/* 主循环调用：将发送队列灌入 UART2 FIFO，并推进异步网络初始化。 */
void DLLN33_Poll(void);

/* 在 DL_LN33_INST_IRQHandler 中调用，见 Drivers/board_isr.c。 */
void DLLN33_IRQHandler(void);

/*
 * 发送一帧用户数据。source_port 必须为 0x80-0xFF；地址在线上自动按小端序编码。
 * 返回 true 只表示帧完整写入本地发送队列，不表示无线对端已收到。
 */
bool DLLN33_Send(uint16_t remote_address, uint8_t source_port,
				 uint8_t destination_port, const uint8_t *payload,
				 uint8_t payload_length);

/* 从接收队列取出一帧。帧的 remote_address 是远端发送模块地址。 */
bool DLLN33_TryReceive(DLLN33_Frame *frame);

/* 发送常用内部端口报文。LED 时间单位为 100 ms。 */
bool DLLN33_LocateNode(uint16_t remote_address, uint8_t duration_100ms);

/* 读取当前模块的暂存/已保存配置；应答通过 DLLN33_TryReceive() 返回。 */
bool DLLN33_ReadAddress(void);
bool DLLN33_ReadNetworkId(void);
bool DLLN33_ReadChannel(void);
bool DLLN33_ReadBaudCode(void);

/*
 * 立即向本地管理端口排队写入配置。所有设置完成后必须调用
 * DLLN33_CommitAndRestart()，新的地址、网络 ID、信道和波特率才会持久生效。
 */
bool DLLN33_SetAddress(uint16_t address);
bool DLLN33_SetNetworkId(uint16_t network_id);
bool DLLN33_SetChannel(uint8_t channel);
bool DLLN33_SetBaud115200(void);
bool DLLN33_CommitAndRestart(void);

/*
 * 异步完成地址、网络 ID、信道和 115200 波特率的完整初始化。
 * 调用后持续调用 DLLN33_Poll()，直到状态为 COMPLETE 或 FAILED。
 * 配置端口必须发往本机地址 0x0000；该函数会在成功后让模块重启。
 */
bool DLLN33_BeginNetworkSetup(const DLLN33_NetworkConfig *config);
DLLN33_NetworkSetupState DLLN33_GetNetworkSetupState(void);
uint8_t DLLN33_GetNetworkSetupError(void);

/* 原子读取驱动统计信息。 */
void DLLN33_GetStatistics(DLLN33_Statistics *statistics);

#endif
