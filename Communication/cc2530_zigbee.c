/*
 * 文件名沿用工程历史构建清单；本文件实现的是 DL-LN33 / DL-LN3X 驱动，
 * 对外接口定义在 Communication/dl_ln33.h。
 */
#include "Communication/dl_ln33.h"

#include "BasicMicroLib/getTime.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

#define DLLN33_SOF                       0xFEU
#define DLLN33_EOF                       0xFFU
#define DLLN33_ESCAPE_FE                 0xFCU
#define DLLN33_ESCAPE_FF                 0xFDU
#define DLLN33_RX_QUEUE_DEPTH            4U
#define DLLN33_TX_BUFFER_SIZE            512U
#define DLLN33_MAX_WIRE_FRAME_LENGTH     129U
#define DLLN33_CONFIG_TIMEOUT_MS         300U
#define DLLN33_UART_FIFO_DRAIN_LIMIT     32U

typedef enum {
	DLLN33_RX_WAIT_SOF = 0,
	DLLN33_RX_WAIT_LENGTH,
	DLLN33_RX_DATA,
	DLLN33_RX_ESCAPE,
} DLLN33_RxState;

static volatile DLLN33_Frame s_rxQueue[DLLN33_RX_QUEUE_DEPTH];
static volatile uint8_t s_rxHead = 0U;
static volatile uint8_t s_rxTail = 0U;

static volatile uint8_t s_txBuffer[DLLN33_TX_BUFFER_SIZE];
static volatile uint16_t s_txHead = 0U;
static volatile uint16_t s_txTail = 0U;

static volatile DLLN33_RxState s_rxState = DLLN33_RX_WAIT_SOF;
static volatile uint8_t s_rxLogical[DLLN33_MAX_LOGICAL_LENGTH];
static volatile uint8_t s_rxLength = 0U;
static volatile uint8_t s_rxCount = 0U;

static volatile DLLN33_NetworkSetupState s_setupState =
	DLLN33_NETWORK_SETUP_IDLE;
static DLLN33_NetworkConfig s_setupConfig;
static volatile uint32_t s_setupDeadlineMs = 0U;
static volatile bool s_setupResponseReady = false;
static volatile uint8_t s_setupResponse = 0U;
static volatile uint8_t s_setupError = DLLN33_MANAGEMENT_OK;

static volatile DLLN33_Statistics s_statistics;

static void DLLN33_RestoreIrq(uint32_t primask)
{
	if (primask == 0U) {
		__enable_irq();
	}
}

static uint16_t DLLN33_TxFreeBytes(void)
{
	if (s_txTail > s_txHead) {
		return (uint16_t)(s_txTail - s_txHead - 1U);
	}
	return (uint16_t)(DLLN33_TX_BUFFER_SIZE - s_txHead + s_txTail - 1U);
}

static bool DLLN33_EnqueueWireFrame(const uint8_t *wire, uint16_t length)
{
	uint32_t primask;
	uint16_t i;

	primask = __get_PRIMASK();
	__disable_irq();
	if ((wire == NULL) || (length == 0U) || (length > DLLN33_TxFreeBytes())) {
		s_statistics.dropped_tx_frames++;
		DLLN33_RestoreIrq(primask);
		return false;
	}

	for (i = 0U; i < length; i++) {
		s_txBuffer[s_txHead] = wire[i];
		s_txHead = (uint16_t)((s_txHead + 1U) % DLLN33_TX_BUFFER_SIZE);
	}
	DLLN33_RestoreIrq(primask);
	return true;
}

static void DLLN33_FlushTx(void)
{
	uint32_t primask;
	uint8_t data;

	while (!DL_UART_Main_isTXFIFOFull(Zigbee_INST)) {
		primask = __get_PRIMASK();
		__disable_irq();
		if (s_txTail == s_txHead) {
			DLLN33_RestoreIrq(primask);
			break;
		}
		data = s_txBuffer[s_txTail];
		s_txTail = (uint16_t)((s_txTail + 1U) % DLLN33_TX_BUFFER_SIZE);
		DLLN33_RestoreIrq(primask);
		DL_UART_Main_transmitData(Zigbee_INST, data);
	}
}

static bool DLLN33_IsTxDrained(void)
{
	uint32_t primask;
	bool empty;

	primask = __get_PRIMASK();
	__disable_irq();
	empty = (s_txTail == s_txHead);
	DLLN33_RestoreIrq(primask);

	return empty && !DL_UART_Main_isBusy(Zigbee_INST);
}

static void DLLN33_ResetParser(void)
{
	s_rxState = DLLN33_RX_WAIT_SOF;
	s_rxLength = 0U;
	s_rxCount = 0U;
}

static void DLLN33_CaptureManagementResponse(const DLLN33_Frame *frame)
{
	if ((frame->source_port == DLLN33_PORT_MANAGEMENT) &&
		(frame->destination_port == DLLN33_CONFIG_SOURCE_PORT) &&
		(frame->remote_address == 0U) && (frame->payload_length == 1U) &&
		(s_setupState >= DLLN33_NETWORK_SETUP_WAIT_ADDRESS) &&
		(s_setupState <= DLLN33_NETWORK_SETUP_WAIT_BAUD)) {
		s_setupResponse = frame->payload[0];
		s_setupResponseReady = true;
	}
}

static void DLLN33_PublishFrameFromIsr(void)
{
	uint8_t nextHead;
	uint8_t i;
	DLLN33_Frame frame;

	if (s_rxLength < 4U) {
		s_statistics.frame_errors++;
		return;
	}

	frame.source_port = s_rxLogical[0];
	frame.destination_port = s_rxLogical[1];
	frame.remote_address = (uint16_t)s_rxLogical[2] |
					   ((uint16_t)s_rxLogical[3] << 8U);
	frame.payload_length = (uint8_t)(s_rxLength - 4U);
	for (i = 0U; i < frame.payload_length; i++) {
		frame.payload[i] = s_rxLogical[4U + i];
	}

	DLLN33_CaptureManagementResponse(&frame);
	nextHead = (uint8_t)((s_rxHead + 1U) % DLLN33_RX_QUEUE_DEPTH);
	if (nextHead == s_rxTail) {
		s_statistics.dropped_received_frames++;
		return;
	}

	s_rxQueue[s_rxHead] = frame;
	s_rxHead = nextHead;
	s_statistics.received_frames++;
}

static void DLLN33_AppendLogicalByte(uint8_t data)
{
	if (s_rxCount >= s_rxLength) {
		s_statistics.frame_errors++;
		DLLN33_ResetParser();
		return;
	}

	s_rxLogical[s_rxCount] = data;
	s_rxCount++;
	s_rxState = DLLN33_RX_DATA;
}

static void DLLN33_ProcessRxByte(uint8_t data)
{
	switch (s_rxState) {
	case DLLN33_RX_WAIT_SOF:
		if (data == DLLN33_SOF) {
			s_rxState = DLLN33_RX_WAIT_LENGTH;
		}
		break;

	case DLLN33_RX_WAIT_LENGTH:
		if ((data >= 4U) && (data <= DLLN33_MAX_LOGICAL_LENGTH)) {
			s_rxLength = data;
			s_rxCount = 0U;
			s_rxState = DLLN33_RX_DATA;
		} else if (data != DLLN33_SOF) {
			s_statistics.frame_errors++;
			DLLN33_ResetParser();
		}
		break;

	case DLLN33_RX_DATA:
		if (data == DLLN33_SOF) {
			s_rxState = DLLN33_RX_ESCAPE;
		} else if (data == DLLN33_EOF) {
			if (s_rxCount == s_rxLength) {
				DLLN33_PublishFrameFromIsr();
			} else {
				s_statistics.frame_errors++;
			}
			DLLN33_ResetParser();
		} else {
			DLLN33_AppendLogicalByte(data);
		}
		break;

	case DLLN33_RX_ESCAPE:
		if (data == DLLN33_ESCAPE_FE) {
			DLLN33_AppendLogicalByte(DLLN33_SOF);
		} else if (data == DLLN33_ESCAPE_FF) {
			DLLN33_AppendLogicalByte(DLLN33_EOF);
		} else if ((data >= 4U) && (data <= DLLN33_MAX_LOGICAL_LENGTH)) {
			/*
			 * 按模块协议，FE 后接小于 0x40 的值是新包长度。将其
			 * 视为新帧可在旧帧残缺时立即重新同步。
			 */
			s_rxLength = data;
			s_rxCount = 0U;
			s_rxState = DLLN33_RX_DATA;
		} else {
			s_statistics.frame_errors++;
			DLLN33_ResetParser();
		}
		break;

	default:
		DLLN33_ResetParser();
		break;
	}
}

static void DLLN33_DrainRxFifo(void)
{
	uint32_t count = 0U;

	while (!DL_UART_Main_isRXFIFOEmpty(Zigbee_INST) &&
		   (count < DLLN33_UART_FIFO_DRAIN_LIMIT)) {
		DLLN33_ProcessRxByte(DL_UART_Main_receiveData(Zigbee_INST));
		count++;
	}
}

static void DLLN33_FlushRxFifo(void)
{
	uint32_t count = 0U;

	while (!DL_UART_Main_isRXFIFOEmpty(Zigbee_INST) &&
		   (count < DLLN33_UART_FIFO_DRAIN_LIMIT)) {
		(void)DL_UART_Main_receiveData(Zigbee_INST);
		count++;
	}
}

static bool DLLN33_IsValidNodeId(uint16_t value)
{
	return (value != 0U) && (value != 0xFFFFU);
}

static bool DLLN33_IsValidChannel(uint8_t channel)
{
	return (channel >= 0x0BU) && (channel <= 0x1AU);
}

static bool DLLN33_SendManagement(uint8_t command, const uint8_t *data,
						  uint8_t dataLength)
{
	uint8_t payload[3U];
	uint8_t i;

	if (dataLength > 2U) {
		return false;
	}
	payload[0] = command;
	for (i = 0U; i < dataLength; i++) {
		payload[1U + i] = data[i];
	}
	return DLLN33_Send(0U, DLLN33_CONFIG_SOURCE_PORT,
					DLLN33_PORT_MANAGEMENT, payload, (uint8_t)(dataLength + 1U));
}

static bool DLLN33_GetSetupResponse(uint8_t *response)
{
	uint32_t primask;
	bool ready;

	primask = __get_PRIMASK();
	__disable_irq();
	ready = s_setupResponseReady;
	if (ready) {
		*response = s_setupResponse;
		s_setupResponseReady = false;
	}
	DLLN33_RestoreIrq(primask);
	return ready;
}

static void DLLN33_AdvanceSetupAfterSuccess(void)
{
	switch (s_setupState) {
	case DLLN33_NETWORK_SETUP_WAIT_ADDRESS:
		s_setupState = DLLN33_NETWORK_SETUP_SET_NETWORK_ID;
		break;
	case DLLN33_NETWORK_SETUP_WAIT_NETWORK_ID:
		s_setupState = DLLN33_NETWORK_SETUP_SET_CHANNEL;
		break;
	case DLLN33_NETWORK_SETUP_WAIT_CHANNEL:
		s_setupState = DLLN33_NETWORK_SETUP_SET_BAUD;
		break;
	case DLLN33_NETWORK_SETUP_WAIT_BAUD:
		s_setupState = DLLN33_NETWORK_SETUP_RESTART;
		break;
	default:
		break;
	}
}

static void DLLN33_ProcessNetworkSetup(uint32_t nowMs)
{
	uint8_t data[2U];
	uint8_t response;
	bool sent = false;

	switch (s_setupState) {
	case DLLN33_NETWORK_SETUP_SET_ADDRESS:
		data[0] = (uint8_t)(s_setupConfig.address & 0xFFU);
		data[1] = (uint8_t)(s_setupConfig.address >> 8U);
		sent = DLLN33_SendManagement(DLLN33_CMD_SET_ADDRESS, data, 2U);
		if (sent) {
			s_setupState = DLLN33_NETWORK_SETUP_WAIT_ADDRESS;
			s_setupDeadlineMs = nowMs + DLLN33_CONFIG_TIMEOUT_MS;
		}
		break;

	case DLLN33_NETWORK_SETUP_SET_NETWORK_ID:
		data[0] = (uint8_t)(s_setupConfig.network_id & 0xFFU);
		data[1] = (uint8_t)(s_setupConfig.network_id >> 8U);
		sent = DLLN33_SendManagement(DLLN33_CMD_SET_NETWORK_ID, data, 2U);
		if (sent) {
			s_setupState = DLLN33_NETWORK_SETUP_WAIT_NETWORK_ID;
			s_setupDeadlineMs = nowMs + DLLN33_CONFIG_TIMEOUT_MS;
		}
		break;

	case DLLN33_NETWORK_SETUP_SET_CHANNEL:
		data[0] = s_setupConfig.channel;
		sent = DLLN33_SendManagement(DLLN33_CMD_SET_CHANNEL, data, 1U);
		if (sent) {
			s_setupState = DLLN33_NETWORK_SETUP_WAIT_CHANNEL;
			s_setupDeadlineMs = nowMs + DLLN33_CONFIG_TIMEOUT_MS;
		}
		break;

	case DLLN33_NETWORK_SETUP_SET_BAUD:
		data[0] = DLLN33_DEFAULT_BAUD_CODE;
		sent = DLLN33_SendManagement(DLLN33_CMD_SET_BAUD, data, 1U);
		if (sent) {
			s_setupState = DLLN33_NETWORK_SETUP_WAIT_BAUD;
			s_setupDeadlineMs = nowMs + DLLN33_CONFIG_TIMEOUT_MS;
		}
		break;

	case DLLN33_NETWORK_SETUP_WAIT_ADDRESS:
	case DLLN33_NETWORK_SETUP_WAIT_NETWORK_ID:
	case DLLN33_NETWORK_SETUP_WAIT_CHANNEL:
	case DLLN33_NETWORK_SETUP_WAIT_BAUD:
		if (DLLN33_GetSetupResponse(&response)) {
			if (response == DLLN33_MANAGEMENT_OK) {
				DLLN33_AdvanceSetupAfterSuccess();
			} else {
				s_setupError = response;
				s_setupState = DLLN33_NETWORK_SETUP_FAILED;
			}
		} else if ((int32_t)(nowMs - s_setupDeadlineMs) >= 0) {
			s_setupError = 0xFFU; /* 本地超时，非模块协议状态码。 */
			s_setupState = DLLN33_NETWORK_SETUP_FAILED;
		}
		break;

	case DLLN33_NETWORK_SETUP_RESTART:
		if (DLLN33_CommitAndRestart()) {
			s_setupState = DLLN33_NETWORK_SETUP_WAIT_RESTART_TX;
		}
		break;

	case DLLN33_NETWORK_SETUP_WAIT_RESTART_TX:
		if (DLLN33_IsTxDrained()) {
			s_setupState = DLLN33_NETWORK_SETUP_COMPLETE;
		}
		break;

	default:
		break;
	}
}

void DLLN33_Init(void)
{
	uint32_t primask;

	primask = __get_PRIMASK();
	__disable_irq();
	s_rxHead = 0U;
	s_rxTail = 0U;
	s_txHead = 0U;
	s_txTail = 0U;
	DLLN33_ResetParser();
	s_setupState = DLLN33_NETWORK_SETUP_IDLE;
	s_setupResponseReady = false;
	s_setupError = DLLN33_MANAGEMENT_OK;
	s_statistics.received_frames = 0U;
	s_statistics.dropped_received_frames = 0U;
	s_statistics.frame_errors = 0U;
	s_statistics.uart_errors = 0U;
	s_statistics.dropped_tx_frames = 0U;
	DLLN33_RestoreIrq(primask);

	/* SysConfig 已初始化 UART2 为 PB15/PB16、115200、8N1；这里配置运行时中断。 */
	DL_UART_Main_enableFIFOs(Zigbee_INST);
	DL_UART_Main_setRXFIFOThreshold(Zigbee_INST,
								DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
	DL_UART_Main_setTXFIFOThreshold(Zigbee_INST,
								DL_UART_MAIN_TX_FIFO_LEVEL_ONE_ENTRY);
	DL_UART_Main_setRXInterruptTimeout(Zigbee_INST, 15U);
	DL_UART_Main_clearInterruptStatus(Zigbee_INST,
			DL_UART_MAIN_INTERRUPT_RX |
			DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
			DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
			DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
	DLLN33_FlushRxFifo();
	DL_UART_Main_enableInterrupt(Zigbee_INST,
			DL_UART_MAIN_INTERRUPT_RX |
			DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
			DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
			DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
	DL_UART_Main_enable(Zigbee_INST);
}

void DLLN33_Poll(void)
{
	DLLN33_FlushTx();
	DLLN33_ProcessNetworkSetup(getNowMs());
	/* 配置状态机可能刚排入一个管理帧，立即再灌一次 FIFO 降低等待时间。 */
	DLLN33_FlushTx();
}

void DLLN33_IRQHandler(void)
{
	DL_UART_IIDX pending;

	do {
		pending = DL_UART_Main_getPendingInterrupt(Zigbee_INST);
		switch (pending) {
		case DL_UART_MAIN_IIDX_RX:
		case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
			DLLN33_DrainRxFifo();
			break;

		case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
		case DL_UART_MAIN_IIDX_FRAMING_ERROR:
			s_statistics.uart_errors++;
			DLLN33_FlushRxFifo();
			DLLN33_ResetParser();
			break;

		default:
			break;
		}
	} while (pending != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}

bool DLLN33_Send(uint16_t remote_address, uint8_t source_port,
				 uint8_t destination_port, const uint8_t *payload,
				 uint8_t payload_length)
{
	uint8_t logical[DLLN33_MAX_LOGICAL_LENGTH];
	uint8_t wire[DLLN33_MAX_WIRE_FRAME_LENGTH];
	uint8_t logicalLength;
	uint8_t i;
	uint16_t wireLength = 0U;

	if ((source_port < DLLN33_APPLICATION_PORT_MIN) ||
		(payload_length > DLLN33_MAX_PAYLOAD_LENGTH) ||
		((payload_length > 0U) && (payload == NULL))) {
		return false;
	}

	logicalLength = (uint8_t)(4U + payload_length);
	logical[0] = source_port;
	logical[1] = destination_port;
	logical[2] = (uint8_t)(remote_address & 0xFFU);
	logical[3] = (uint8_t)(remote_address >> 8U);
	for (i = 0U; i < payload_length; i++) {
		logical[4U + i] = payload[i];
	}

	wire[wireLength++] = DLLN33_SOF;
	wire[wireLength++] = logicalLength;
	for (i = 0U; i < logicalLength; i++) {
		if (logical[i] == DLLN33_SOF) {
			wire[wireLength++] = DLLN33_SOF;
			wire[wireLength++] = DLLN33_ESCAPE_FE;
		} else if (logical[i] == DLLN33_EOF) {
			wire[wireLength++] = DLLN33_SOF;
			wire[wireLength++] = DLLN33_ESCAPE_FF;
		} else {
			wire[wireLength++] = logical[i];
		}
	}
	wire[wireLength++] = DLLN33_EOF;

	return DLLN33_EnqueueWireFrame(wire, wireLength);
}

bool DLLN33_TryReceive(DLLN33_Frame *frame)
{
	uint32_t primask;

	if (frame == NULL) {
		return false;
	}

	primask = __get_PRIMASK();
	__disable_irq();
	if (s_rxTail == s_rxHead) {
		DLLN33_RestoreIrq(primask);
		return false;
	}
	*frame = s_rxQueue[s_rxTail];
	s_rxTail = (uint8_t)((s_rxTail + 1U) % DLLN33_RX_QUEUE_DEPTH);
	DLLN33_RestoreIrq(primask);
	return true;
}

bool DLLN33_LocateNode(uint16_t remote_address, uint8_t duration_100ms)
{
	return DLLN33_Send(remote_address, DLLN33_CONFIG_SOURCE_PORT,
					DLLN33_PORT_LED, &duration_100ms, 1U);
}

bool DLLN33_ReadAddress(void)
{
	return DLLN33_SendManagement(DLLN33_CMD_READ_ADDRESS, NULL, 0U);
}

bool DLLN33_ReadNetworkId(void)
{
	return DLLN33_SendManagement(DLLN33_CMD_READ_NETWORK_ID, NULL, 0U);
}

bool DLLN33_ReadChannel(void)
{
	return DLLN33_SendManagement(DLLN33_CMD_READ_CHANNEL, NULL, 0U);
}

bool DLLN33_ReadBaudCode(void)
{
	return DLLN33_SendManagement(DLLN33_CMD_READ_BAUD, NULL, 0U);
}

bool DLLN33_SetAddress(uint16_t address)
{
	uint8_t value[2U];

	if (!DLLN33_IsValidNodeId(address)) {
		return false;
	}
	value[0] = (uint8_t)(address & 0xFFU);
	value[1] = (uint8_t)(address >> 8U);
	return DLLN33_SendManagement(DLLN33_CMD_SET_ADDRESS, value, 2U);
}

bool DLLN33_SetNetworkId(uint16_t network_id)
{
	uint8_t value[2U];

	if (!DLLN33_IsValidNodeId(network_id)) {
		return false;
	}
	value[0] = (uint8_t)(network_id & 0xFFU);
	value[1] = (uint8_t)(network_id >> 8U);
	return DLLN33_SendManagement(DLLN33_CMD_SET_NETWORK_ID, value, 2U);
}

bool DLLN33_SetChannel(uint8_t channel)
{
	if (!DLLN33_IsValidChannel(channel)) {
		return false;
	}
	return DLLN33_SendManagement(DLLN33_CMD_SET_CHANNEL, &channel, 1U);
}

bool DLLN33_SetBaud115200(void)
{
	uint8_t baudCode = DLLN33_DEFAULT_BAUD_CODE;
	return DLLN33_SendManagement(DLLN33_CMD_SET_BAUD, &baudCode, 1U);
}

bool DLLN33_CommitAndRestart(void)
{
	return DLLN33_SendManagement(DLLN33_CMD_RESTART, NULL, 0U);
}

bool DLLN33_BeginNetworkSetup(const DLLN33_NetworkConfig *config)
{
	uint32_t primask;

	if ((config == NULL) || !DLLN33_IsValidNodeId(config->address) ||
		!DLLN33_IsValidNodeId(config->network_id) ||
		!DLLN33_IsValidChannel(config->channel)) {
		return false;
	}

	primask = __get_PRIMASK();
	__disable_irq();
	if ((s_setupState != DLLN33_NETWORK_SETUP_IDLE) &&
		(s_setupState != DLLN33_NETWORK_SETUP_COMPLETE) &&
		(s_setupState != DLLN33_NETWORK_SETUP_FAILED)) {
		DLLN33_RestoreIrq(primask);
		return false;
	}
	s_setupConfig = *config;
	s_setupResponseReady = false;
	s_setupError = DLLN33_MANAGEMENT_OK;
	s_setupState = DLLN33_NETWORK_SETUP_SET_ADDRESS;
	DLLN33_RestoreIrq(primask);
	return true;
}

DLLN33_NetworkSetupState DLLN33_GetNetworkSetupState(void)
{
	return s_setupState;
}

uint8_t DLLN33_GetNetworkSetupError(void)
{
	return s_setupError;
}

void DLLN33_GetStatistics(DLLN33_Statistics *statistics)
{
	uint32_t primask;

	if (statistics == NULL) {
		return;
	}
	primask = __get_PRIMASK();
	__disable_irq();
	*statistics = s_statistics;
	DLLN33_RestoreIrq(primask);
}
