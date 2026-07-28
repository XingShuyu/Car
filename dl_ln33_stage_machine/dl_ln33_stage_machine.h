/* Project-root DL-LN33 message-driven stage machine. */
#ifndef DL_LN33_STAGE_MACHINE_H
#define DL_LN33_STAGE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

#include "Communication/dl_ln33.h"

/*
 * 一条 DL-LN33 应用层消息。
 *
 * 用作 enter_message 时，remote_address 是目标车地址，source_port 是本车
 * 发送端口，destination_port 是目标车接收端口。
 * 用作 finish_message 时，remote_address 是期望发送方地址，source_port 和
 * destination_port 分别匹配接收帧的源端口和目标端口。
 */
typedef struct StageMessage {
	uint16_t remote_address;
	uint8_t source_port;
	uint8_t destination_port;
	const uint8_t *payload;
	uint8_t payload_length;
} StageMessage;

/* finish_message.remote_address 使用该值时接受任意发送方。 */
#define STAGE_MESSAGE_ANY_REMOTE_ADDRESS UINT16_MAX

/*
 * 收到当前阶段的非结束消息时调用。回调在主循环上下文执行，不在 UART ISR
 * 中执行，因此可安全更新业务状态，但应保持非阻塞和快速返回。
 */
typedef void (*StageNonFinishMessageHandler)(const DLLN33_Frame *message,
										  void *user_context);

/*
 * 无线阶段定义。该结构体严格只描述阶段协议：
 *   1. enter_message：进入阶段后发送的消息；NULL 表示本阶段无需发送。
 *   2. finish_message：收到该消息后自动切换到下一阶段。
 *   3. non_finish_message_handler：收到其他消息时的处理函数；可为 NULL。
 *
 * 保留 struct tag 的写法是为了与现有 typedef Stage（循迹命令枚举）共存。
 */
struct Stage {
	const StageMessage *enter_message;
	const StageMessage *finish_message;
	StageNonFinishMessageHandler non_finish_message_handler;
};

typedef enum StageMachineStatus {
	StageMachineStatus_Uninitialized = 0,
	StageMachineStatus_Entering,
	StageMachineStatus_WaitingForFinish,
	StageMachineStatus_Complete,
	StageMachineStatus_InvalidConfiguration,
} StageMachineStatus;

typedef struct StageMachine {
	const struct Stage *stages;
	uint16_t stage_count;
	uint16_t stage_index;
	void *user_context;
	StageMachineStatus status;
} StageMachine;

/*
 * 初始化新的多车无线阶段机。stages 和每个阶段的 finish_message 必须有效。
 * 消息内容由调用方静态保存，直到阶段机完成。
 */
bool StageMachine_Init(StageMachine *machine, const struct Stage *stages,
				   uint16_t stage_count, void *user_context);

/*
 * 主循环调用。进入阶段时通过 DLLN33_Send() 排队发送 enter_message；发送队列
 * 暂满时会在下一次调用重试。随后从 DLLN33 接收队列中处理消息。
 */
void StageMachine_Update(StageMachine *machine);

StageMachineStatus StageMachine_GetStatus(const StageMachine *machine);
uint16_t StageMachine_GetStageIndex(const StageMachine *machine);

#endif
