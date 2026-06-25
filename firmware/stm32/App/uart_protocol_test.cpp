#include "uart_protocol_test.h"

#include "usart.h"

static const uint8_t kFrameHead0 = 0xA5;
static const uint8_t kFrameHead1 = 0x5A;
static const uint8_t kMaxPayloadLen = 96;

enum RxState {
    kHead0,
    kHead1,
    kType,
    kLen,
    kSeqLo,
    kSeqHi,
    kPayload,
    kCrcLo,
    kCrcHi,
};

extern "C" {
volatile UartProtocolTestStats uart2_protocol_test_stats = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFFFFFFFFu, 0
};
}

namespace {

uint8_t g_rx_byte = 0;
RxState g_state = kHead0;
uint8_t g_frame_type = 0;
uint8_t g_payload_len = 0;
uint8_t g_payload_index = 0;
uint16_t g_seq = 0;
uint16_t g_rx_crc = 0;
uint8_t g_payload[kMaxPayloadLen] = {0};

uint16_t Crc16CcittUpdate(uint16_t crc, uint8_t data)
{
    crc ^= static_cast<uint16_t>(data) << 8;
    for (uint8_t i = 0; i < 8; ++i) {
        if ((crc & 0x8000u) != 0u) {
            crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
        } else {
            crc = static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

uint16_t CalculateFrameCrc()
{
    uint16_t crc = 0xFFFFu;
    crc = Crc16CcittUpdate(crc, g_frame_type);
    crc = Crc16CcittUpdate(crc, g_payload_len);
    crc = Crc16CcittUpdate(crc, static_cast<uint8_t>(g_seq & 0xFFu));
    crc = Crc16CcittUpdate(crc, static_cast<uint8_t>((g_seq >> 8) & 0xFFu));
    for (uint8_t i = 0; i < g_payload_len; ++i) {
        crc = Crc16CcittUpdate(crc, g_payload[i]);
    }
    return crc;
}

void ResetParser()
{
    g_state = kHead0;
    g_frame_type = 0;
    g_payload_len = 0;
    g_payload_index = 0;
    g_seq = 0;
    g_rx_crc = 0;
}

void MarkFrameOk()
{
    const uint32_t now = HAL_GetTick();
    if (uart2_protocol_test_stats.last_frame_tick_ms != 0u) {
        const uint32_t gap = now - uart2_protocol_test_stats.last_frame_tick_ms;
        if (gap < uart2_protocol_test_stats.min_frame_gap_ms) {
            uart2_protocol_test_stats.min_frame_gap_ms = gap;
        }
        if (gap > uart2_protocol_test_stats.max_frame_gap_ms) {
            uart2_protocol_test_stats.max_frame_gap_ms = gap;
        }
    }

    uart2_protocol_test_stats.frames_ok++;
    uart2_protocol_test_stats.last_seq = g_seq;
    uart2_protocol_test_stats.last_type = g_frame_type;
    uart2_protocol_test_stats.last_len = g_payload_len;
    uart2_protocol_test_stats.last_frame_tick_ms = now;
}

void ConsumeByte(uint8_t byte)
{
    uart2_protocol_test_stats.rx_bytes++;

    switch (g_state) {
    case kHead0:
        if (byte == kFrameHead0) {
            g_state = kHead1;
        } else {
            uart2_protocol_test_stats.sync_losses++;
        }
        break;

    case kHead1:
        if (byte == kFrameHead1) {
            g_state = kType;
        } else {
            uart2_protocol_test_stats.sync_losses++;
            g_state = (byte == kFrameHead0) ? kHead1 : kHead0;
        }
        break;

    case kType:
        g_frame_type = byte;
        g_state = kLen;
        break;

    case kLen:
        g_payload_len = byte;
        if (g_payload_len > kMaxPayloadLen) {
            uart2_protocol_test_stats.length_errors++;
            ResetParser();
        } else {
            g_payload_index = 0;
            g_state = kSeqLo;
        }
        break;

    case kSeqLo:
        g_seq = byte;
        g_state = kSeqHi;
        break;

    case kSeqHi:
        g_seq |= static_cast<uint16_t>(byte) << 8;
        g_state = (g_payload_len == 0u) ? kCrcLo : kPayload;
        break;

    case kPayload:
        g_payload[g_payload_index++] = byte;
        if (g_payload_index >= g_payload_len) {
            g_state = kCrcLo;
        }
        break;

    case kCrcLo:
        g_rx_crc = byte;
        g_state = kCrcHi;
        break;

    case kCrcHi:
        g_rx_crc |= static_cast<uint16_t>(byte) << 8;
        if (g_rx_crc == CalculateFrameCrc()) {
            MarkFrameOk();
        } else {
            uart2_protocol_test_stats.crc_errors++;
        }
        ResetParser();
        break;
    }
}

void RestartReceive()
{
    if (HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1) != HAL_OK) {
        uart2_protocol_test_stats.uart_errors++;
    }
}

}  // namespace

extern "C" void UartProtocolTest_Init(void)
{
    ResetParser();
    UartProtocolTest_ResetStats();
    RestartReceive();
}

extern "C" void UartProtocolTest_GetStats(UartProtocolTestStats *out)
{
    if (out == 0) {
        return;
    }

    __disable_irq();
    out->rx_bytes = uart2_protocol_test_stats.rx_bytes;
    out->frames_ok = uart2_protocol_test_stats.frames_ok;
    out->crc_errors = uart2_protocol_test_stats.crc_errors;
    out->length_errors = uart2_protocol_test_stats.length_errors;
    out->sync_losses = uart2_protocol_test_stats.sync_losses;
    out->uart_errors = uart2_protocol_test_stats.uart_errors;
    out->last_seq = uart2_protocol_test_stats.last_seq;
    out->last_type = uart2_protocol_test_stats.last_type;
    out->last_len = uart2_protocol_test_stats.last_len;
    out->last_frame_tick_ms = uart2_protocol_test_stats.last_frame_tick_ms;
    out->min_frame_gap_ms = uart2_protocol_test_stats.min_frame_gap_ms;
    out->max_frame_gap_ms = uart2_protocol_test_stats.max_frame_gap_ms;
    __enable_irq();
}

extern "C" void UartProtocolTest_ResetStats(void)
{
    __disable_irq();
    uart2_protocol_test_stats.rx_bytes = 0;
    uart2_protocol_test_stats.frames_ok = 0;
    uart2_protocol_test_stats.crc_errors = 0;
    uart2_protocol_test_stats.length_errors = 0;
    uart2_protocol_test_stats.sync_losses = 0;
    uart2_protocol_test_stats.uart_errors = 0;
    uart2_protocol_test_stats.last_seq = 0;
    uart2_protocol_test_stats.last_type = 0;
    uart2_protocol_test_stats.last_len = 0;
    uart2_protocol_test_stats.last_frame_tick_ms = 0;
    uart2_protocol_test_stats.min_frame_gap_ms = 0xFFFFFFFFu;
    uart2_protocol_test_stats.max_frame_gap_ms = 0;
    __enable_irq();
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        ConsumeByte(g_rx_byte);
        RestartReceive();
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uart2_protocol_test_stats.uart_errors++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        RestartReceive();
    }
}
