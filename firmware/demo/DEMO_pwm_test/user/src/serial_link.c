#include "serial_link.h"
#include "app_runtime.h"
#include "state_machine.h"
#include "fsl_dmamux.h"
#include "fsl_lpuart_edma.h"
#include <string.h>

#define TX_SLOTS 8U
#define TX_SLOT_BYTES 192U  /* Whole 32-byte cache lines; no unrelated data shares a line. */
#define TX_DMA_CHANNEL 8U
#define RX_ACK_SLOTS 8U

enum
{
    RX_ACK_START = 1,
    RX_ACK_DEBUG,
    RX_ACK_STOP,
    RX_ACK_CALIBRATE
};

/* The project's scatter file places *OCRAM_CACHE at 0x20200000. A stack
 * buffer must never be passed directly to eDMA. Clean each slot before TX. */
AT_OCRAM_SECTION_ALIGN(static uint8 s_tx_data[TX_SLOTS][TX_SLOT_BYTES], 32);
static uint16 s_tx_length[TX_SLOTS];
static volatile uint8 s_tx_head, s_tx_tail, s_tx_count, s_tx_busy;
static volatile uint8 s_tx_fault;
static uint32 s_tx_start_ms;
static edma_handle_t s_dma_handle;
static lpuart_edma_handle_t s_uart_handle;
static char s_command[16];
static uint8 s_command_length, s_command_discard;
/* UART ISR is the sole producer; foreground serial_link_service is the sole
 * consumer. One slot is left empty to distinguish full from empty. */
static volatile uint8 s_rx_ack[RX_ACK_SLOTS];
static volatile uint8 s_rx_ack_head, s_rx_ack_tail;
volatile uint32 serial_tx_dropped, serial_tx_errors, serial_rx_errors;

static void queue_rx_ack(uint8 command)
{
    uint8 next = (uint8)((s_rx_ack_head + 1U) % RX_ACK_SLOTS);
    if (next != s_rx_ack_tail)
    {
        s_rx_ack[s_rx_ack_head] = command;
        __DMB();
        s_rx_ack_head = next;
    }
}

static const char *rx_ack_line(uint8 command)
{
    switch (command)
    {
        case RX_ACK_START: return "RX,START,RECEIVED\r\n";
        case RX_ACK_DEBUG: return "RX,DEBUG,RECEIVED\r\n";
        case RX_ACK_STOP: return "RX,STOP,RECEIVED\r\n";
        case RX_ACK_CALIBRATE: return "RX,CALIBRATE,RECEIVED\r\n";
        default: return NULL;
    }
}

static void tx_done(LPUART_Type *base, lpuart_edma_handle_t *handle,
                    status_t status, void *user_data)
{
    (void)base; (void)handle; (void)user_data;
    if (status == kStatus_LPUART_TxIdle && s_tx_busy && !s_tx_fault)
    {
        s_tx_head = (uint8)((s_tx_head + 1U) % TX_SLOTS);
        --s_tx_count;
        s_tx_busy = 0;
    }
}

void serial_link_init(void)
{
    edma_config_t config;
    s_tx_head = s_tx_tail = s_tx_count = s_tx_busy = s_tx_fault = 0;
    s_command_length = s_command_discard = 0;
    s_rx_ack_head = s_rx_ack_tail = 0;
    serial_tx_dropped = serial_tx_errors = serial_rx_errors = 0;
    /* Application owns controller initialization. Do not reinitialize DMA
     * from another peripheral later; reserve channel 8 for UART1 TX. */
    DMAMUX_Init(DMAMUX);
    EDMA_GetDefaultConfig(&config);
    /* Handle our channel error explicitly; a global HALT would otherwise keep
     * ignoring requests even after its error flag has been cleared. */
    config.enableHaltOnError = false;
    EDMA_Init(DMA0, &config);
    DMAMUX_SetSource(DMAMUX, TX_DMA_CHANNEL, kDmaRequestMuxLPUART1Tx);
    DMAMUX_EnableChannel(DMAMUX, TX_DMA_CHANNEL);
    EDMA_CreateHandle(&s_dma_handle, DMA0, TX_DMA_CHANNEL);
    LPUART_TransferCreateHandleEDMA(LPUART1, &s_uart_handle, tx_done, NULL,
                                  &s_dma_handle, NULL);
    EDMA_EnableChannelInterrupts(DMA0, TX_DMA_CHANNEL, kEDMA_ErrorInterruptEnable);
    NVIC_SetPriority(DMA8_DMA24_IRQn, IRQ_PRIORITY_DMA);
    NVIC_SetPriority(DMA_ERROR_IRQn, IRQ_PRIORITY_DMA);
    NVIC_ClearPendingIRQ(DMA_ERROR_IRQn);
    NVIC_EnableIRQ(DMA_ERROR_IRQn);
    NVIC_SetPriority(LPUART1_IRQn, IRQ_PRIORITY_COMMAND);
    /* CreateHandleEDMA disables UART interrupts: RX must be enabled AFTER it. */
    uart_rx_interrupt(UART_1, 1);
}

uint8 serial_link_send(const void *data, uint32 length)
{
    uint8 slot;
    uint32 mask;
    if (!length || length > TX_SLOT_BYTES || s_tx_fault) return 0;
    if (s_tx_count >= TX_SLOTS)
    {
        ++serial_tx_dropped;
        return 0;
    }
    slot = s_tx_tail; /* Only foreground produces; ISR can only free slots. */
    memcpy(s_tx_data[slot], data, length);
    s_tx_length[slot] = (uint16)length;
    SCB_CleanDCache_by_Addr((uint32_t *)s_tx_data[slot], (int32)((length + 31U) & ~31U));
    __DMB();
    mask = __get_PRIMASK();
    __disable_irq();
    s_tx_tail = (uint8)((slot + 1U) % TX_SLOTS);
    ++s_tx_count; /* Publish only after the complete line and cache clean. */
    __set_PRIMASK(mask);
    return 1;
}

/* CRC-16/CCITT-FALSE, nibble lookup: bounded cost, no printf or heap. */
static uint16 frame_crc(const uint8 *p, uint32 n)
{
    static const uint16 lut[16] = {0x0000,0x1021,0x2042,0x3063,
        0x4084,0x50A5,0x60C6,0x70E7,0x8108,0x9129,0xA14A,0xB16B,
        0xC18C,0xD1AD,0xE1CE,0xF1EF};
    uint16 crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16)*p++ << 8;
        crc = (uint16)((crc << 4) ^ lut[crc >> 12]);
        crc = (uint16)((crc << 4) ^ lut[crc >> 12]);
    }
    return crc;
}

uint8 serial_link_frame(uint8 type, const uint8 *payload, uint32 length)
{
    uint8 raw[188], encoded[192];
    uint32 i, out = 1, code_at = 0;
    uint8 code = 1;
    uint16 crc;
    if (length > 185U) return 0;
    raw[0] = (uint8)(0x10U | (type & 15U));
    memcpy(raw + 1, payload, length);
    crc = frame_crc(raw, length + 1);
    raw[length + 1] = (uint8)crc;
    raw[length + 2] = (uint8)(crc >> 8);
    for (i = 0; i < length + 3; ++i) {
        if (raw[i]) { encoded[out++] = raw[i]; ++code; }
        else { encoded[code_at] = code; code_at = out++; code = 1; }
    }
    encoded[code_at] = code;
    encoded[out++] = 0;
    return serial_link_send(encoded, out);
}

uint8 serial_link_send_text(const char *text)
{
    return serial_link_frame(2, (const uint8 *)text, (uint32)strlen(text));
}

uint8 serial_link_can_report(void)
{
    /* Leave slots for command ACKs and avoid a backlog of old telemetry. */
    return !s_tx_fault && s_tx_count < 2U;
}

void serial_link_service(void)
{
    lpuart_transfer_t transfer;
    uint32 mask;
    uint8 ack_tail = s_rx_ack_tail;
    if (ack_tail != s_rx_ack_head)
    {
        const char *line = rx_ack_line(s_rx_ack[ack_tail]);
        if (line && serial_link_send_text(line))
            s_rx_ack_tail = (uint8)((ack_tail + 1U) % RX_ACK_SLOTS);
    }
    if (s_tx_busy && app_runtime_now_ms() - s_tx_start_ms >= UART_TX_TIMEOUT_MS)
        s_tx_fault = 1;
    if (s_tx_fault)
    {
        mask = __get_PRIMASK();
        __disable_irq();
        LPUART_TransferAbortSendEDMA(LPUART1, &s_uart_handle);
        LPUART_DisableInterrupts(LPUART1, kLPUART_TransmissionCompleteInterruptEnable);
        EDMA_ClearChannelStatusFlags(DMA0, TX_DMA_CHANNEL, kEDMA_ErrorFlag | kEDMA_InterruptFlag);
        s_tx_head = s_tx_tail = s_tx_count = s_tx_busy = 0;
        /* Discard a possibly truncated line, then give the parser a terminator. */
        LPUART1->FIFO |= LPUART_FIFO_TXFLUSH_MASK;
        s_tx_fault = 0;
        __set_PRIMASK(mask);
        ++serial_tx_errors;
        if (g_sm.state == ST_RUN)
            state_machine_request_fault(SM_ERR_UART_DMA, state_machine_epoch());
        (void)serial_link_send_text("\r\nLINK,TX_DMA_ERROR\r\n");
    }
    if (s_tx_busy || !s_tx_count) return;
    transfer.data = s_tx_data[s_tx_head];
    transfer.dataSize = s_tx_length[s_tx_head];
    s_tx_start_ms = app_runtime_now_ms();
    s_tx_busy = 1;
    if (LPUART_SendEDMA(LPUART1, &s_uart_handle, &transfer) != kStatus_Success)
        s_tx_fault = 1;
}

static void parse_byte(uint8 byte)
{
    if (byte == '\r' || byte == '\n')
    {
        if (!s_command_discard)
        {
            if (s_command_length == 5 && memcmp(s_command, "START", 5) == 0)
            {
                queue_rx_ack(RX_ACK_START);
                state_machine_request_start();
            }
            else if (s_command_length == 5 && memcmp(s_command, "DEBUG", 5) == 0)
            {
                queue_rx_ack(RX_ACK_DEBUG);
                state_machine_request_debug();
            }
            else if (s_command_length == 4 && memcmp(s_command, "STOP", 4) == 0)
            {
                queue_rx_ack(RX_ACK_STOP);
                state_machine_request_stop();
            }
            else if (s_command_length == 9 && memcmp(s_command, "CALIBRATE", 9) == 0)
            {
                queue_rx_ack(RX_ACK_CALIBRATE);
                state_machine_request_calibration();
            }
            else if ((s_command_length == 9 || s_command_length == 10) &&
                     memcmp(s_command, "JOY,", 4) == 0 &&
                     (s_command[4] == '0' || s_command[4] == '1') && s_command[5] == ',' &&
                     (s_command[6] == '0' || s_command[6] == '1') && s_command[7] == ',')
            {
                int8 steering;
                if (s_command_length == 9 && (s_command[8] == '0' || s_command[8] == '1'))
                    steering = (int8)(s_command[8] - '0');
                else if (s_command_length == 10 && s_command[8] == '-' && s_command[9] == '1')
                    steering = -1;
                else steering = 2; /* Invalid token; ignore it. */
                if (steering >= -1 && steering <= 1)
                    state_machine_request_joystick((uint8)(s_command[4] - '0'),
                                                   (uint8)(s_command[6] - '0'), steering);
            }
        }
        s_command_length = s_command_discard = 0;
    }
    else if (!s_command_discard)
    {
        if (byte >= 'a' && byte <= 'z') byte = (uint8)(byte - 'a' + 'A');
        if (s_command_length < sizeof(s_command))
            s_command[s_command_length++] = (char)byte;
        else s_command_discard = 1;
    }
}

void serial_link_uart_irq(void)
{
    uint8 byte;
    uint32 flags = LPUART_GetStatusFlags(LPUART1);
    uint32 errors = flags & (kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag |
                            kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag);
    if (errors)
    {
        ++serial_rx_errors;
        s_command_length = 0;
        s_command_discard = 1; /* A damaged command is never accepted. */
        LPUART_ClearStatusFlags(LPUART1, errors);
        if (g_sm.state == ST_RUN)
            state_machine_request_fault(SM_ERR_UART_RX, state_machine_epoch());
    }
    while (uart_query_byte(UART_1, &byte)) parse_byte(byte);
    /* SDK's eDMA callback enables TC only after the last DMA byte. The UART
     * interrupt must finish the SDK transfer too, or TX remains busy forever. */
    if ((LPUART_GetEnabledInterrupts(LPUART1) & kLPUART_TransmissionCompleteInterruptEnable) &&
        (LPUART_GetStatusFlags(LPUART1) & kLPUART_TransmissionCompleteFlag))
        LPUART_TransferEdmaHandleIRQ(LPUART1, &s_uart_handle);
    __DSB();
}

void serial_link_dma_error_irq(void)
{
    if (EDMA_GetChannelStatusFlags(DMA0, TX_DMA_CHANNEL) & kEDMA_ErrorFlag)
    {
        LPUART_EnableTxDMA(LPUART1, false);
        EDMA_ClearChannelStatusFlags(DMA0, TX_DMA_CHANNEL, kEDMA_ErrorFlag);
        s_tx_fault = 1;
        if (g_sm.state == ST_RUN)
            state_machine_request_fault(SM_ERR_UART_DMA, state_machine_epoch());
    }
    __DSB();
}
