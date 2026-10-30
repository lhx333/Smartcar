/* Hardware boundary for host regression tests; production headers/constants
 * and production application C sources are used unchanged by the harness. */
#ifndef HOST_ZF_COMMON_H
#define HOST_ZF_COMMON_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
typedef uint8_t uint8;
typedef int8_t int8;
typedef uint16_t uint16;
typedef int16_t int16;
typedef uint32_t uint32;
typedef int32_t int32;
typedef int IRQn_Type;
enum { PendSV_IRQn = -2, PIT_IRQn = 1, DMA8_DMA24_IRQn = 2, DMA_ERROR_IRQn = 3, LPUART1_IRQn = 4 };
enum { PIT_CH0, PIT_CH1, PIT_CH2, PIT_CH3, UART_1, B9, GPO, GPIO_HIGH, GPO_PUSH_PULL };
typedef struct { uint32 ICSR; } mock_scb_t;
typedef struct { uint32 CYCCNT, CTRL; } mock_dwt_t;
typedef struct { uint32 DEMCR; } mock_debug_t;
extern mock_scb_t mock_scb;
extern mock_dwt_t mock_dwt;
extern mock_debug_t mock_debug;
#define SCB (&mock_scb)
#define DWT (&mock_dwt)
#define CoreDebug (&mock_debug)
#define SCB_ICSR_PENDSVSET_Msk 1U
#define SCB_ICSR_PENDSVCLR_Msk 2U
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
uint32 __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32 value);
#define __DMB() ((void)0)
#define __DSB() ((void)0)
#define __ISB() ((void)0)
void NVIC_SetPriority(IRQn_Type irq, uint32 priority);
void NVIC_ClearPendingIRQ(IRQn_Type irq);
void NVIC_EnableIRQ(IRQn_Type irq);
void pit_ms_init(int channel, uint32 period);
void system_delay_ms(uint32 ms);
void gpio_toggle_level(int pin);

#define IMU660RC_QUARTERNION_DISABLE 0
#define IMU660RC_SPI 1
#define SPI_MODE0 0
#define IMU660RC_SPI_SPEED 10000000
#define IMU660RC_SPC_PIN 23
#define IMU660RC_SDI_PIN 22
#define IMU660RC_SDO_PIN 21
#define IMU660RC_CS_PIN 20
#define SPI_CS_NULL -1
void spi_init(int bus, int mode, int rate, int sck, int mosi, int miso, int cs);
void gpio_init(int pin, int direction, int level, int config);
extern int16 imu660rc_acc_x, imu660rc_acc_y, imu660rc_acc_z;
extern int16 imu660rc_gyro_x, imu660rc_gyro_y, imu660rc_gyro_z;
extern float imu660rc_transition_factor[2];
uint8 imu660rc_init(int mode);
uint8 imu660rc_is_present(void);
void imu660rc_get_acc(void);
void imu660rc_get_gyro(void);

#define AT_OCRAM_SECTION_ALIGN(var, alignment) var __attribute__((aligned(alignment)))
void SCB_CleanDCache_by_Addr(uint32_t *address, int32_t length);
typedef struct { uint32 FIFO; } LPUART_Type;
extern LPUART_Type mock_uart;
#define LPUART1 (&mock_uart)
#define LPUART_FIFO_TXFLUSH_MASK 1U
enum {
    kLPUART_RxOverrunFlag = 1, kLPUART_NoiseErrorFlag = 2,
    kLPUART_FramingErrorFlag = 4, kLPUART_ParityErrorFlag = 8,
    kLPUART_TransmissionCompleteInterruptEnable = 16,
    kLPUART_TransmissionCompleteFlag = 32,
    kEDMA_ErrorInterruptEnable = 1, kEDMA_ErrorFlag = 1,
    kEDMA_InterruptFlag = 2, kDmaRequestMuxLPUART1Tx = 258,
    kStatus_Success = 0, kStatus_LPUART_TxIdle = 1
};
typedef int status_t;
typedef struct { int unused; bool enableHaltOnError; } edma_config_t;
typedef struct { int unused; } edma_handle_t;
typedef struct { uint8 *data; size_t dataSize; } lpuart_transfer_t;
typedef struct lpuart_edma_handle lpuart_edma_handle_t;
typedef void (*mock_tx_callback)(LPUART_Type *, lpuart_edma_handle_t *, status_t, void *);
struct lpuart_edma_handle { mock_tx_callback callback; int busy; };
#define DMA0 ((void *)1)
#define DMAMUX ((void *)2)
void DMAMUX_Init(void *base);
void EDMA_GetDefaultConfig(edma_config_t *config);
void EDMA_Init(void *base, edma_config_t *config);
void DMAMUX_SetSource(void *base, uint32 channel, uint32 source);
void DMAMUX_EnableChannel(void *base, uint32 channel);
void EDMA_CreateHandle(edma_handle_t *handle, void *base, uint32 channel);
void EDMA_EnableChannelInterrupts(void *base, uint32 channel, uint32 mask);
void EDMA_ClearChannelStatusFlags(void *base, uint32 channel, uint32 mask);
uint32 EDMA_GetChannelStatusFlags(void *base, uint32 channel);
void LPUART_TransferCreateHandleEDMA(LPUART_Type *base, lpuart_edma_handle_t *handle,
                                   mock_tx_callback cb, void *user, edma_handle_t *tx, void *rx);
void uart_rx_interrupt(int uart, int enabled);
int uart_query_byte(int uart, uint8 *byte);
status_t LPUART_SendEDMA(LPUART_Type *base, lpuart_edma_handle_t *handle, lpuart_transfer_t *transfer);
void LPUART_TransferAbortSendEDMA(LPUART_Type *base, lpuart_edma_handle_t *handle);
void LPUART_TransferEdmaHandleIRQ(LPUART_Type *base, void *handle);
void LPUART_EnableTxDMA(LPUART_Type *base, bool enabled);
void LPUART_DisableInterrupts(LPUART_Type *base, uint32 mask);
uint32 LPUART_GetStatusFlags(LPUART_Type *base);
uint32 LPUART_GetEnabledInterrupts(LPUART_Type *base);
void LPUART_ClearStatusFlags(LPUART_Type *base, uint32 flags);
#endif
