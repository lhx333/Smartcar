#ifndef SERIAL_LINK_H
#define SERIAL_LINK_H

#include "my_common.h"

/* All TX callers are foreground. Data is copied to persistent DMA buffers. */
void serial_link_init(void);
uint8 serial_link_frame(uint8 type, const uint8 *payload, uint32 length);
uint8 serial_link_send(const void *data, uint32 length);
uint8 serial_link_send_text(const char *text);
uint8 serial_link_can_report(void);
void serial_link_service(void);
void serial_link_uart_irq(void);
void serial_link_dma_error_irq(void);
extern volatile uint32 serial_tx_dropped;
extern volatile uint32 serial_tx_errors;
extern volatile uint32 serial_rx_errors;

#endif
