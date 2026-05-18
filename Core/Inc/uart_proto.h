/* ============================================================================
 * uart_proto.h — UART host protocol for FOC controller.
 *
 * Wire format (10-byte commands, 18-byte responses) is unchanged from
 * the original main4.c implementation.
 * ============================================================================ */
#ifndef UART_PROTO_H
#define UART_PROTO_H


/* Initialise UART DMA receive. Call once at boot. */
void uart_init(void);


/* Called from TIM7 1 kHz callback. */
void uart_poll_rx        (void);
void uart_check_timeout  (void);
void uart_update_velocity(void);


#endif /* UART_PROTO_H */
