/*
 * io.c
 *
 *  Created on: Dec 2, 2021
 *      Author: alan
 *
 *  This file/routine is used to fully implement the printf routine using
 *  the appropriate usart
 */
#include "common.h"



#ifdef __cplusplus
 extern "C" {
#endif
// FIXME : is this still needed if we switch to a simpler printf implementation?
int __io_putchar(int ch) {
  //HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  //while (!(instance.regs->ISR & USART_ISR_TXE)) {};
  while (!LL_USART_IsActiveFlag_TXE(USART3)) {};
  //instance.regs->TDR = (uint8_t) ch;
  LL_USART_TransmitData8(USART3, (uint8_t) ch);
  return ch;
}

#ifdef __cplusplus
}
#endif

