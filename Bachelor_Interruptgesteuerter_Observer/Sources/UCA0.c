#include <msp430.h>
#include "..\base.h"
#include "event.h"
#include "uca0.h"

LOCAL const Char * ptr;
LOCAL UChar i;
LOCAL Char ch;
GLOBAL Char rx_buf[DIGISIZE + 1];

#pragma FUNC_ALWAYS_INLINE(UCA0_init)
GLOBAL Void UCA0_init(Void) {

   SETBIT(UCA0CTLW0, UCSWRST);  // UCA0 software reset

   UCA0CTLW1 = 0x0002;          // deglitch time approximately 100 ns
   UCA0BRW   = 2;               // set clock prescaler for 14400 baud
   UCA0MCTLW = 0xD6 << 8        // second modulation stage
             | 0x0A << 4        // first modulation stage
             | UCOS16;          // enable 16 times oversampling

   UCA0CTLW0 = UCPEN            // enable parity
             | UCPAR            // even parity
             | 0                // LSB first
             | 0                // 8-bit-data
             | 0                // one stop bit
             | UCMODE_0         // UART mode
             | 0                // Asynchronous mode
             | UCSSEL__ACLK     // select clock source: ACLK
             | UCRXEIE          // error interrupt enable
             | UCBRKIE          // break interrupt enable
             | 0;               // release the UCA0 for operation

   UCA0IE    = 0                // Transmit Complete Interrupt Disable
             | 0                // Start Bit Interrupt Disable
             | 0                // Transmit Interrupt Disable
             | UCRXIE;          // Receive Interrupt Enable

   i = 0;
   ch = '\0';
}

#pragma vector = USCI_A0_VECTOR
__interrupt Void UCA0_ISR(Void) {


   switch (__even_in_range(UCA0IV, 0x04)) {

      case 0x02:
         if (TSTBIT(UCA0STATW, UCBRK)) {
            set_error(BREAK_ERROR);
            Char ch = UCA0RXBUF;
            return;
         }

         if (TSTBIT(UCA0STATW, UCRXERR)) {
            set_error(FROVPAR_ERROR);
            Char ch = UCA0RXBUF;
            return;
         }

         ch = UCA0RXBUF;

         if (between('0', ch, '9')) {
            if(i < DIGISIZE){
               rx_buf[i++] = ch;
               set_error(BYTE_RECEIVED);
            } else {
               i = 0;
               set_error(BUFFER_ERROR);
            }
         } else if (ch EQ '\r'){
            if(i == DIGISIZE){
               rx_buf[i] = '\0';
               i = 0;
               Event_set(EVENT_RXD);
               set_error(NO_ERROR);
            } else {
               i = 0;
               set_error(BUFFER_ERROR);
            }
         } else {
            i = 0;
            set_error(CHARACTOR_ERROR);
         }

         __low_power_mode_off_on_exit();

         break;

      case 0x04:

         if (TSTBIT(UCA0STATW, UCBRK)) {
            Char ch = UCA0RXBUF;
            set_error(BREAK_ERROR);
            return;
         }

         if (TSTBIT(UCA0STATW, UCRXERR)) {
            Char ch = UCA0RXBUF;
            set_error(FROVPAR_ERROR);
            return;
         }

         if (*ptr NE '\0') {
            UCA0TXBUF = *ptr++;
            return;
         }
         CLRBIT(UCA0IE, UCTXIE);
         Char ch = UCA0RXBUF;
         set_error(NO_ERROR);
         SETBIT(UCA0IE, UCRXIE);
         break;
   }
}

GLOBAL Int UCA0_printf(const Char * str) {
   if (str EQ NULL) {
      return -1;
   }
   ptr = str;
   SETBIT(UCA0IFG, UCTXIFG);
   SETBIT(UCA0IE,  UCTXIE);
   return 0;
}
