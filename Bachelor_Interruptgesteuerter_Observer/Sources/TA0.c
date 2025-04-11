#include <msp430.h>
#include "../base.h"
#include "TA0.h"

#define SCALING (2400 - 1)
#define HIGH 0x80
#define LOW  0x00

LOCAL const UChar muster_all[] = {
    HIGH | 8, LOW | 2, 0,

    HIGH|3, LOW | 3, 0,

    HIGH|1, LOW | 1,  0,

    LOW | 2, HIGH| 2, LOW | 6,   0,

    LOW | 2, HIGH |2, LOW| 2,  HIGH |2, LOW| 6, 0,

    LOW | 2, HIGH |2, LOW | 2, HIGH |2, LOW| 2, HIGH|2 , LOW| 6, 0
};

LOCAL const UChar * const blink_ptr_arr[] = {
    &muster_all[0],
    &muster_all[3],
    &muster_all[6],
    &muster_all[9],
    &muster_all[13],
    &muster_all[19]
};

LOCAL const UChar* cur_pattern_ptr;
LOCAL UChar cnt_led;
LOCAL UChar req_pattern_index;

GLOBAL Void set_blink_muster(UInt arg) {
    req_pattern_index = arg;
}

#pragma FUNC_ALWAYS_INLINE(TA0_init)
GLOBAL Void TA0_init(Void) {
   cnt_led              = 0;
   req_pattern_index    = 0;

   CLRBIT(TA0CTL, MC0 | MC1   // stop mode
                  | TAIE      // disable interrupt
                  | TAIFG);   // clear interrupt flag
   CLRBIT(TA0CCTL0, CM1 | CM0 // no capture mode
                  | CAP       // compare mode
                  | CCIE      // disable interrupt
                  | CCIFG);   // clear interrupt flag

   TA0CCR0  = SCALING;        // set up Compare Register with SCALING Factor
   TA0EX0   = TAIDEX_7;       // set up expansion register. IDEX in TAxEX0: {/1, /2, /3, /4, /5, /6, /7, /8}
   TA0CTL   = TASSEL__ACLK    // 614.4 kHz
            | MC__UP          // Up Mode
            | ID__8           // /8     ID in TAxCTL: {/1, /2, /4, /8}
            | TACLR;          // clear and start Timer

   SETBIT(TA0CTL, TAIE        // enable interrupt
                | TAIFG);     // set interrupt flag

   SETBIT(P1OUT, BIT2);
   cur_pattern_ptr = blink_ptr_arr[0];
}

#pragma vector = TIMER0_A1_VECTOR
__interrupt Void TIMER0_A1_ISR(Void) {
    const char MASK = 0x7F;
    if (cnt_led != ((*cur_pattern_ptr) & MASK)) {
        cnt_led++;
        if(((*cur_pattern_ptr) & HIGH) == HIGH)
            SETBIT(P1OUT, BIT2);
        else
            CLRBIT(P1OUT, BIT2);
    }
    if(cnt_led == ((*cur_pattern_ptr) & MASK)) {
        cur_pattern_ptr++;
        cnt_led = 0;
    }
    if (*cur_pattern_ptr == 0) {
        cur_pattern_ptr = blink_ptr_arr[req_pattern_index];
        cnt_led = 0;
    }
    CLRBIT(TA0CTL, TAIFG);
}
