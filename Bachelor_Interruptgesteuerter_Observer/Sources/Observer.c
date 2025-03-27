/*
 * Observer.c
 *
 *  Created on: 26.03.2025
 *      Author: Julian Rapp
 */

#include <msp430.h>
#include <..\base.h>
#include <Observer.h>

#pragma FUNC_ALWAYS_INLINE(Observer_init)
GLOBAL Void Observer_init(Void) {

    // Initialize Clock (Interrupt Timer & Time-Out Timer) TB0
    CLRBIT(TB0CTL, MC0 | MC1        // stop timer
                | TBIE              // interrupt disabled
                | TBIFG             // no interrupt pending
    );


    TB0R = 0FFFFh;                  // set Timer B0 to 16-bit timer

    // Interrupt Timer
    CLRBIT(TB0CCTL0, CM0 | CM1      // no capture-mode
                       | CAP        // compare-mode
                       | CCIE       // disable interrupt
                       | CCIFG      // no interrupt pending
    );

    /*
     * ACLK = 614.4kHz
     * min. Timer = 10ms
     * 614.4kHz * 10ms = 6144
     * 6144 / 8 / 8 = 96
     */
    TB0CCR0 = 96;                   // compare register interrupt-timer
    TB0EX0 = TBIDEX_7;              // divide by 8


    // Time-Out-Timer
    CLRBIT(TB0CCTL0, CM0 | CM1      // no capture-mode
                       | CAP        // compare-mode
                       | CCIE       // disable interrupt
                       | CCIFG      // no interrupt pending
    );

    /*
     * ACLK = 614.4kHz
     * min. Timer = 6s
     * 614.4kHz * 6s / 8 / 8 = 57.600
     * max count to 65.535 (16-bit)
     */
    TB0CCR1 = 57600;                // compare register time-out-timer
    TB0EX1 = TBIDEX_7;              // divide by 8

    // Set and Start Timer B
    TB0CTL = TASSEL__ACLK           // set to ACLK (614.4kHz)
            | MC__UP                // mode control, count upwards
            | ID__8                 // input divider set to /8
            | TBCLR                 // clears TBR, clock divider logic, count direction


    // Initialize UART


}
