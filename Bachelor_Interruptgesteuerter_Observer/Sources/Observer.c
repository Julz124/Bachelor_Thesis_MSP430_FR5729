/*
 * Observer.c
 *
 *  Created on: 26.03.2025
 *      Author: Julian Rapp
 */

#include <msp430.h>
#include <msp430fr5729.h>
#include <..\base.h>
#include <Observer.h>

#pragma FUNC_ALWAYS_INLINE(Observer_init)
GLOBAL Void Observer_init(Void) {
    /*
     * Initialize Clock (Interrupt Timer & Time-Out Timer) TA2
     */
    TA2CTL = MC__STOP               // stop timer
                | TACLR;            // clear counter

    /*
     * Interrupt Timer
     *
     * ACLK = 614.4kHz
     * min. Timer = 10ms
     * 614.4kHz * 10ms = 6144
     * 6144 / 8 / 8 = 96
     *
     * Compare-Mode
     */

    TA2CCTL0 = 0;                   // clear timer A2 control register 0

    TA2CCR0 = 96;                   // compare register interrupt-timer
    TA2EX0 = TBIDEX_7;              // divide by 8

    /*
     * Time-Out-Timer
     *
     * ACLK = 614.4kHz
     * min. Timer = 6s
     * 614.4kHz * 6s / 8 / 8 = 57.600
     * max count to 65.535 (16-bit)
     *
     * Capture-Mode rising and falling edge
     * TODO: Select Input signal CCIxA (see 11.3.3 User's Guide)
     */

    TA2CCTL1 = 0;                   // clear timer A2 control register 1
    TA2CCTL1 = CAP                  // enable capture-mode
                | CM_3;             // capture-mode rising & falling edge

    TA2CCR1 = 57600;                // compare register time-out-timer
    TA2EX1 = TBIDEX_7;              // divide by 8

    /*
     * Set and Start Timer A
     */
    TA2CTL = TASSEL__ACLK           // set to ACLK (614.4kHz)
            | ID__8                 // input divider set to /8
            | MC__CONTINUOUS        // mode control, count continuously upwards
            | TAIE;                 // enable timer interrupt


    /*
     * Initialize UART
     */

}

#pragma vector = TIMER2_A1_VECTOR
__interrupt Void TIMER2_A1_ISR(Void) {


}

#pragma FUNC_ALWAYS_INLINE(Observer_print)
LOCAL int Observer_print(const char * str) {
    if (str == NULL) {
        return -1;
    }
    ptr = str;
    SETBIT(UCA0IFG, UCTXIFG);
    SETBIT(UCA0IE,  UCTXIE);
    return 0;
}

#pragma FUNC_ALWAYS_INLINE(read_mem)
LOCAL int read_mem(const int str, const int addr, const int blocks) {

}

#pragma FUNC_ALWAYS_INLINE(write_mem)
LOCAL int write_mem(const int str, const int addr, const int blocks) {

}

#pragma FUNC_ALWAYS_INLINE(interrupt)
LOCAL int interrupt() {

}
