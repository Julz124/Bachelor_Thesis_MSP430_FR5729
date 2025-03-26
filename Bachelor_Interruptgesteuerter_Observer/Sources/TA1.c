#include <msp430.h>
#include "..\base.h"
#include "TA1.h"
#include "event.h"

#define NUM_BUTTONS 5
#define GET_PORT_REG(cfg) (((cfg)&0x10) ? &P3IN : &P1IN)
#define GET_INPUT_PIN(cfg) ((cfg)&0x0F)

LOCAL Void debounce_BTN(UChar idx);
LOCAL UChar buttonStates[NUM_BUTTONS]; // Single byte per button
LOCAL UChar btn_index;

const UChar buttonConfigs[NUM_BUTTONS] = {
    (BIT1 | 0x00), // P1.1
    (BIT0 | 0x10), // P3.0
    (BIT1 | 0x10), // P3.1
    (BIT2 | 0x10), // P3.2
    (BIT3 | 0x10)  // P3.3
};

LOCAL const TEvent events[NUM_BUTTONS] = {EVENT_BTN1, EVENT_BTN3, EVENT_BTN4,
                                          EVENT_BTN5, EVENT_BTN6};

#pragma FUNC_ALWAYS_INLINE(TA1_init)
GLOBAL Void TA1_init(Void) {

   CLRBIT(TA1CTL,   MC0 | MC1  // stop mode
                     | TAIE       // disable interrupt
                     | TAIFG);    // clear interrupt flag
      CLRBIT(TA1CCTL0, CM1 | CM0  // no capture mode
                     | CAP        // compare mode
                     | CCIE       // disable interrupt
                     | CCIFG);    // clear interrupt flag
      TA1CCR0  = 2-1;            // set up Compare Register
      TA1EX0   = TAIDEX_7;        // set up expansion register
      TA1CTL   = TASSEL__ACLK     // 614.4 kHz
               | MC__UP           // Up Mode
               | ID__8            // /8
               | TACLR            // clear and start Timer
               | TAIE;            // enable interrupt

      for (UChar i = 0; i < NUM_BUTTONS; i++) {
          buttonStates[i] = 0;
      }

      btn_index = 0;
   }

#pragma vector = TIMER1_A1_VECTOR
__interrupt Void TIMER1_A1_ISR(Void) {

   TA1CCR0  = 2-1;

   debounce_BTN(btn_index);

   btn_index++;

   if(btn_index >= NUM_BUTTONS) {
       btn_index = 0;
       TA1CCR0  = (1*96)-1;
   }

   CLRBIT(TA1CTL, TAIFG);           // clear interrupt flag
   __low_power_mode_off_on_exit();  // restore Active Mode on return
}

#pragma FUNC_ALWAYS_INLINE(debounce_BTN)
LOCAL Void debounce_BTN(UChar idx) {

    UChar cfg = buttonConfigs[idx];
    UChar inputPin = GET_INPUT_PIN(cfg);
    const volatile UChar *portReg = GET_PORT_REG(cfg);

    UChar count = GET_COUNT(buttonStates[idx]);
    UChar state = GET_STATE(buttonStates[idx]);

    if (TSTBIT(*portReg, inputPin)) {
      if (count < 5) {
        count++;
      }
    } else {
      if (count > 0) {
        count--;
      }
    }

    if (state == 0) {
      if (count == 5) {
        state = 1;
        Event_set(events[idx]);
      }
    } else {
      if (count == 0) {
        state = 0;
      }
    }

    SET_COUNT(buttonStates[idx], count);
    SET_STATE(buttonStates[idx], state);
}
