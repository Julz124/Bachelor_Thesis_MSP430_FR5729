#include "TA0.h"
#include "../base.h"
#include <msp430fr5729.h>
#include <stdint.h>

#define SCALING 2400 - 1

LOCAL UChar pattern_index;
LOCAL UChar cnt_led;
LOCAL UChar req_pattern_index;
LOCAL UChar state_index;

// Set the requested blink pattern index
#pragma FUNC_ALWAYS_INLINE(set_blink_muster)
GLOBAL Void set_blink_muster(UChar arg) { req_pattern_index = arg; }

// Initialize Timer A0
#pragma FUNC_ALWAYS_INLINE(TA0_init)
GLOBAL Void TA0_init(Void) {


  TA0CTL = TASSEL__ACLK | MC__STOP |
           TACLR; // Stop the timer, ACLK source, clear timer
  TA0CCTL0 = 0;   // Clear control register

  TA0CCR0 = SCALING; // Set up Compare Register with SCALING Factor
  TA0EX0 = TAIDEX_7; // Set up expansion register
  TA0CTL |= MC__UP | ID__8 | TAIE; // Up mode, /8 divider, enable interrupt

  pattern_index = 0;
  cnt_led = 0;
  req_pattern_index = 0;
  state_index = 0;
  SETBIT(P1OUT, BIT2);
}

// Timer A0 interrupt service routine
#pragma vector = TIMER0_A1_VECTOR
__interrupt Void TIMER0_A1_ISR(Void) {
  // Blink pattern array
  LOCAL const UChar blink_pattern[] = {2, 8, 0, 1, 1, 0, 2, 2, 6, 0, 2,
                                       2, 2, 2, 6, 0, 2, 2, 2, 2, 2, 2, 6, 0};

  // Blink pattern pointer array
  LOCAL const UChar blink_ptr_arr[] = {0, 3, 6, 10, 16};

  cnt_led++;

  // Get the current pattern value based on the pattern index and state index
  UChar cur_pattern_val =
      blink_pattern[(uint16_t) blink_ptr_arr[pattern_index] + state_index];

  if (cnt_led == cur_pattern_val) {

    TGLBIT(P1OUT, BIT2); // Toggle the LED
    cnt_led = 0;
    state_index++;

    // Check if the next pattern value is 0, indicating the end of the pattern
    if (blink_pattern[(uint16_t) blink_ptr_arr[pattern_index] + state_index] == 0) {
      state_index = 0;
      CLRBIT(P1OUT, BIT2);
      pattern_index = req_pattern_index; // Update the pattern index
    }
  }

  CLRBIT(TA0CTL, TAIFG); // Clear the interrupt flag
}
