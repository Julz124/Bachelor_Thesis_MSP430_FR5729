
#include "Handler.h"
#include "TA0.h"
#include "UCA0.h"
#include "UCA1.h"
#include "event.h"
#include <msp430fr5729.h>

#define BASE 10

typedef Void (*VoidFunc)(Void);

LOCAL UChar selected_button;
LOCAL UChar bcd_digits[DIGISIZE];
LOCAL UChar state;
LOCAL UChar idx;
LOCAL UChar error;

#pragma FUNC_ALWAYS_INLINE(Button_Handler)
GLOBAL Void Button_Handler(Void) {
  TEvent local_btn_event = get_events(0x003F);

  if (local_btn_event & EVENT_BTN1) {
    P2OUT ^= BIT7;
  }

  if (local_btn_event & (EVENT_BTN3 | EVENT_BTN4 | EVENT_BTN5 | EVENT_BTN6)) {
    selected_button = (local_btn_event >> 2) & 0xF;
    Event_set(EVENT_UPDATE_CNT);
  }
}

#pragma FUNC_ALWAYS_INLINE(Number_Handler)
GLOBAL Void Number_Handler(Void) {
  if (get_events(EVENT_UPDATE_CNT)) {
    UChar increment = !(P2OUT & BIT7);
    for (UChar i = 0; i < 4; i++) {
      UChar digit_change = (selected_button >> i) & 0x1;
      if (increment) {
        bcd_digits[i] += digit_change;
        if (bcd_digits[i] == BASE) {
          bcd_digits[i] = 0;
          if (i < 3)
            bcd_digits[i + 1]++;
        }
      } else {
        bcd_digits[i] -= digit_change;
        if (bcd_digits[i] >= BASE) {
          bcd_digits[i] = 9;
          if (i < 3)
            bcd_digits[i + 1]--;
        }
      }
    }
    selected_button = 0;
    if (!Event_tst(EVENT_UPDATE_CNT))
      Event_set(EVENT_UPDATE_SEG);
  }
}

#pragma FUNC_ALWAYS_INLINE(AS1108_Handler)
GLOBAL Void AS1108_Handler(Void) {
  TEvent local_event = get_events(EVENT_UPDATE_SEG | EVENT_DONE_SEG);

  if (!state && (local_event & EVENT_UPDATE_SEG)) {
    idx = 1;
    state = 1;
    Event_set(EVENT_DONE_SEG);
  } else if (state && (local_event & EVENT_DONE_SEG)) {
    if (idx <= DIGISIZE) {
      UCA1_emit(idx, bcd_digits[idx - 1]);
      idx++;
    } else {
      state = 0;
      Event_set(EVENT_TXD);
    }
  }
}

#pragma FUNC_ALWAYS_INLINE(UART_Handler)
GLOBAL Void UART_Handler(Void) {
  TEvent local_event = get_events(EVENT_RXD | EVENT_TXD);

  if (local_event & EVENT_RXD) {
    UChar i;
    for (i = 0; i < DIGISIZE; i++) {
      bcd_digits[i] = rx_buf[DIGISIZE - 1 - i] - '0';
    }
    Event_set(EVENT_UPDATE_SEG);
  }

  if (local_event & EVENT_TXD) {
    LOCAL Char bcd_uart[DIGISIZE + 3];

    for (UChar i = 0; i < DIGISIZE; i++) {
      bcd_uart[i] = bcd_digits[DIGISIZE - 1 - i] + '0';
    }
    bcd_uart[DIGISIZE] = '\r';
    bcd_uart[DIGISIZE + 1] = '\n';
    bcd_uart[DIGISIZE + 2] = '\0';

    UCA0_printf(bcd_uart);
  }
}

#pragma FUNC_ALWAYS_INLINE(Error_Handler)
GLOBAL Void Error_Handler(Void) {
  if (get_events(EVENT_ERR)) {
    set_blink_muster((error == BREAK_ERROR)       ? MUSTER2
                     : (error == FROVPAR_ERROR)   ? MUSTER5
                     : (error == CHARACTOR_ERROR) ? MUSTER4
                     : (error == BUFFER_ERROR)    ? MUSTER3
                                                  : MUSTER1);
  }
}

#pragma FUNC_ALWAYS_INLINE(set_error)
GLOBAL Void set_error(UChar err) {
  if (err == NO_ERROR || error > err) {
    error = err;
    Event_set(EVENT_ERR);
  }
}

#pragma FUNC_ALWAYS_INLINE(Handler_init)
GLOBAL Void Handler_init(Void) {
  state = 0;
  idx = 1;
  error = NO_ERROR;
  for (UChar i = 0; i < DIGISIZE; i++) {
    bcd_digits[i] = 0;
  }
}
