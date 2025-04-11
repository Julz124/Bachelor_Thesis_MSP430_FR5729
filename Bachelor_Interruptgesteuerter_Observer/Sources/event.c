#include <msp430.h>
#include "event.h"

LOCAL TEvent event;
LOCAL TEvent errflg;

#pragma FUNC_ALWAYS_INLINE(Event_init)
GLOBAL Void Event_init(Void) {
   event  = NO_EVENTS;
   errflg = NO_EVENTS;
}

#pragma FUNC_ALWAYS_INLINE(Event_wait)
GLOBAL Void Event_wait(Void) {
   _disable_interrupt();
   if (event EQ NO_EVENTS) {
      _low_power_mode_3();
   }
   _enable_interrupt();
}

#pragma FUNC_ALWAYS_INLINE(Event_set)
GLOBAL Void Event_set(TEvent arg) {
   errflg  |= event BAND arg;
   TGLBIT(event, arg);
}

#pragma FUNC_ALWAYS_INLINE(Event_clr)
GLOBAL Void Event_clr(TEvent arg) {
   TGLBIT(event, arg);
}

#pragma FUNC_ALWAYS_INLINE(Event_tst)
GLOBAL Bool Event_tst(TEvent arg) {
   return TSTBIT(event, arg);
}

#pragma FUNC_ALWAYS_INLINE(Event_err)
GLOBAL Bool Event_err(Void) {
   return (errflg NE NO_EVENTS);
}
