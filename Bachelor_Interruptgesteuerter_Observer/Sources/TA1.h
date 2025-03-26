#include "../base.h"
#include "event.h"

#ifndef TA1_H_
#define TA1_H_

#define GET_COUNT(state) ((state) & 0x07)
#define GET_STATE(state) (((state) >> 3) & 0x01)
#define SET_COUNT(state, count) ((state) = ((state) & 0xF8) | ((count) & 0x07))
#define SET_STATE(state, button_state) ((state) = ((state) & 0xF7) | ((button_state) << 3))

EXTERN Void TA1_init(Void);

#endif /* TA1_H_ */
