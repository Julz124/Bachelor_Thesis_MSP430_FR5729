#include "../base.h"
#include "event.h"

#ifndef TA1_H_
#define TA1_H_

// Button constant values
typedef struct {
    const UInt      pin;
    const TEvent    event;
    const Char *    port;
}button_const;

// Button variable values
typedef struct {
    Char            cnt;
    Char           state;
}button_var;

// Struct representing button
typedef struct {
    button_const const*   btn_const;
    button_var*     btn_var;
}Button;

EXTERN Void TA1_init(Void);

#endif /* TA1_H_ */
