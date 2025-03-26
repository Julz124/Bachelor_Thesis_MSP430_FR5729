#include "..\base.h"

#ifndef UCA0_H_
#define UCA0_H_

#include "Handler.h"

extern Char rx_buf[DIGISIZE + 1];

EXTERN Void UCA0_init(Void);
EXTERN Int UCA0_printf(const Char * str);

#endif /* UCA0_H_ */
