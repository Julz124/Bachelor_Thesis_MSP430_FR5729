#include "..\base.h"

#ifndef HANDLER_H_
#define HANDLER_H_

#define DIGISIZE 4

EXTERN Void Handler_init(Void);
EXTERN Void Button_Handler(Void);
EXTERN Void Number_Handler(Void);
EXTERN Void AS1108_Handler(Void);
EXTERN Void UART_Handler(Void);
EXTERN Void Error_Handler(Void);

EXTERN Void get_bcd_cnt(Void);
EXTERN Void set_error(UChar err);

#endif /* HANDLER_H_ */
