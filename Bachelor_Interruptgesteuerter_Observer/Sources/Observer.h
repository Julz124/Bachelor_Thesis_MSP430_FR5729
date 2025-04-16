/*
 * Observer.h
 *
 *  Created on: 26.03.2025
 *      Author: Julian Rapp
 */

#include "..\base.h"

#ifndef OBSERVER_H_
#define OBSERVER_H_

typedef unsigned int TEvt;

#define CMD_LENGTH (3 + 1)

/*
 * Event initialization
 */
#define NO_EVTS   0x0000
#define EVT_1     0x0001
#define EVT_2     0x0002
#define EVT_3     0x0004
#define EVT_4     0x0008
#define EVT_5     0x0010
#define EVT_6     0x0020
#define EVT_7     0x0040
#define EVT_8     0x0080
#define EVT_9     0x0100
#define EVT_10    0x0200
#define EVT_11    0x0400
#define EVT_12    0x0800
#define EVT_13    0x1000
#define EVT_14    0x2000
#define EVT_15    0x4000
#define EVT_16    0x8000
#define ALL_EVTS  0xFFFF

/*
 * Event allocation
 */
#define CMD_RDY         EVT_1     // Command event
#define CMD_RUN         EVT_2     // Command running
#define RST             EVT_3     // Reset Register

#define UART_ERR        EVT_4     // UART error
#define CMD_ERR         EVT_5     // Command error

/*
 * UART Error
 */
#define NO_ERR              6       // no error
#define TIME_OUT            5       // time out
#define BUFFER_ERROR        4       // buffer error (e.g. to many bytes received)
#define CHARACTOR_ERROR     3       // character error (e.g. wrong character received)
#define FROVPAR_ERROR       2       // frame overrun or parity error
#define BREAK_ERROR         1       // break error (lost communication)
#define PRINT_ERROR         0       // unable to print on UART

/*
 * Command Error
 */
#define NO_CMD              5       // no command to compute
#define UNKNOWN_CMD         4       // unknown command
#define INV_PTR             3       // Invalid function pointer
#define INV_ADDR            2       // Invalid memory address
#define INV_BLCK            1       // Invalid block-size
#define INV_STR             0       // Invalid string


// Struct to combine key and function pointer
//typedef Void (*ObserverFunc)(Void);

typedef struct {
    const Char key[CMD_LENGTH];     // String key for the function
    Void (*func)(Void);             // Pointer to the function
} ObserverFuncEntry;

/*
 * Main-Functionality Logic
 */

// Reads from memory cell(s)
LOCAL Void read_mem(Void);

// Writes to memory cell(s)
LOCAL Void write_mem(Void);

// Set interrupt breakpoint
LOCAL Void set_interrupt(Void);

// EXTERN Function declarations
EXTERN Void Observer_init(Void);
EXTERN Void Observer(Void);

LOCAL Int observer_print(const Char * str);

#endif /* OBSERVER_H_ */
