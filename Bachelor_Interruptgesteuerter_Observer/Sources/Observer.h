/*
 * Observer.h
 *
 *  Created on: 26.03.2025
 *      Author: Julian Rapp
 */

#include "..\base.h"

#ifndef OBSERVER_H_
#define OBSERVER_H_

typedef UInt TEvt;

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
#define TIMEOUT_FLAG    EVT_1   // Timeout flag for timeout counter

#define CMD_RDY         EVT_2   // Command event
#define CMD_RUN         EVT_3   // Command running
#define RST             EVT_4   // Reset Register

#define UART_ERR        EVT_5   // UART error
#define CMD_ERR         EVT_6   // Command error

/*
 * UART Error
 */
#define NO_ERR              0x00    // no error
#define TIME_OUT            'A'     // time out
#define BUFFER_ERROR        'B'     // buffer error (e.g. to many bytes received)
#define FROVPAR_ERROR       'C'     // frame overrun or parity error
#define BREAK_ERROR         'D'     // break error (lost communication)
#define PRINT_ERROR         'E'     // unable to print on UART

/*
 * Command Error
 */
#define UNKNOWN_CMD         '1'     // unknown command
#define INV_PTR             '2'     // Invalid function pointer
#define INV_ADDR            '3'     // Invalid memory address
#define INV_BLCK            '4'     // Invalid block-size
#define INV_STR             '5'     // Invalid string
#define BRP_SET             '6'     // Breakpoint already set
#define BRP_MAX             '7'     // Max available Breakpoints set
#define BRP_RES             '8'     // Breakpoint reset failed

/*
 * Custom Makros
 */
#define IS_ALNUM(ch) (between('0', (ch), '9') || between('A', (ch), 'Z') || between('a', (ch), 'z'))

// Struct to combine key and function pointer
//typedef Void (*ObserverFunc)(Void);

typedef struct {
    const Char key[CMD_LENGTH];     // String key for the function
    Void (*func)(Void);             // Pointer to the function
} ObserverFuncEntry;

// EXTERN Function declarations
EXTERN Void Observer_init(Void);
EXTERN Void Observer(Void);

LOCAL Int observer_print(const Char * str);

#endif /* OBSERVER_H_ */
