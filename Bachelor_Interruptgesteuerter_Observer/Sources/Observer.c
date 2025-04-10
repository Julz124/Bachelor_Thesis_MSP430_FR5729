/*
 * Observer.c
 *
 *  Created on: 26.03.2025
 *      Author: Julian Rapp
 */

#include <msp430.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "..\base.h"
#include "Observer.h"

// Zähllänge der Timer
#define ISR_TIMER 96

// UART Einstellungen
#define UART_BUFFER_SIZE 64
#define TIMEOUT_THRESHOLD 600  // 6s = 600 * 10ms


// Event/Error Logic
LOCAL TEvent event;
LOCAL TEvent errflg;
LOCAL UChar error;


// Function Logic
static ObserverFuncEntry OBSERVER_FUNC_DICT[] = {
    {"rdm", (void*)read_mem},     // read_mem function
    {"wrm", (void*)write_mem},    // write_mem function
    {"inr", (void*)set_interrupt},    // interrupt function
    {"\0", NULL}
};

ObserverFuncEntry* dict_ptr;

// Define error message strings
static const char* ERROR_MESSAGES[] = {
    "no error",                         // NO_ERR (assuming 0)
    "no or unknown command",            // CMD_ERROR
    "lost communication",               // BREAK_ERROR
    "frame overrun or parity error",    // FROVPAR_ERROR
    "wrong character received",         // CHARACTOR_ERROR
    "too many bytes received",          // BUFFER_ERROR
    "time out",                         // TIME_OUT
    "unable to print on UART"           // PRINT_ERROR
};

static const char** err_msg_ptr;

// UART Logic
static int timeout_counter;
static char uart_buffer[UART_BUFFER_SIZE];
static int buffer_index;
static const char* print_ptr;

/*
 * Initialization
 */
#pragma FUNC_ALWAYS_INLINE(Observer_init)
GLOBAL Void Observer_init(Void) {
    /*
     * Initialize Clock (Interrupt Timer & Time-Out Timer) TA2
     */
    TB0CTL = MC__STOP               // stop timer
                | TACLR;            // clear counter

    /*
     * Interrupt Timer
     *
     * ACLK = 614.4kHz
     * min. Timer = 10ms
     * 614.4kHz * 10ms = 6144
     * 6144 / 8 / 8 = 96
     *
     * Compare-Mode
     */

    TB0CCTL0 = CCIE;                // clear timer A2 control register 0 and set it to Compare Mode

    TB0CCR0 = ISR_TIMER;            // compare register interrupt-timer
    TB0EX0 = TBIDEX_7;              // divide by 8

    /*
     * Set and Start Timer B
     */
    TB0CTL = TASSEL__ACLK           // set to ACLK (614.4kHz)
            | ID__8                 // input divider set to /8
            | MC__UP                // mode control, count continuously upwards
            | TAIE;                 // enable timer interrupt


    /*
     * Initialize UART
     */
    SETBIT(UCA0CTLW0, UCSWRST);  // UCA0 in Software-Reset versetzen

    UCA0CTLW1 = 0x0002;          // Entprellzeit ~100ns
    UCA0BRW   = 4;               // Baudraten-Prescaler für 9600 Baud (614,4kHz / (16*9600) = 4)
    UCA0MCTLW = 0x20 << 8        // zweite Modulationsstufe
              | 0x00 << 4        // erste Modulationsstufe
              | UCOS16;          // 16-faches Oversampling aktivieren

    UCA0CTLW0 = UCPEN            // Parität aktivieren
              | UCPAR            // Gerade Parität (even)
              | 0                // LSB first
              | 0                // 8 Datenbits
              | 0                // Ein Stoppbit
              | UCMODE_0         // UART-Modus
              | 0                // Asynchroner Modus
              | UCSSEL__ACLK     // Taktquelle: ACLK
              | UCRXEIE          // Fehler-Interrupt aktivieren
              | UCBRKIE          // Break-Interrupt aktivieren
              | 0;               // UCA2 aus Reset freigeben

    UCA0IE    = 0                // Transmit Complete Interrupt deaktivieren
              | 0                // Start Bit Interrupt deaktivieren
              | 0                // Transmit Interrupt deaktivieren
              | UCRXIE;          // Empfangs-Interrupt aktivieren

    event  = NO_EVENTS;
    errflg = NO_EVENTS;
    error = NO_ERR;

    dict_ptr = &OBSERVER_FUNC_DICT[0];

    timeout_counter = 0;
    buffer_index = 0;
    uart_buffer[0] = '\0';

}

/*
 * Event & Error Handling Logic
 */
#pragma FUNC_ALWAYS_INLINE(set_evt)
LOCAL Void set_evt(TEvent arg) {
   errflg  |= event BAND arg;
   TGLBIT(event, arg);
}

#pragma FUNC_ALWAYS_INLINE(set_error)
LOCAL Void set_error(UChar err) {
  if (err == NO_ERR || error > err) {
    error = err;
    set_evt(EVT_ERR);
  }
}

#pragma FUNC_ALWAYS_INLINE(clr_evt)
GLOBAL Void clr_evt(TEvent arg) {
   TGLBIT(event, arg);
}

#pragma FUNC_ALWAYS_INLINE(tst_evt)
GLOBAL Bool tst_evt(TEvent arg) {
   return TSTBIT(event, arg);
}

#pragma FUNC_ALWAYS_INLINE(err_evt)
GLOBAL Bool err_evt(Void) {
   return (errflg NE NO_EVENTS);
}

#pragma FUNC_ALWAYS_INLINE(get_evt)
GLOBAL TEvent get_evt(TEvent mask) {
   TEvent tmp_event = event & mask;
   CLRBIT(event, mask);
   return tmp_event;
}

// Print chars to UART-connection
#pragma FUNC_ALWAYS_INLINE(observer_print)
LOCAL int observer_print(const char * str) {
    if (str EQ NULL) {
        set_error(PRINT_ERROR);
        return -1;
    }
    print_ptr = str;
    SETBIT(UCA0IFG, UCTXIFG);   // UART Transmit Interrupt Flag
    SETBIT(UCA0IE,  UCTXIE);    // Enable UART Receive Interrupt
    return 0;
}

/*
 * Main-Functionality Logic
 */

// Reads from memory cell(s)
#pragma FUNC_ALWAYS_INLINE(read_mem)
LOCAL int read_mem(void) {

    // Extract useful
    char *mem_addr_ptr = strtok(uart_buffer + 4, " ");
    char *blocks_ptr = strtok(NULL, " ");
    int blocks = atoi(blocks_ptr);

    char mem_value = *mem_addr_ptr;

    return 0;
}

// Writes to memory cell(s)
#pragma FUNC_ALWAYS_INLINE(write_mem)
LOCAL int write_mem(void) {
    /*
     * Some Code goes in there
     */
    return 0;
}

// Set interrupt breakpoint
#pragma FUNC_ALWAYS_INLINE(set_interrupt)
LOCAL int set_interrupt(void) {
    /*
     * Some Code goes in there
     */
    return 0;
}

/*
 * UART ISR
 */
#pragma vector = USCI_A0_VECTOR
__interrupt Void USCI_A0_ISR(Void) {
    char rx_byte;

    switch(__even_in_range(UCA0IV, USCI_UART_UCTXIFG)) {

        case USCI_NONE:             // Interrupt Vector 0: No Interrupt pending
            break;

        case USCI_UART_UCRXIFG:     // Interrupt Vector 2: Data ready to read

            if (TSTBIT(UCA0STATW, UCBRK)) {
               set_error(BREAK_ERROR);
               Char ch = UCA0RXBUF;
               return;
            }

            if (TSTBIT(UCA0STATW, UCRXERR)) {
               set_error(FROVPAR_ERROR);
               Char ch = UCA0RXBUF;
               return;
            }

            // Read and save byte
            rx_byte = UCA0RXBUF;

            // Reset Timeout-Timer
            timeout_counter = 0;

            if (rx_byte == '\n') {
                return;
            }

            // Checks if rx_byte is a alphanumeric letter
            if (!isalnum(rx_byte)) {
                set_error(CHARACTOR_ERROR);
                return;
            }

            // Check on end of word
            if (rx_byte == '\r' ) {
                set_evt(CMD_RDY);       // Command ready
                return;
            }

            if (buffer_index >= UART_BUFFER_SIZE) {
                set_error(BUFFER_ERROR);;
                buffer_index = 0;
                return;
            }

            // Save chars in Buffer if space available
            uart_buffer[buffer_index] = rx_byte;
            uart_buffer[buffer_index++] = '\0';
            return;

        case USCI_UART_UCTXIFG:     // Interrupt Vector 4: Data ready to write

            // USCI Break received
            if (TSTBIT(UCA0STATW, UCBRK)) {
               Char ch = UCA0RXBUF;
               set_error(BREAK_ERROR);
               return;
            }

            // USCI RX Error Flag
            if (TSTBIT(UCA0STATW, UCRXERR)) {
               Char ch = UCA0RXBUF;
               set_error(FROVPAR_ERROR);
               return;
            }

            // Transmit character
            if (*print_ptr NE '\0') {
               UCA0TXBUF = *print_ptr++;
               return;
            }

            // Disable UART Transmit Interrupt
            CLRBIT(UCA0IE, UCTXIE);
            Char ch = UCA0RXBUF;
            set_error(NO_ERR);

            // Enable UART Receive Interrupt
            SETBIT(UCA0IE, UCRXIE);
            break;

        default:
            break;
    }
}

/*
 * Command ISR
 */
#pragma vector = TIMER0_B0_VECTOR
__interrupt Void TIMER0_B0_ISR(Void) {

    // UART Time-Out counter logic
    timeout_counter++;

    TEvent local_event = get_evt(CMD_RDY | EVT_ERR);

    if (local_event & EVT_ERR) {
        int msg_idx = ((error == CMD_ERROR)         ? 1
                    :  (error == BREAK_ERROR)       ? 2
                    :  (error == FROVPAR_ERROR)     ? 3
                    :  (error == CHARACTOR_ERROR)   ? 4
                    :  (error == BUFFER_ERROR)      ? 5
                    :  (error == TIME_OUT)          ? 6
                    :  (error == PRINT_ERROR)       ? 7
                    : 0 );

        err_msg_ptr = ERROR_MESSAGES + msg_idx;
        observer_print(*err_msg_ptr);
    }


    if (timeout_counter >= TIMEOUT_THRESHOLD) {
        timeout_counter = 0;  // Reset counter
        set_error(TIME_OUT);
        buffer_index = 0;
        uart_buffer[0] = '\0';
    }

    // Command computation
    if (local_event & CMD_RDY) {
        clr_evt(CMD_RDY);

        if (uart_buffer EQ '\0') {
            set_error(CMD_ERROR);
            return;
        }

        // Reset dict_ptr and set error
        if (dict_ptr->key EQ '\0') {
            dict_ptr = &OBSERVER_FUNC_DICT[0];
            set_error(CMD_ERROR);
            return;
        }

        // Compare buffer with dict entry
        if (strncmp(uart_buffer, dict_ptr->key, 3) == 0) {
            // Execute function
            int (*func_ptr)(void) = (int(*)(void))dict_ptr->func;
            if (func_ptr) {
                func_ptr();
            } else {
                set_error(CMD_ERROR);
            }

            // Reset buffer and pointer
            buffer_index = 0;
            uart_buffer[0] = '\0';
            dict_ptr = &OBSERVER_FUNC_DICT[0];
            return;
        }

        dict_ptr++;

    }
}
