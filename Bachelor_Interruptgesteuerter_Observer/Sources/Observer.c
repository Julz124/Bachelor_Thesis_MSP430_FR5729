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
LOCAL TEvent event = NO_EVENTS;
LOCAL TEvent errflg = NO_EVENTS;
LOCAL UChar error = NO_ERROR;


// Function Logic
static ObserverFuncEntry OBSERVER_FUNC_DICT[] = {
    {"rdm", (void*)read_mem},     // read_mem function
    {"wrm", (void*)write_mem},    // write_mem function
    {"inr", (void*)interrupt},    // interrupt function
    {"\0", NULL}
};

ObserverFuncEntry* dict_ptr = &OBSERVER_FUNC_DICT[0];

// UART Logic
static int command_ready = 0;
static int timeout_counter = 0;

static char uart_buffer[UART_BUFFER_SIZE];
static int buffer_index = 0;
static const char * print_ptr;

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
    SETBIT(UCA2CTLW0, UCSWRST);  // UCA2 in Software-Reset versetzen

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

    buffer_index = 0;
    uart_buffer[0] = '\0';

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
 * Event & Error Handling Logic
 */
#pragma FUNC_ALWAYS_INLINE(set_error)
LOCAL void set_error(UChar err) {
  if (err == NO_ERROR || error > err) {
    error = err;
    set_evt(EVENT_ERR);
  }
}

#pragma FUNC_ALWAYS_INLINE(clr_evt)
GLOBAL Void clr_evt(TEvent arg) {
   TGLBIT(event, arg);
}

#pragma FUNC_ALWAYS_INLINE(Event_tst)
GLOBAL Bool tst_evt(TEvent arg) {
   return TSTBIT(event, arg);
}

#pragma FUNC_ALWAYS_INLINE(Event_err)
GLOBAL Bool err_evt(Void) {
   return (errflg NE NO_EVENTS);
}

#pragma FUNC_ALWAYS_INLINE(get_evts)
GLOBAL TEvent get_evt(TEvent mask) {
   TEvent tmp_event = event & mask;
   CLRBIT(event, mask);
   return tmp_event;
}

#pragma FUNC_ALWAYS_INLINE(Evt_Handler)
LOCAL void Evt_Handler() {
    if (get_evt(EVENT_ERR)) {
        observer_print((error == BREAK_ERROR)       ? 'lost communication'
                     : (error == FROVPAR_ERROR)     ? 'frame overrun or parity error'
                     : (error == CHARACTOR_ERROR)   ? 'wrong character received'
                     : (error == BUFFER_ERROR)      ? 'to many bytes received'
                     : (error == TIME_OUT)          ? 'time out'
                     : (error == PRINT_ERROR)       ? 'unable to print on UART');
    }
}

/*
 * Main-Functionality Logic
 */

// Reads from memory cell(s)
#pragma FUNC_ALWAYS_INLINE(read_mem)
LOCAL int read_mem(const int addr, const int blocks) {
    /*
     * Some Code goes in there
     */
    return 0;
}

// Writes to memory cell(s)
#pragma FUNC_ALWAYS_INLINE(write_mem)
LOCAL int write_mem(const int addr, const char* str) {
    /*
     * Some Code goes in there
     */
    return 0;
}

// Set interrupt breakpoint
#pragma FUNC_ALWAYS_INLINE(interrupt)
LOCAL int interrupt() {
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

    switch(__even_in_range(UCA2IV, USCI_UART_UCTXCPTIFG)) {

        case USCI_NONE:             // Interrupt Vector 0: No Interrupt pending
            break;

        case USCI_UART_UCRXIFG:     // Interrupt Vector 2: Data ready to read

            if (TSTBIT(UCA2STATW, UCBRK)) {
               set_error(BREAK_ERROR);
               Char ch = UCA0RXBUF;
               return;
            }

            if (TSTBIT(UCA2STATW, UCRXERR)) {
               set_error(FROVPAR_ERROR);
               Char ch = UCA0RXBUF;
               return;
            }

            // Read and save byte
            rx_byte = UCA2RXBUF;

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
                command_ready = 1;                  // Command ready
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
            if (TSTBIT(UCA2STATW, UCBRK)) {
               Char ch = UCA2RXBUF;
               set_error(BREAK_ERROR);
               return;
            }

            // USCI RX Error Flag
            if (TSTBIT(UCA2STATW, UCRXERR)) {
               Char ch = UCA2RXBUF;
               set_error(FROVPAR_ERROR);
               return;
            }

            // Transmit character
            if (*ptr NE '\0') {
               UCA2TXBUF = *print_ptr++;
               return;
            }

            // Disable UART Transmit Interrupt
            CLRBIT(UCA2IE, UCTXIE);
            Char ch = UCA2RXBUF;
            set_error(NO_ERROR);

            // Enable UART Receive Interrupt
            SETBIT(UCA2IE, UCRXIE);
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

    if (timeout_counter >= TIMEOUT_THRESHOLD) {
        timeout_counter = 0;  // Reset counter
        set_error(TIME_OUT);
        buffer_index = 0;
        uart_buffer[0] = '\0';
    }

    // Command computation
    if (command_ready == 1) {
        command_ready = 0;

        if (uart_buffer EQ '\0') {
            set_error(CMD_ERROR);
            return;
        }

        if (strncmp(uart_buffer, dict_ptr->key, 3) == 0) {

            /*
            char *params = uart_buffer + 3;

            if (i == 0) {
                char *token = strtok(params, " ");
                if (token) addr = (int)strtol(token, NULL, 16);
                token = strtok(NULL, " ");
                if (token) blocks = (int)strtol(token, NULL, 10);

                char* (*func_ptr)(const int, const int) = (char* (*)(const int, const int))OBSERVER_FUNC_DICT[i].func;
                Observer_print((*func_ptr)(addr, blocks));


            } else if (i == 1) {
                char *token = strtok(params, " ");
                if (token) addr = (int)strtol(token, NULL, 16);
                token = strtok(NULL, " ");
                if (token) str = token;

                char* (*func_ptr)(const int, const char*) = (char* (*)(const int, const char*))OBSERVER_FUNC_DICT[i].func;
                Observer_print((*func_ptr)(addr, str));


            } else if (i == 2) {
                char* (*func_ptr)() = (char* (*)())OBSERVER_FUNC_DICT[i].func;
                Observer_print((*func_ptr)());
            }
            */

            buffer_index = 0;
            uart_buffer[0] = '\0';
        }

    }
}

