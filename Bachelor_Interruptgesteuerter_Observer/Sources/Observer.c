/*
 * Observer.c
 *
 *  Created on: 26.03.2025
 *      Author: Julian Rapp
 */

#include <msp430.h>
#include <msp430fr5729.h>
#include <string.h>
#include <stdlib.h>
#include <..\base.h>
#include <Observer.h>

// Zähllänge der Timer
#define ISR_TIMER 96
#define TIME_OUT 57600

// UART Einstellungen
#define UART_BUFFER_SIZE 64
#define TIMEOUT_THRESHOLD 600  // 6s = 600 * 10ms

// UART Logic
static char uart_buffer[UART_BUFFER_SIZE];
static int buffer_index = 0;
static Bool timeout_occurred = 0;
static Bool command_ready = 0;
static volatile int timeout_counter = 0;

/*
 * Initialization
 */
#pragma FUNC_ALWAYS_INLINE(Observer_init)
GLOBAL Void Observer_init(Void) {
    /*
     * Initialize Clock (Interrupt Timer & Time-Out Timer) TA2
     */
    TA2CTL = MC__STOP               // stop timer
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

    TA2CCTL0 = CCIE;                // clear timer A2 control register 0 and set it to Compare Mode

    TA2CCR0 = ISR_TIMER;            // compare register interrupt-timer
    TA2EX0 = TBIDEX_7;              // divide by 8

    /*
     * Time-Out-Timer
     *
     * ACLK = 614.4kHz
     * min. Timer = 6s
     * 614.4kHz * 6s / 8 / 8 = 57.600
     * max count to 65.535 (16-bit)
     *
     * Capture-Mode rising and falling edge
     */
/*
    TA2CCTL1 = CCIE;                // clear timer A2 control register 1 and set it to Compare Mode
    TA2CCTL1 = CAP                  // enable capture-mode
                | CM_3;             // capture-mode rising & falling edge

    TA2CCR1 = 57600;                // compare register time-out-timer
    TA2EX1 = TBIDEX_7;              // divide by 8
*/
    /*
     * Set and Start Timer A
     */
    TA2CTL = TASSEL__ACLK           // set to ACLK (614.4kHz)
            | ID__8                 // input divider set to /8
            | MC__UP                // mode control, count continuously upwards
            | TAIE;                 // enable timer interrupt


    /*
     * Initialize UART
     */
    SETBIT(UCA2CTLW0, UCSWRST);  // UCA2 in Software-Reset versetzen

    UCA2CTLW1 = 0x0002;          // Entprellzeit ~100ns
    UCA2BRW   = 4;               // Baudraten-Prescaler für 9600 Baud (614,4kHz / (16*9600) = 4)
    UCA2MCTLW = 0x20 << 8        // zweite Modulationsstufe
              | 0x00 << 4        // erste Modulationsstufe
              | UCOS16;          // 16-faches Oversampling aktivieren

    UCA2CTLW0 = UCPEN            // Parität aktivieren
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

    UCA2IE    = 0                // Transmit Complete Interrupt deaktivieren
              | 0                // Start Bit Interrupt deaktivieren
              | 0                // Transmit Interrupt deaktivieren
              | UCRXIE;          // Empfangs-Interrupt aktivieren

    buffer_index = 0;
    uart_buffer[0] = '\0';

}

// Compare Strings
#pragma FUNC_ALWAYS_INLINE(string_compare)
LOCAL int string_compare(const char *str1, const char *str2, int len) {
    int i;
    for (i = 0; i < len; i++) {
        if (str1[i] != str2[i] || str1[i] == '\0' || str2[i] == '\0')
            return 0;
    }
    return 1;
}

// Print chars to UART-connection
#pragma FUNC_ALWAYS_INLINE(Observer_print)
LOCAL int Observer_print(const char * str) {
    while (*str) {
        while (!(UCA2IFG & UCTXIFG));  // Warten, bis TX-Buffer leer ist
        UCA2TXBUF = *str++;            // Zeichen senden
    }
    return 0;
}

// Reads from memory cell(s)
#pragma FUNC_ALWAYS_INLINE(read_mem)
LOCAL int read_mem(const int str, const int addr, const int blocks) {

}

// Writes to memory cell(s)
#pragma FUNC_ALWAYS_INLINE(write_mem)
LOCAL int write_mem(const int str, const int addr, const int blocks) {

}

// Set interrupt breakpoint
#pragma FUNC_ALWAYS_INLINE(interrupt)
LOCAL int interrupt() {

}

/*
 * UART ISR
 */
#pragma vector = USCI_A2_VECTOR
__interrupt Void USCI_A2_ISR(Void) {
    switch(__even_in_range(UCA2IV, USCI_UART_UCTXCPTIFG)) {
        case USCI_NONE:
            // TODO: Fehlerausgabe
            break;
        case USCI_UART_UCRXIFG:     // Data ready to read
            // Read and save byte
            char rx_byte = UCA2RXBUF;

            // Reset Timeout-Timer
            timeout_counter = 0;

            // Check on end of word
            if (rx_byte == '\r' || rx_byte == '\n') {
                uart_buffer[buffer_index] = '\0';   // Terminate string
                command_ready = 1;                  // Command ready
            }
            // Save chars in Buffer if space available
            else if (buffer_index < UART_BUFFER_SIZE - 1) {
                uart_buffer[buffer_index++] = rx_byte;
            }
            break;
        default:
            // TODO: Fehlerausgabe
            break;
    }
}

/*
 * Command ISR
 */
#pragma vector = TIMER2_A1_VECTOR
__interrupt Void TIMER2_A1_ISR(Void) {

    // UART Time-Out counter logic
    timeout_counter++;

    if (timeout_counter >= TIMEOUT_THRESHOLD) {
        timeout_counter = 0;  // Reset counter
        timeout_occurred = 1;
    }

    // Command computation
    if (command_ready == 1) {
        command_ready = 0;

        for (int i = 0; i < OBSERVER_FUNC_COUNT; i++) {

            if (string_compare(uart_buffer, OBSERVER_FUNC_DICT[i].key, 3) == 1){

                char *params = uart_buffer + 3;

                char *token = strtok(params, " ");
                if (token) int addr = (int)strtol(token, NULL, 16);  // Hexadezimal
                token = strtok(NULL, " ");
                if (token) int blocks = (int)strtol(token, NULL, 10); // Dezimal

                if (i = 0) {
                    int (*func_ptr)(const int, const int, const int) = (int (*)(const int, const int, const int))OBSERVER_FUNC_DICT[i].func;
                    (*func_ptr)(0, addr, blocks);
                } else if (i = 1) {
                    int (*func_ptr)(const int, const int, const int) = (int (*)(const int, const int, const int))OBSERVER_FUNC_DICT[i].func;
                    (*func_ptr)(0, addr, blocks);
                } else if (i = 2) {
                    int (*func_ptr)() = (int (*)())OBSERVER_FUNC_DICT[i].func;
                    (*func_ptr)();
                }
            }
        }
    }

}

