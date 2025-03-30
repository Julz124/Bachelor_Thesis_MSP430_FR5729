/*
 * Observer.c
 *
 *  Created on: 26.03.2025
 *      Author: Julian Rapp
 */

#include <msp430.h>
#include <msp430fr5729.h>
#include <..\base.h>
#include <Observer.h>

// Z�hll�nge der Timer
#define ISR_TIMER 96
#define TIME_OUT 57600

// UART Einstellungen
#define UART_BUFFER_SIZE 64
#define BAUD_RATE 9600

// UART Logic
static char uart_buffer[UART_BUFFER_SIZE];
static int buffer_index = 0;
static Bool timeout_occurred = FALSE;
static Bool command_ready = FALSE;

#define TIMEOUT_THRESHOLD 600  // 6s = 600 * 10ms
static volatile int timeout_counter = 0;


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
     * TODO: Select Input signal CCIxA (see 11.3.3 User's Guide)
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
    UCA2BRW   = 6;               // Baudraten-Prescaler f�r 9600 Baud
    UCA2MCTLW = 0x20 << 8        // zweite Modulationsstufe
              | 0x00 << 4        // erste Modulationsstufe
              | UCOS16;          // 16-faches Oversampling aktivieren

    UCA2CTLW0 = UCPEN            // Parit�t aktivieren
              | UCPAR            // Gerade Parit�t (even)
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


// ISR f�r UART Empfang
#pragma vector = USCI_A2_VECTOR
__interrupt Void USCI_A2_ISR(Void) {
    switch(__even_in_range(UCA2IV, USCI_UART_UCTXCPTIFG)) {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:     // Empfangsdaten verf�gbar
            // Empfange Byte und speichere im Buffer
            char rx_byte = UCA2RXBUF;

            // Reset Timeout-Timer bei jedem empfangenen Zeichen
            timeout_counter = 0;    // Clear Timer

            // Pr�fe auf Zeilenende (Befehlsende)
            if (rx_byte == '\r' || rx_byte == '\n' || rx_byte ' ') {
                uart_buffer[buffer_index] = '\0';  // String terminieren
                command_ready = TRUE;              // Befehl ist vollst�ndig
                // Kein R�cksetzen des Buffer-Index hier, wird in der ISR gemacht
            }
            // Speichere Zeichen im Buffer, wenn noch Platz ist
            else if (buffer_index < UART_BUFFER_SIZE - 1) {
                uart_buffer[buffer_index++] = rx_byte;
            }
            break;
        default: break;
    }
}


#pragma vector = TIMER2_A1_VECTOR
__interrupt Void TIMER2_A1_ISR(Void) {

    // UART Time-Out Counter Logic
    timeout_counter++;

    if (timeout_counter >= TIMEOUT_THRESHOLD) {
        timeout_counter = 0;  // Reset Counter
        Timeout-timeout_occurred = TRUE;
    }

    // Befehlsverarbeitung
    if (command_ready){

    }

}

#pragma FUNC_ALWAYS_INLINE(Observer_print)
LOCAL int Observer_print(const char * str) {

}

#pragma FUNC_ALWAYS_INLINE(read_mem)
LOCAL int read_mem(const int str, const int addr, const int blocks) {

}

#pragma FUNC_ALWAYS_INLINE(write_mem)
LOCAL int write_mem(const int str, const int addr, const int blocks) {

}

#pragma FUNC_ALWAYS_INLINE(interrupt)
LOCAL int interrupt() {

}
