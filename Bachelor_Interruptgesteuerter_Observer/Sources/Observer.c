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
#define RW_BUFFER_SIZE 64
#define MASK (CMD_RDY | CMD_RUN | RST | UART_ERR | CMD_ERR)

// Event/Error Logic
LOCAL TEvt global_events;
LOCAL UChar uart_error;
LOCAL UChar cmd_error;

/*
// Function Logic
static ObserverFuncEntry OBSERVER_FUNC_DICT[] = {
    {"rdm", (void*)read_mem},       // read_mem function
    {"wrm", (void*)write_mem},      // write_mem function
    {"inr", (void*)set_interrupt},  // interrupt function
    {"\0", NULL}
};

ObserverFuncEntry* dict_ptr;

LOCAL Int (*func_ptr)(void);
LOCAL UInt num_block;
LOCAL UInt blocks;
LOCAL Char *mem_addr_ptr;
LOCAL Char rw_buffer[RW_BUFFER_SIZE + 1];
*/

// UART Logic
LOCAL const Char* print_ptr;
LOCAL const Char newline[4] = {'\n','\r','>','\0'};
LOCAL const Char backspace[4] = {'\b',' ','\b','\0'};
LOCAL Char uart_buffer[UART_BUFFER_SIZE + 1];
LOCAL Char rx_byte;
LOCAL Char buffer_index;
LOCAL Int timeout_counter;



/*
 * Initialization
 */
#pragma FUNC_ALWAYS_INLINE(Observer_init)
GLOBAL Void Observer_init(Void) {
    /*
     * Initialize Clock (Interrupt Timer & Time-Out Timer) TA2
     */

    TB0CTL = MC__STOP               // stop timer
                | TBCLR;            // clear counter

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

    TB0CTL = TBSSEL__ACLK           // set to ACLK (614.4kHz)
            | ID__8                 // input divider set to /8
            | MC__UP                // mode control, count continuously upwards
            | TBIE;                 // enable timer interrupt


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

    global_events = NO_EVTS;
    uart_error = NO_ERR;
    cmd_error = NO_ERR;
/*
    dict_ptr = &OBSERVER_FUNC_DICT[0];

    num_block = 0;
    blocks = 0;
*/
    timeout_counter = 0;
    buffer_index = 0;
    uart_buffer[0] = '\0';
    rx_byte = '\0';

    observer_print(newline);

}

/*
 * Event & Error Handling Logic
 */
#pragma FUNC_ALWAYS_INLINE(set_uart_error)
LOCAL Void set_uart_error(UChar err) {
  if (err == NO_ERR || uart_error > err) {
    uart_error = err;
    TGLBIT(global_events, UART_ERR);
  }
}

#pragma FUNC_ALWAYS_INLINE(set_cmd_error)
LOCAL Void set_cmd_error(UChar err) {
  if (err == NO_ERR || cmd_error > err) {
    cmd_error = err;
    TGLBIT(global_events, CMD_ERR);
  }
}

// Print chars to UART-connection
#pragma FUNC_ALWAYS_INLINE(observer_print)
LOCAL int observer_print(const char * str) {
    if (str EQ NULL) {
        set_uart_error(PRINT_ERROR);
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


// Writes to memory cell(s)
#pragma FUNC_ALWAYS_INLINE(write_mem)
LOCAL int read_mem(void) {
/*
    if (num_block < RW_BUFFER_SIZE || num_block > (blocks * 8)) {
        rw_buffer[num_block++] = '\0';
        observer_print(rw_buffer);
        TGLBIT(global_events, RST);
    }

    if (blocks == 0) {
        // Extract useful arguments
        Char *mem_addr_str = strtok(uart_buffer + 4, " ");
        Char *block_str = strtok(NULL, " ");

        mem_addr_ptr = (Char *)strtol(mem_addr_str, NULL, 0);
        blocks = (UInt)atoi(block_str);
    }

    rw_buffer[num_block] = *((volatile Char *)mem_addr_ptr + (num_block*8));
    num_block++;

    TGLBIT(global_events, CMD_RUN);
*/
    return 0;
}

// Reads from memory cell(s)
#pragma FUNC_ALWAYS_INLINE(read_mem)
LOCAL int write_mem(void) {
/*
    if (num_block < RW_BUFFER_SIZE || num_block > (blocks * 8)) {
        observer_print(rw_buffer);
        TGLBIT(global_events, RST);
    }

    if (blocks == 0) {
        // Extract useful arguments
        Char *mem_addr_str = strtok(uart_buffer + 4, " ");
        mem_addr_ptr = (Char *)strtol(mem_addr_str, NULL, 0);

        Char *token = strtok(NULL, " ");
        if (token != NULL) {
            strncpy(rw_buffer, token, RW_BUFFER_SIZE - 1);
            rw_buffer[buffer_index] = '\0';
        }

        blocks = (strlen(rw_buffer) + 7) / 8;
    }

    *((volatile Char *)mem_addr_ptr + (num_block*8)) = rw_buffer[num_block];
    num_block++;
*/
    return 0;
}

// Set interrupt breakpoint
#pragma FUNC_ALWAYS_INLINE(set_interrupt)
LOCAL int set_interrupt(void) {


    // Some Code goes in there


    return 0;
}

/*
 * UART ISR
 */
#pragma vector = USCI_A0_VECTOR
__interrupt Void UCA0_ISR(Void) {

    switch(__even_in_range(UCA0IV, USCI_UART_UCTXIFG)) {

        case USCI_NONE:             // Interrupt Vector 0: No Interrupt pending
            break;

        case USCI_UART_UCRXIFG:     // Interrupt Vector 2: Data ready to read

            if (TSTBIT(UCA0STATW, UCBRK)) {
               set_uart_error(BREAK_ERROR);
               Char ch = UCA0RXBUF;
               return;
            }

            if (TSTBIT(UCA0STATW, UCRXERR)) {
               set_uart_error(FROVPAR_ERROR);
               Char ch = UCA0RXBUF;
               return;
            }

            // Read and save byte
            rx_byte = UCA0RXBUF;
            //UCA0TXBUF = rx_byte;

            // Reset Timeout-Timer
            timeout_counter = 0;

            if (rx_byte EQ '\n') {
                return;
            }

            // Checks if rx_byte is a alphanumeric letter
            if (!isalnum(rx_byte)
                    AND rx_byte NE ' '
                    AND rx_byte NE '\r'
                    AND rx_byte NE '\b') {
                set_uart_error(CHARACTOR_ERROR);
                return;
            }

            // Check backspace
            if (rx_byte EQ '\b'
                    AND buffer_index GT 0) {
                // Delete last character
                uart_buffer[buffer_index--] = ' ';
                uart_buffer[buffer_index] = '\0';
                // Backspace
                observer_print(backspace);
                return;
            }

            // Check on end of word
            if (rx_byte EQ '\r' ) {
                observer_print(newline);
                TGLBIT(global_events, CMD_RDY);       // Command ready
                return;
            }

            if (buffer_index GE UART_BUFFER_SIZE) {
                set_uart_error(BUFFER_ERROR);
                buffer_index = 0;
                return;
            }

            // Save chars in Buffer if space available
            uart_buffer[buffer_index++] = rx_byte;
            uart_buffer[buffer_index] = '\0';

            // Echo
            observer_print(&uart_buffer[buffer_index - 1]);

            __low_power_mode_off_on_exit();

            break;

        case USCI_UART_UCTXIFG:     // Interrupt Vector 4: Data ready to write

            // USCI Break received
            if (TSTBIT(UCA0STATW, UCBRK)) {
               Char ch = UCA0RXBUF;
               set_uart_error(BREAK_ERROR);
               return;
            }

            // USCI RX Error Flag
            if (TSTBIT(UCA0STATW, UCRXERR)) {
               Char ch = UCA0RXBUF;
               set_uart_error(FROVPAR_ERROR);
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
            set_uart_error(NO_ERR);

            // Enable UART Receive Interrupt
            SETBIT(UCA0IE, UCRXIE);
            break;

    }
}


/*
 * Command ISR
 */

#pragma vector = TIMER0_B1_VECTOR
__interrupt Void TIMER0_B1_ISR(Void) {

    // UART Time-Out counter logic
    timeout_counter++;

    // Get Events
    TEvt local_event = global_events & MASK;
    CLRBIT(global_events, MASK);

    // Error Handling
    if (local_event & UART_ERR) {
        observer_print((uart_error == TIME_OUT)         ? "#A\n"    // time out
                    :  (uart_error == BUFFER_ERROR)     ? "#B\n"    // buffer error (e.g. to many bytes received)
                    :  (uart_error == CHARACTOR_ERROR)  ? "#C\n"    // character error (e.g. wrong charactor received)
                    :  (uart_error == FROVPAR_ERROR)    ? "#D\n"    // frame overrun or parity error
                    :  (uart_error == BREAK_ERROR)      ? "#E\n"    // break error (lost communication)
                    :  (uart_error == PRINT_ERROR)      ? "#F\n"    // unable to print on UART
                    : "\n" );
        TGLBIT(global_events, RST);
    }

    if (local_event & CMD_ERR) {
        observer_print((cmd_error == NO_CMD)            ? "#1\n"    // no command to compute
                    :  (cmd_error == UNKNOWN_CMD)       ? "#2\n"    // unknown command
                    :  (cmd_error == INV_PTR)           ? "#3\n"    // Invalid function pointer
                    :  (cmd_error == INV_ADDR)          ? "#4\n"    // Invalid memory address
                    :  (cmd_error == INV_BLCK)          ? "#5\n"    // Invalid block-size
                    :  (cmd_error == INV_STR)           ? "#6\n"    // Invalid string
                    : "\n" );
        TGLBIT(global_events, RST);
    }

    // Timeout Handling
    if (timeout_counter >= TIMEOUT_THRESHOLD) {
        timeout_counter = 0;  // Reset counter
        set_uart_error(TIME_OUT);
        buffer_index = 0;
        uart_buffer[0] = '\0';
    }
/*
    // Command computation
    if (local_event & CMD_RDY) {

        if (uart_buffer EQ '\0') {
            TGLBIT(global_events, CMD_RDY);
            set_cmd_error(NO_CMD);
            return;
        }

        // Reset dict_ptr and set error
        if (dict_ptr->key EQ '\0') {
            TGLBIT(global_events, CMD_RDY);
            dict_ptr = &OBSERVER_FUNC_DICT[0];
            set_cmd_error(UNKNOWN_CMD);
            return;
        }

        // Compare buffer with dict entry
        if (strncmp(uart_buffer, dict_ptr->key, 3) == 0) {
            TGLBIT(global_events, CMD_RDY);
            // Extract functionpointer
            func_ptr = (int(*)(void))dict_ptr->func;
            TGLBIT(global_events, CMD_RUN);
            return;
        }
        dict_ptr++;
    }

    if (local_event & CMD_RUN) {
        // Check if function pointer is available
        if (!func_ptr) {
            set_cmd_error(INV_PTR);
        }
        // execute function routine
        func_ptr();
    }

    if (local_event & RST) {
        // Display Memory Content
        observer_print(rw_buffer);

        // Reset buffer and pointer
        blocks = 0;
        rw_buffer[0] = '\0';

        buffer_index = 0;
        uart_buffer[0] = '\0';
        dict_ptr = &OBSERVER_FUNC_DICT[0];
        return;
    }
*/
}

