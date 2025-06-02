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

// Z�hll�nge Timer
#define ISR_TIMER (96 - 1)

// Observer Function Dictionary Length
#define OBS_FUNCT_CMDS (3 + 1)

#define RW_BUFF_SIZE (2 + 1)

// UART Properties
#define UART_BUFFER_SIZE (64 + 1)
#define TIMEOUT_THRESHOLD 1500  // 15s = 1500 * 10ms

// Software Debugger Constants
//#define BREAKPOINT_INSTRUCTION_CALL 0x3C00  // Opcode for JMP #absolute_addr

// Event/Error Logic
LOCAL TEvt global_events;
LOCAL Char uart_error;
LOCAL Char cmd_error;

/*
 * State Machine
 */
LOCAL Void (*state_ptr)(Void);
LOCAL Void state_dummy(Void);
LOCAL Void state_0(Void);
LOCAL Void state_1(Void);
LOCAL Void state_2(Void);


/*
 * Main-Functionality Logic
 */

LOCAL Void read_mem(Void);              // Reads from memory cell(s)
LOCAL Void write_mem(Void);             // Writes to memory cell(s)
/*
LOCAL Void set_breakpoint(Void);        // Set breakpoint
LOCAL Void cancel_breakpoint(Void);   // Continue breakpoint
LOCAL Void breakpoint_handler(Void);    // Continue breakpoint
*/

// Function Logic
LOCAL const ObserverFuncEntry Observer_func_dict[OBS_FUNCT_CMDS] = {
    {"rdm", read_mem},              // read_mem function
    {"wrm", write_mem},             // write_mem function
    //{"sbr", set_breakpoint},        // set breakpoint
    //{"cbr", cancel_breakpoint},   // continue breakpoint
    {"", NULL}
};

LOCAL UInt dict_idx;

/*
// Software Debugger
LOCAL UInt *brp_addr_ptr;                                          // Address of the breakpoint
LOCAL UInt brp_og_cmd_1;
LOCAL UInt brp_og_cmd_2;
*/

// Buffer and Pointer
LOCAL UInt mem_addr_idx;
LOCAL UInt blocks;
LOCAL Char *mem_addr_ptr;
LOCAL Char *write_str_ptr;
LOCAL Char rw_buf[RW_BUFF_SIZE];
LOCAL Char *rw_buf_ptr;

/*
 * UART Logic
 */

LOCAL const Char new_line[4] = {'\n', '\r', '>', '\0'};
LOCAL const Char* print_ptr;
LOCAL Char uart_buffer[UART_BUFFER_SIZE];
LOCAL Char *uart_buffer_ptr;
LOCAL Char rx_byte;
LOCAL Char buffer_index;
LOCAL Int timeout_counter;


/*
 * Initialization
 */
#pragma FUNC_ALWAYS_INLINE(Observer_init)
GLOBAL Void Observer_init(Void) {
    /*
     * Initialize Clock (Interrupt Timer & Time-Out Timer) TB0
     */

    CLRBIT(TB0CTL, MC__STOP     // stop mode
                   | MC__UP     // no up-mode
                   | TBIE       // disable interrupt
                   | TBIFG);    // clear interrupt flag

    /*
     * Interrupt Timer
     *
     * ACLK = 614.4kHz
     * min. Timer = 10ms
     * 614.4kHz * 10ms = 6144
     * 6144 / 8 / 8 = 96
     *
     * UP-Mode
     */


    CLRBIT(TB0CCTL0, CM1 | CM0      // no capture mode
                   | CAP            // compare mode
                   | CCIE           // disable interrupt
                   | CCIFG);        // clear interrupt flag

    TB0CCR0 = ISR_TIMER;            // compare register interrupt-timer
    TB0EX0 = TBIDEX__8;             // divide by 8

    /*
     * Set and Start Timer B
     */

    TB0CTL = TBSSEL__ACLK           // set to ACLK (614.4kHz)
            | ID__8                 // input divider set to /8
            | MC__UP                // mode control, count continuously upwards
            | TBCLR;

    SETBIT(TB0CTL, TBIE             // enable timer interrupt
            | TBIFG);               // set interrupt flag


    /*
     * Initialize UART
     */
    SETBIT(UCA0CTLW0, UCSWRST);  // UCA0 in Software-Reset versetzen

    UCA0CTLW1 = 0x0002;          // Entprellzeit ~100ns
    UCA0BRW   = 4;               // Baudraten-Prescaler f�r 9600 Baud (614,4kHz / (16*9600) = 4)
    UCA0MCTLW = 0x20 << 8        // zweite Modulationsstufe
              | 0x00 << 4        // erste Modulationsstufe
              | UCOS16;          // 16-faches Oversampling aktivieren

    UCA0CTLW0 = UCPEN            // Parit�t aktivieren
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

    UCA0IE    = 0                // Transmit Complete Interrupt deaktivieren
              | 0                // Start Bit Interrupt deaktivieren
              | 0                // Transmit Interrupt deaktivieren
              | UCRXIE;          // Empfangs-Interrupt aktivieren

    global_events = NO_EVTS;
    uart_error = NO_ERR;
    cmd_error = NO_ERR;

    dict_idx = 0;

    mem_addr_ptr = NULL;
    mem_addr_idx = 0;
    blocks = 0;
    rw_buf_ptr = rw_buf;
    *(rw_buf_ptr + 1) = '\0';
    *(rw_buf_ptr + 2) = '\0';

    /*
    // Initialize breakpoint structures
    brp_addr_ptr = NULL;
    brp_og_cmd_1 = 0x0000;
    brp_og_cmd_2 = 0x0000;
    */

    timeout_counter = 0;
    buffer_index = 0;
    uart_buffer_ptr = uart_buffer;
    *uart_buffer_ptr = '\0';
    rx_byte = '\0';

    state_ptr = &state_dummy;

    observer_print(new_line);

}

/*
 * Event & Error Handling Logic
 */

#pragma FUNC_ALWAYS_INLINE(set_uart_error)
LOCAL Void set_uart_error(Char err) {
  if (err == NO_ERR || uart_error < err) {
    uart_error = err;
    SETBIT(global_events, UART_ERR);
  }
}

#pragma FUNC_ALWAYS_INLINE(set_cmd_error)
LOCAL Void set_cmd_error(Char err) {
  if (err == NO_ERR || cmd_error < err) {
    cmd_error = err;
    SETBIT(global_events, CMD_ERR);
  }
}

// Print chars to UART-connection
#pragma FUNC_ALWAYS_INLINE(observer_print)
LOCAL int observer_print(const char * str) {
    if (str EQ NULL) {
        set_uart_error(PRINT_ERROR);
        return -1;
    }
    SETBIT(global_events, WRT_UART);
    print_ptr = str;
    SETBIT(UCA0IFG, UCTXIFG);   // UART Transmit Interrupt Flag
    SETBIT(UCA0IE,  UCTXIE);    // Enable UART Receive Interrupt
    return 0;
}

/*
// Blocking receive UART-Char Routine
LOCAL Void observer_getchar_blocking(Void) {
    while (!TSTBIT(UCA0IFG, UCRXIFG)) {}

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

    // Read character
    rx_byte = UCA0RXBUF;

    return;
}
*/

/*
 * Main-Functionality Logic
 */

// Reads from memory cell(s)
#pragma FUNC_ALWAYS_INLINE(read_mem)
LOCAL Void read_mem(Void) {

    //SETBIT(P3OUT, BIT4);
    if (TSTBIT(global_events & WRT_UART, WRT_UART)) {
        //CLRBIT(P3OUT, BIT4);
        return;
    }

    if (mem_addr_idx > (blocks - 1)) {
        state_ptr = &state_2;
        //CLRBIT(P3OUT, BIT4);
        return;
    }

    if (mem_addr_ptr EQ NULL) {
        // Extract useful arguments
        Char *mem_addr_str = strtok(uart_buffer_ptr + 4, " ");
        Char *block_str = mem_addr_str + strlen(mem_addr_str) + 1;

        mem_addr_ptr = (Char *)strtol(mem_addr_str, NULL, 0);
        blocks = (UInt)atoi(block_str);
    }

    *rw_buf_ptr = *((volatile Char *)mem_addr_ptr + mem_addr_idx);
    if (!IS_ALNUM(*rw_buf_ptr)) {
        *rw_buf_ptr = ' ';
    }

    print_ptr = rw_buf_ptr;
    SETBIT(global_events, WRT_UART);
    SETBIT(UCA0IFG, UCTXIFG);   // UART Transmit Interrupt Flag
    SETBIT(UCA0IE,  UCTXIE);    // Enable UART Receive Interrupt

    mem_addr_idx++;

    //CLRBIT(P3OUT, BIT4);
}

// Writes to memory cell(s)
#pragma FUNC_ALWAYS_INLINE(write_mem)
LOCAL Void write_mem(Void) {

    //SETBIT(P3OUT, BIT4);
    if (TSTBIT(global_events & WRT_UART, WRT_UART)) {
        //CLRBIT(P3OUT, BIT4);
        return;
    }

    if (mem_addr_ptr EQ NULL) {
        // Extract useful arguments
        Char *mem_addr_str = strtok(uart_buffer_ptr + 4, " ");
        write_str_ptr = mem_addr_str + strlen(mem_addr_str) + 1;

        mem_addr_ptr = (Char *)strtol(mem_addr_str, NULL, 0);
    }

    if (between("0x000000", mem_addr_ptr, "0x0001FF")
            || between("0x000C00", mem_addr_ptr, "0x000FFF")) {
        set_cmd_error(INV_ADDR);
        return;
    }

    *rw_buf_ptr = *((volatile Char *)write_str_ptr + mem_addr_idx);

    if (*rw_buf_ptr EQ '\0') {
        state_ptr = &state_2;
        //CLRBIT(P3OUT, BIT4);
        return;
    }

    if (!IS_ALNUM(*rw_buf_ptr)) {
        *rw_buf_ptr = ' ';
    }

    print_ptr = rw_buf_ptr;
    SETBIT(global_events, WRT_UART);
    SETBIT(UCA0IFG, UCTXIFG);   // UART Transmit Interrupt Flag
    SETBIT(UCA0IE,  UCTXIE);    // Enable UART Receive Interrupt

    *((volatile Char *)mem_addr_ptr + mem_addr_idx) = *rw_buf_ptr;
    mem_addr_idx++;

    //CLRBIT(P3OUT, BIT4);
}

/*
// Set breakpoint
#pragma FUNC_ALWAYS_INLINE(set_breakpoint)
LOCAL Void set_breakpoint(Void) {

    Char *mem_addr_str = strtok(uart_buffer_ptr + 4, " ");
    brp_addr_ptr = (UInt *)strtol(mem_addr_str, NULL, 0);

    if (brp_addr_ptr EQ NULL || (*brp_addr_ptr % 2 NE 0)) {
        set_cmd_error(INV_ADDR);
        return;
    }

    // --- Start Critical Section ---
    unsigned short status_register = __get_SR_register();
    __disable_interrupt();

    // Save original instruction bytes (4 bytes for CALL #addr)
    brp_og_cmd_1 = *(volatile UInt *)brp_addr_ptr;
    brp_og_cmd_2 = *((volatile UInt *)brp_addr_ptr + 1);

    // Write the breakpoint instruction (CALL #breakpoint_handler)
    *(volatile UInt *)brp_addr_ptr = BREAKPOINT_INSTRUCTION_CALL;       // Write opcode
    *((volatile UInt *)brp_addr_ptr + 1) = (UInt)&breakpoint_handler;   // Write handler address

    // --- End Critical Section ---
    __set_interrupt_state(status_register);

    return;
}

// Continue breakpoint
#pragma FUNC_ALWAYS_INLINE(cancel_breakpoint)
LOCAL Void cancel_breakpoint(Void) {

    return;
}

// Handle breakpoint
#pragma FUNC_ALWAYS_INLINE(breakpoint_handler)
LOCAL Void breakpoint_handler(Void) {


    // --- Start Critical Section ---
    unsigned short status_register = __get_SR_register();
    __disable_interrupt();

    UInt return_addr;
    return_addr = __get_SP_register();          // Get return address from stack R1 (Stack Pointer)

    while(rx_byte != 'c') {

        observer_getchar_blocking();

        // Handling Read and Write Memory

    }

    *(volatile UInt*)return_addr = brp_og_cmd_1;
    *((volatile UInt*)return_addr + 1) = brp_og_cmd_2;

    // *(volatile UInt*)brp_addr_ptr = brp_og_cmd_1;
    // *((volatile UInt*)brp_addr_ptr + 1) = brp_og_cmd_2;


    // --- End Critical Section ---
    __set_interrupt_state(status_register);

    return; // RETA to top of stack (Address from SP)
}
*/

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

            // Reset Timeout-Timer
            timeout_counter = 0;

            if (rx_byte EQ '\n') {
                return;
            }

            // Check on end of word
            if (rx_byte EQ '\r' ) {
                observer_print(new_line);
                state_ptr = &state_1;
                SETBIT(TB0CTL, TBIFG);      // Set ISR Interrupt Flag
                return;
            }

            // Check backspace
            if (rx_byte EQ '\b'
                    AND buffer_index GT 0) {
                // Delete last character
                *(uart_buffer_ptr + --buffer_index) = '\0';
                // Backspace
                observer_print("\b \b");
                return;
            }

            // Checks if rx_byte is a alphanumeric letter
            if (!IS_ALNUM(rx_byte)
                    AND rx_byte NE ' ') {
                observer_print("?");
                return;
            }

            if (buffer_index GE UART_BUFFER_SIZE) {
                set_uart_error(BUFFER_ERROR);
                buffer_index = 0;
                return;
            }

            // Save chars in Buffer if space avaiable
            *(uart_buffer_ptr + buffer_index++) = rx_byte;
            *(uart_buffer_ptr + buffer_index) = '\0';

            // Echo
            observer_print(uart_buffer_ptr + (buffer_index - 1));

            // Set Time-Out-Flag
            if (state_ptr EQ &state_dummy) {
                state_ptr = &state_0;
            }
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

            // Clear Write to UART Flag
            CLRBIT(global_events, WRT_UART);

            // Disable UART Transmit Interrupt
            CLRBIT(UCA0IE, UCTXIE);
            Char ch = UCA0RXBUF;

            // Enable UART Receive Interrupt
            SETBIT(UCA0IE, UCRXIE);

            break;

    }
}


/*
 * Command ISR
 */

LOCAL Void state_dummy(Void) {}

LOCAL Void state_0(Void) {

    timeout_counter++;

    // Timeout Handling
    if (timeout_counter >= TIMEOUT_THRESHOLD) {
        timeout_counter = 0;  // Reset counter
        set_uart_error(TIME_OUT);
        state_ptr = &state_2;
    }
}

LOCAL Void state_1(Void) {
    //SETBIT(P3OUT, BIT4);

    if (*uart_buffer_ptr EQ '\0') {
        state_ptr = &state_2;
        CLRBIT(P3OUT, BIT4);
        return;
    }

    // Reset dict_ptr and set error
    if (Observer_func_dict[dict_idx].func EQ NULL) {
        dict_idx = 0;
        set_cmd_error(UNKNOWN_CMD);
        CLRBIT(P3OUT, BIT4);
        return;
    }

    // Compare buffer with dict entry
    if (strncmp(uart_buffer, Observer_func_dict[dict_idx].key, 3) == 0) {
        // Extract functionpointer
        state_ptr = Observer_func_dict[dict_idx].func;
        return;
    }
    dict_idx++;

    //CLRBIT(P3OUT, BIT4);
}

// Reset buffer and pointer
LOCAL Void state_2(Void) {

    SETBIT(P3OUT, BIT4);
    if (!TSTBIT(global_events & WRT_UART, WRT_UART)) {
        mem_addr_ptr = 0;

        blocks = 0;
        mem_addr_idx = 0;
        *rw_buf_ptr = ' ';
        *(rw_buf_ptr + 1) = '\0';

        buffer_index = 0;
        *uart_buffer_ptr = '\0';
        dict_idx = 0;

        state_ptr = &state_dummy;
    }
    CLRBIT(P3OUT, BIT4);
}

#pragma vector = TIMER0_B1_VECTOR
__interrupt Void TIMER0_B0_ISR(Void) {

    CLRBIT(TB0CTL, TBIFG);

    // Error Handling
    if (TSTBIT(global_events & UART_ERR, UART_ERR)) {
        *rw_buf_ptr = '#';
        *(rw_buf_ptr + 1) = uart_error;
        observer_print(rw_buf_ptr);
        uart_error = NO_ERR;
        CLRBIT(global_events, UART_ERR);
        state_ptr = &state_2;
    }

    if (TSTBIT(global_events & CMD_ERR, CMD_ERR)) {
        *rw_buf_ptr = '#';
        *(rw_buf_ptr + 1) = cmd_error;
        observer_print(rw_buf_ptr);
        cmd_error = NO_ERR;
        CLRBIT(global_events, CMD_ERR);
        state_ptr = &state_2;
    }

    // execute main function routine
    //SETBIT(P3OUT, BIT4);
    state_ptr();
    //CLRBIT(P3OUT, BIT4);
}
