#include <msp430.h>
#include "UCA1.h"
#include "event.h"

// AS1108 Data Frame struct
// D15 D14 D13 D12 D11  D10  D9  D8   D7 D6 D5 D4 D3 D2 D1 D0
// X   X    X   X  Register Address   MSB <--- Data ----> LSB
// -----------------------------------------------------------

// Register Address
//                   Register Address Map
// Register       HEX Code              Address
//                              D15:D12 D11 D10 D9 D8
// No-Op              0xX0         X     0   0   0  0
// Digit 0            0xX1         X     0   0   0  1
// Digit 1            0xX2         X     0   0   1  0
// Digit 2            0xX3         X     0   0   1  1
// Digit 3            0xX4         X     0   1   0  0
// Decode-Mode        0xX9         X     1   0   0  1
// Intensity Control  0xXA         X     1   0   1  0
// Scan Limit         0xXB         X     1   0   1  1
// Shutdown           0xXC         X     1   1   0  0
// N/A                0xXD         X     1   1   0  1
// Feature            0xXE         X     1   1   1  0
// Display Test       0xXF         X     1   1   1  1

#define INITSIZE 10

typedef struct {
   UChar adr;
   UChar val;
} TFrame;

LOCAL const TFrame init[INITSIZE] = {
   { 0x0E, 0x0C },  // internal oscillator, enable B/HEX decoding, enable SPI (0x0C = 0b00001100) [see data sheet p. 14]
   { 0x0C, 0x81 },  // shutdown register. 0x81 = normal mode [see data sheet p. 11]
   { 0x0F, 0x00 },  // normal mode = 0, display test = 1 [see data sheet p. 12]
   { 0x01, 0x00 },  // set digit 0 to 0
   { 0x02, 0x00 },  // set digit 1 to 0
   { 0x03, 0x00 },  // set digit 2 to 0
   { 0x04, 0x00 },  // set digit 3 to 0
   { 0x09, 0xFF },  // decode mode: 0xFF = B/HEX decoding for all digits [see data sheet p. 12]
   { 0x0A, 0x03 },  // intensity control 7/32. 0x03 = 7/32 [see data sheet p. 13]
   { 0x0B, 0x03 }   // scan limit, display all numbers (digits 0-3) [see data sheet p. 13]
};

//#pragma FUNC_ALWAYS_INLINE(emit)
LOCAL Void emit(const UChar adr, const UChar val) {
   UChar ch = UCA1RXBUF;        // RXBUF auslesen, UCRXIFG := 0, UCOE := 0
   CLRBIT(P2OUT, BIT3);         // Select aktivieren
   UCA1TXBUF = adr;
   while (NOT TSTBIT(UCA1IFG, UCRXIFG)) ;
   ch = UCA1RXBUF;
   UCA1TXBUF = val;
   while (NOT TSTBIT(UCA1IFG, UCRXIFG)) ;
   ch = UCA1RXBUF;
   SETBIT(P2OUT, BIT3);         // Select deaktivieren
}

#pragma FUNC_ALWAYS_INLINE(UCA1_init)
GLOBAL Void UCA1_init(Void) {
   // set up Universal Serial Communication Interface A
   SETBIT(UCA1CTLW0, UCSWRST);  // UCA1 software reset
   UCA1BRW = 20;          // prescaler

   // in �bereinstimung mit dem SPI-Timing-Diagramm von AS1108
   UCA1CTLW0 = UCCKPH           // 15: clock phase select: rising edge
             | 0                // 14: clock polarity: inactive low
             | UCMSB            // 13: MSB first
             | 0                // 12: 8-bit data
             | UCMST            // 11: SPI master mode
             | UCMODE_0         // 10-9: mode select:3 -pin SPI
             | UCSYNC           // 8:  synchronous mode enable
             | UCSSEL__ACLK     // 7-6: clock source select
             | 0;               // 0: release the UCA0 for operation

   UInt i;
   for(i=0; i LT INITSIZE; i++) {
      emit(init[i].adr, init[i].val);
   }
   SETBIT(UCA1IE, UCRXIE); // interrupt enable
}

#define DATASIZE 2
LOCAL UInt  idx;              // Index
LOCAL UChar data[DATASIZE];   // Pointer auf das Datenfeldd

GLOBAL Void UCA1_emit(const UChar adr, const UChar val) {
   idx  = 0;
   data[0] = adr;
   data[1] = val;
   SETBIT(UCA1IFG, UCRXIFG); // indirekter Aufruf der ISR
}

#pragma vector = USCI_A1_VECTOR
__interrupt Void UCA1_ISR(Void) {
   UChar ch = UCA1RXBUF;          // RXBUF auslesen, UCRXIFG := 0, UCOE := 0
   CLRBIT(P2OUT, BIT3);           // Select aktivieren
   if (idx GE DATASIZE) {
      SETBIT(P2OUT,  BIT3);       // Select deaktivieren
      Event_set(EVENT_DONE_SEG);      // Event senden
      __low_power_mode_off_on_exit();
   } else {
      UCA1TXBUF = data[idx++];    // output next byte
   }
}
