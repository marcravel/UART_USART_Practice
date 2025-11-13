/*
 * File:   main.c
 * Author: ANJA RAVEL
 *
 * Created on October 10, 2025, 10:31 PM
 */



// PIC16F15376 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = ECH    // External Oscillator mode selection bits (EC above 8MHz; PFM set to high power)
#pragma config RSTOSC = HFINTPLL// Power-up default value for COSC bits (HFINTOSC with 2x PLL, with OSCFRQ = 16 MHz and CDIV = 1:1 (FOSC = 32 MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF        // WDT operating mode (WDT enabled regardless of sleep; SWDTEN ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register not write protected)
#pragma config WRTSAF = OFF     // Storage Area Flash Write Protection bit (SAF not write protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 32000000

#define TMR0_PRELOAD(desired_ms, prescaler) \
    ((uint8_t)(256 - ((desired_ms / 1000.0) / ((prescaler) / (_XTAL_FREQ / 4.0)))))

unsigned int tick_count = 0;
uint8_t preload = TMR0_PRELOAD(1, 256); // 1 ms with 1:256 prescale

__interrupt() void timer_interrupt(void) {
    if(PIR0bits.TMR0IF == 1) {
        tick_count++;
        TMR0L = preload;
        PIR0bits.TMR0IF = 0;
    }
}

void uart_init() {
    TX1STAbits.BRGH = 0;
    BAUD1CONbits.BRG16 = 1;
    
    SP1BRGH = 0x00;               // Baud rate at 9600 for 32MHz freq
    SP1BRGL = 0xCF;
     
    TX1STAbits.SYNC = 0;         // Asynchronous
    RC1STAbits.SPEN = 1;         // Serial Pin enable
    TX1STAbits.TXEN = 1;         // Enabling transmission
    
    TRISCbits.TRISC2 = 0;        // RC2 as output
    ANSELCbits.ANSC2 = 0;        // RC2 digital
    RC2PPS = 0x0F;               // Route TX1 to RC2
}

void main(void) {
    // Configure RE0 as output
    TRISEbits.TRISE0 = 0;
    LATEbits.LATE0 = 0;          // Initialize pin low

    // TIMER0 CLOCK AND PRESCALER SETTINGS
    T0CON1bits.T0CS = 0b010;     // Timer0 clock source = Fosc/4
    T0CON1bits.T0ASYNC = 0;      // Synchronous clock
    T0CON1bits.T0CKPS = 0b1000;  // Prescaler = 1:256

    TMR0L = preload;             // Load Timer0 initial value

    PIR0bits.TMR0IF = 0;         // Clear overflow flag
    PIE0bits.TMR0IE = 1;         // Enable Timer0 interrupt

    INTCONbits.GIE = 1;          // Global interrupt enable
    INTCONbits.PEIE = 1;         // Peripheral interrupt enable

    T0CON0bits.T0EN = 1;         // Enable Timer0 module
    
    uart_init();
    
    unsigned short delay_time = 500;
    
    while(1) {
        if(tick_count > delay_time) {
            LATEbits.LATE0 = !LATEbits.LATE0;
            while(!TX1STAbits.TRMT);  // wait until shift register empty
            TX1REG = 0x32; //Sending 1byte data
            tick_count = 0;
        }
    }
    
    return;
}
