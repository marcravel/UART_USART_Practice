PIC16F15376 → Arduino UART Communication (XC8 + Arduino)

This repository contains a simple test setup for sending data from a PIC16F15376 microcontroller to an Arduino using UART.
The PIC runs XC8 (MPLAB X) and toggles an LED + transmits a byte every second using Timer0 and UART1.
The Arduino receives the byte and prints it over USB serial.

📌 Features
PIC16F15376 (XC8)

32 MHz HFINTPLL clock configuration

Timer0 1 ms tick using preload calculation

Periodic 1-second LED toggle (RE0)

UART1 TX transmission (9600 baud)

PPS configuration for TX pin

Sends raw byte (23) every 1 second

Arduino (UNO / Nano)

Receives UART at 9600 baud

Prints received byte to Serial Monitor

Toggles built-in LED on each received byte

📡 Hardware Connections

PIC TX → Arduino RX (Pin 0)
COMMON GND is mandatory.

Note: Arduino USB serial must not be connected to RX0 during programming; disconnect if needed.

🧰 PIC: UART Requirements

The PIC16F15376 requires:

Correct baud rate (SPBRG values calculated for 32 MHz)

BRGH and BRG16 configuration

PPS mapping for TX1 to a valid pin

TRIS + ANSEL configuration on that pin

Example PPS (check datasheet for your selected pin):

TRISCbits.TRISC0 = 0;
ANSELCbits.ANSELC0 = 0;
RC0PPS = 0x0F;   // Example PPS code for TX1

🕒 PIC Timer0

Configured with Fosc/4

Prescaler 1:256

Preload macro calculates exact 1 ms overflow

In ISR: reload, clear flag, increment tick counter

📄 File Structure
/ ── README.md          (this file)
/pic/                   (XC8 project for PIC16F15376)
/arduino/               (Arduino .ino sketch)
/docs/                  (optional notes, wiring, oscilloscope captures)

▶️ How It Works
PIC:

Every 1 second:

Toggle RE0

Send byte 0x17 (decimal 23) via UART

Reset the timer counter

Arduino:

Wait for UART data

If a byte arrives: print it and toggle LED

💻 Build Instructions
PIC (XC8 / MPLAB X)

Open the project under /pic/

Build normally

Flash to PIC16F15376 (PICkit, Curiosity Nano, etc.)

Arduino

Open /arduino/uart_receive_test/uart_receive_test.ino

Upload to Arduino board

Open Serial Monitor @ 9600 baud

🛠️ Requirements

PIC16F15376 (or compatible board)

Arduino Uno / Nano / Mega

Common ground between both

MPLAB X + XC8 (or your preferred toolchain)

Arduino IDE or PlatformIO

Git 2.51+

⚠️ Notes

The byte value 23 is non-printable ASCII.
The Arduino prints (int)c so you see the numeric value.

If UART shows garbage: check PPS + baud rate calculation.

Always verify your TX pin routing against the PIC datasheet.

📈 Future Improvements

Implement RX on PIC (bidirectional UART)

Switch to ring buffer for UART RX

Add oscilloscope / logic analyzer captures

Optional CRC or packet structure
