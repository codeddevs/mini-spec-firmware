/* Copyright (c) 2025 Coded Devices Oy
 * 
 * File: mini_spec.c
 * 
 * Ver and date:
 *        1.0.5.3 / 2025-3-5
 *        - median filter option for sampling
 *        - redefining of data types
 *        - removing of unused variables
 *        - improved intToArray function
 *        - write back the new LED intensity value
 *        - fix for the sample counting offset error (samples needed 290 --> 288)
 *        - when reading a full spectrum, now returns ch_number; earlier returned channel index
 * 
 *        1.0.4.1 /2024-10-9
 *        - Optimizing values of ADCLK and ADACQL 
 * 
 *        1.0.4.0 / 2024-8-31
 *        - ReadADC function modified
 *        - ADC clock setting moved from ReadADC to initialization part of main
 *        - Added constant adc sampling delay using reg ADACQ
 * 
 *        1.0.3.1 / 2024-3-28
 *        - added RA2 as input and command 'W' to read its state
 *        - time delays in 'S', 'G' and 'A' commands set to 50 ms.
 *        - changes in ms_delay function
 * 
 *        1.0.3.0 / 2024-3-27
 *        - time delay in 'G' command handling changed from 200 to 50 ms
 * 
 *        1.0.2.1 / 2024-2-9
 *        - added the 200 ms delay in 'G' command handling
 * 
 *        1.0.2.0 / 2024-1-27
 *        - new command "G" for sending a channel reading from previously measured spectrum
 * 
 *        1.0.1.1 / 2024-1-1
 *        - new delay function expMsDelay
 *        - integration time set using numBuffer instead of rotation method
 *        - XC8 compiler version 2.45
 *        - DFP version 1.23.382 
 *        0.3.4 / 13.5.2022
 *        - extra getSpectrum - call in init
 *        - systematic use of ms_delay -function
 *        - adjustable LED intensity 
 *       
 *        0.3.1 / 8.4.2022 (edit this version to fVersion variable!)
 *        This version is for printed circuit board without level shifting transistors.
 *        These transistors also invert their level (note in spec_ST variable). Last version for level shifting 
 *        model is 17.12.2021.
 * 
 *        Based on "test_184_mcu ver 2022-3-11.c" after project restart and renaming on 2022-3-18.
 * 
 * Desc : Mini Spec project for 16F18426.
 *        
 *
 *        
 */

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = ON    // Clock Out Enable bit (CLKOUT function is enabled; FOSC/4 clock appears at OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTS = PWRT_64  // Power-up Timer Enable bit (PWRT set at 64 ms)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) is set to 2.7V)
//#pragma config ZCDDIS = OFF     // ** OLD CONFIG NAME FOR ZCD** Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)

#pragma config PPS1WAY = OFF    // Peripheral Pin Select one-way control (The PPSLOCK bit can be set and cleared repeatedly by software)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will not cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF        // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SOSC    // WDT input clock selector (WDT reference clock is the 32kHz secondary oscillator)

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register not write protected)
#pragma config WRTD = OFF       // Data EEPROM write protection bit (Data EEPROM NOT write protected)
#pragma config WRTSAF = OFF     // Storage Area Flash Write Protection bit (SAF not write protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)

#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <pic16f18426.h>

#define LED LATCbits.LATC3

// ** C12880MA pins **
// ST @ RC0 / pin 10
// TRG @ RA5 (INT0 input)
// VIDEO @ RC1 (ADC input)
// CLK @ RC2 (reference clock output)
// EOS ??

#define spec_ST LATCbits.LATC0  // start pulse ST, PIC16 pin 10 -> C12880MA pin 8
#define CH_COUNT 288            // Hamamatsu C12880MA has 288 channels

// GLOBALS
static char fVersion[4] = {1, 0, 5, 3}; // FIRMWARE VERSION, [ <major_1>, <major_2>, <minor>, <patch>]
static uint16_t RTC;
uint16_t ADCValues[CH_COUNT];
static int TRG_counter; // must be signed integer
uint16_t integ_time;

// PROTOTYPES
void ms_delay(uint16_t msec);
void blinkLED(uint8_t count);
char getU1();
void putU1(uint8_t c);
void intToArray(uint16_t value, uint8_t output[4]);
uint16_t GetSpectrum(uint8_t c);
uint16_t MedianOfThree(uint16_t a, uint16_t b, uint16_t c);
void SetCLKRDivision(uint8_t divisor);
uint16_t ReadADC();
void SourceIntensity(uint8_t DAC_value);

// INTERRUPT SERVICE ROUTINE
// edit : 17.12.2021

void __interrupt() myISR() {

    // INT0
    if (PIR0bits.INTF == 1) {
        PIR0bits.INTF = 0;
        TRG_counter++;
        return;
    }
    // Timer0
    if (PIR0bits.TMR0IF == 1) {
        PIR0bits.TMR0IF = 0;
        RTC++;
        return;
    }
}

// DELAY
// edit : 2023-12-28
// desc : Timer0 overflow every 4/Fosc * prescaler * postscaler =
//        4 / 4 MHz * 1:4 * 1:1 = 1 ms

void ms_delay(uint16_t msec) {
    msec = msec + RTC;
    while (msec != RTC)
        ;
}

// BLINK
// edit : 10.12.2021

void blinkLED(uint8_t blinks) {

    while (blinks > 0) {
        LED = 1;
        ms_delay(500); // 0.5 sec
        LED = 0;
        ms_delay(500);
        blinks--;
    }
}

// READ CHAR 
// edit : 15.11.2021
// todo : simplify this when reading becomes more reliable

char getU1() {
    char c = 0;

    while (!PIR3bits.RC1IF) // wait for data
        ;

    if (RC1STAbits.FERR) // framing error
        blinkLED(2);

    c = RC1REG;

    if (RC1STAbits.OERR) { // overrun error
        RC1STAbits.CREN = 0;
        blinkLED(3);
        RC1STAbits.CREN = 1;
    }

    return c;
}

// WRITE CHAR
// edit : 13.11.2021

void putU1(uint8_t c) {
    while (TX1STAbits.TRMT == 0) // wait until Transmit Shift Register is empty
        ;
    TX1REG = c;
}


// READ ADC VALUE
// edit : 2025-1-29
// desc : Set GO bit to start the conversion and wait GO bit to reset when conversion is done.
// TODO : Check if this function can be made to return uint16_t type instead of int without loss of speed.

uint16_t ReadADC() {
    int result;

    ADCON0bits.GO = 1; // start conversion
    while (ADCON0bits.GO == 1)
        ;
    result = ADRESH << 8;
    return (uint16_t) (result + ADRESL);
}

// READ A FULL SPECTRUM
// edit : 2025-3-5
// desc : Sequence: 1) set ST-bit HIGH and wait for at least 6 / (sensor CLK freq), this wait is the adjustable integration time,
//					2) set ST-bit LOW,
//					3) count 88 rising TRG edges, interrupt routine catches the edges and updates the TRG_counter, 
//					4) take first reading right after 89th rising TRG edge,
//					5) take 287 readings in the same way right after rising TRG edge.
//        
//        Option for three samples per channel and then median filtering.
//        Three samples require slower clock pulsing --> longer integration time --> higher intensity values.
//
//        2025-2-8 : Scope test verifies that correct number of integration & init steps is 88 &
//        while(old_TRG_count < CH_COUNT - 1)
//        gives correct number of measured channels.

uint16_t GetSpectrum(uint8_t c) {
    int16_t old_TRG_count; // must be signed integer
    uint16_t k;
    uint16_t temp[3];

    // Select sensor CLK freq, multisampling requires slower CLK.
    if (c != 0)
        SetCLKRDivision(32); // sensor CLK freq = 1 kHz --> sampling interval = 1 ms
    else
        SetCLKRDivision(16); // sensor CLK freq = 2 kHz --> sampling interval is 0.5 ms

    // Init buffer with recognizable values
    for (k = 0; k < CH_COUNT; k++) {
        ADCValues[k] = 0;
    }

    // Init measurement and wait to integrate sensor. 
    // Total integration time = integ_time + 47 sensor CLK pulses. 
    spec_ST = 1;
    ms_delay(integ_time);
    spec_ST = 0;

    // Wait for 88 integration & init steps
    TRG_counter = 0;
    while (TRG_counter != 88)
        ;

    // TRG_counter increases every time a new channel is ready for reading
    TRG_counter = -1;
    old_TRG_count = -1;

    
    // Multiple samples for median filtering
    // edit: 2025-1-29
    k = 0;
    if (c != 0) {
        while (old_TRG_count < CH_COUNT - 1) {
            if (old_TRG_count != TRG_counter) {
                old_TRG_count = TRG_counter;
                //LED = 1;
                temp[0] = ReadADC();
                temp[1] = ReadADC();
                temp[2] = ReadADC();
                //LED = 0;
                ADCValues[old_TRG_count] = MedianOfThree(temp[0], temp[1], temp[2]);
                k++; // actual number of stored values
            }
        }
    }        
    // One sample
    else {
        while (old_TRG_count < CH_COUNT - 1) {
            if (old_TRG_count != TRG_counter) {
                old_TRG_count = TRG_counter;
                //LED = 1;
                ADCValues[old_TRG_count] = ReadADC();
                //LED = 0;
                k++; // actual number of stored values
            }
        }
    }

    // Return the actual number of measured channels --> smaller than expected if channels were skipped.
    return k;
}

// edit : 2025-1-29
// desc : Quick way to get the median of just three values.

uint16_t MedianOfThree(uint16_t a, uint16_t b, uint16_t c) {
    if ((a > b) != (a > c))
        return a;
    if ((b > a) != (b > c))
        return b;
    return c;
}

// edit 2025-1-27
// desc : Adjust refresh rate of the signal.

void SetCLKRDivision(uint8_t divisor) {
    CLKRCONbits.CLKREN = 0; // CLKR disable
    if (divisor == 16)
        CLKRCONbits.CLKRDIV = 0b100; // 100 is division by 16 (--> 2 kHz with 32 kHz internal oscillator)
    else {
        if (divisor == 32)
            CLKRCONbits.CLKRDIV = 0b101; // 101 is division by 32 (--> 1 kHz with 32 kHz internal oscillator)
    }
    CLKRCONbits.CLKREN = 1; // CLKR enable
}

// edit : 2025-2-04
// desc : Extract numbers of four digit long value in to buf[].

void intToArray(uint16_t value, uint8_t buf[4]) {

    buf[0] = value / 1000;
    value = value - buf[0] * 1000;

    buf[1] = (uint8_t) (value / 100);
    value = value - buf[1] * 100;

    buf[2] = (uint8_t) (value / 10);
    value = value - buf[2] * 10;

    buf[3] = (uint8_t) value;
}

// func : SourceIntensity
// edit : 2025-1-30
// desc : 5-bit DAC output is used to control intensity of the source (LED).
//        Sets DAC output, DAC_value in range 0...31. 

void SourceIntensity(uint8_t DAC_value) {
    if (DAC_value > 31)
        DAC_value = 31;
    DAC1CON1bits.DAC1R = DAC_value;
}

// MAIN
// edit : 2025-2-8

void main(void) {
    // LOCAL VARIABLES
    char command = 0;
    uint16_t i; // use to index channels
    uint8_t j; // use to index outputNumbers[] 
    char numBuffer[4] = {'\n', '\n', '\n', '\n'};
    uint8_t b_index = 0;
    uint8_t source_int = 10; // LED intensity value 0...31
    uint8_t isMultiSampling = 0; // like boolean for single vs multi samples per channel
    uint8_t outputNumbers[4] = {0, 0, 0, 0}; // four digit char representation of an integer
    

    // 4MHz INTERNAL OSCILLATOR
    // edit 17.12.2021
    OSCCON1bits.NDIV = 0x0; // no clock division
    OSCFRQbits.HFFRQ0 = 0; // 010 is 4 MHz, 011 is 8 MHz
    OSCFRQbits.HFFRQ1 = 1;
    OSCFRQbits.HFFRQ2 = 0;

    // INT in RA5
    INTCONbits.INTEDG = 1; // rising edge
    INTPPS = 0x5; // RA5 code is 00101
    ANSELAbits.ANSA5 = 0; // RA5 is digital I/O
    TRISAbits.TRISA5 = 1; // RA5 is input
    PIR0bits.INTF = 0; // reset flag
    PIE0bits.INTE = 1; // enable int interrupt

    // CLKRCON (reference clock control) at RC2 --> CLK pin at CA12880MA
    CLKRCLKbits.CLKRCLK3 = 0; // 0100 is MFINTOSC (32 kHz)
    CLKRCLKbits.CLKRCLK2 = 1;
    CLKRCLKbits.CLKRCLK1 = 0;
    CLKRCLKbits.CLKRCLK0 = 0;

    CLKRCONbits.CLKRDIV0 = 0; // 100 is division by 16 (--> 2 kHz with 32 kHz internal oscillator)
    CLKRCONbits.CLKRDIV1 = 0; // 
    CLKRCONbits.CLKRDIV2 = 1;
    RC2PPS = 0x19; //  is CLKR output code 01 1001
    TRISCbits.TRISC2 = 0;
    CLKRCONbits.CLKREN = 1; // CLKR enable

    // TIMER0
    // edit : 10.12.2021
    T0CON1bits.T0CS = 0x02; // b010, Timer0 clock source is Fosc/4
    T0CON1bits.T0CKPS = 0x02; // b0010, Timer0 prescaler is 1:4
    T0CON0bits.OUTPS = 0x0; // b0000, Timer0 postscaler is 1:1
    PIR0bits.TMR0IF = 0; // reset Timer0 interrupt flag
    PIE0bits.TMR0IE = 1; // enable TimerO interrupt
    T0CON0bits.T0EN = 1; // enable Timer0 module

    // UART1
    // TX1 at RC4 & RX1 at RC5
    // edit : 16.11.2021
    TX1STAbits.BRGH = 1; // High Baud Rate Select bit
    SPBRG = 0x19; // 9600 with 4MHz oscillator


    RC4PPS = 0x0F; // TX1 at pin RC4
    ANSELCbits.ANSC4 = 0; // RC4 is digital
    TRISCbits.TRISC4 = 0; // RC4 is output

    TX1STAbits.SYNC = 0; // async operation
    TX1STAbits.TXEN = 1; // enable EUSART transmitter

    // NEW REGISTER NAMES FOR RX1, use with DFP ver 1.23 and later  
    RX1PPSbits.RX1PPS4 = 1; // RX1 at pin RC5
    RX1PPSbits.RX1PPS3 = 0;
    RX1PPSbits.RX1PPS2 = 1;
    RX1PPSbits.RX1PPS1 = 0;
    RX1PPSbits.RX1PPS0 = 1;

    ANSELCbits.ANSC5 = 0; // RC5 is digital (default analog i.e. 1)
    TRISCbits.TRISC5 = 1; // RC5 is input

    RC1STAbits.CREN = 1; // continuous receive enable
    RC1STAbits.SPEN = 1; // serial port enable (receiver)

    // LED
    TRISCbits.TRISC3 = 0;

    // ADC at RC1
    // edit : 2024-8-31
    TRISCbits.TRISC1 = 1; // disable output driver
    ANSELCbits.ANSC1 = 1; // analog input
    ADPCH = 0x11; // b010001, RC1 Positive Input Channel (PCH) selection bits

    ADCON0bits.CS = 0; // ADC clock supplied by Fosc
    //ADCLK = 0x01;           // b000001,  --> Fosc/(2*(ADCLK[7:0]+1)) = Fosc/4 --> 1 us Tad
    //ADCLK = 0x03;           // b000011,  --> Fosc/(2*(ADCLK[7:0]+1)) = Fosc/8 --> 2 us Tad
    //ADCLK = 0x04;           // b000100,  --> Fosc/(2*(ADCLK[7:0]+1)) = Fosc/10 --> 4 us Tad
    //ADCLK = 0x0F;           // 0x0F --> 8 us Tad
    ADCLK = 0x0F; // 0x1F --> 16 us Tad

    ADACQH = 0x0; // upper adc sampling delay reg
    //ADACQL = 0x80;          // lower adc sampling delay reg, 128 x 0.28 usec delay
    ADACQL = 0x0;

    ADREFbits.NREF = 0; // 0, negative ref voltage is Vss
    ADREFbits.PREF1 = 1; // b11, positive reference is internal Fixed Voltage Reference (FVR) module
    ADREFbits.PREF0 = 1;
    FVRCONbits.ADFVR1 = 1; // b11, ADC FVR buffer gain is 4x --> 4.096 V
    FVRCONbits.ADFVR0 = 1;
    FVRCONbits.FVREN = 1; // Fixed Voltage Reference (FVR) is enabled

    ADCON0bits.FM = 1; // data is right justified
    ADCON0bits.CONT = 0; // continuous operation disabled
    ADCON2bits.MD = 0; // Legacy mode, no DSP
    ADCON0bits.ON = 1; // ADC is enabled

    // DAC for LED intensity control
    // uses earlier set FVR value
    // edit : 2024-7-30
    
    FVRCONbits.CDAFVR1 = 1; // Comparator & DAC FVR buffer gain is 4x (4.096)
    FVRCONbits.CDAFVR0 = 1;

    DAC1CON0bits.PSS1 = 1;  // PSS = [1,0] DAC positive source is FVR
    DAC1CON0bits.PSS0 = 0;  
    DAC1CON0bits.NSS = 0;   // 0 --> DAC negative source is Vss (gnd)
    DAC1CON0bits.OE1 = 1;   // 1 --> DAC output on DAC1OUT1 pin (RA0 / #13)

    DAC1CON1bits.DAC1R = 0x10; // 0x10 = 16 --> 50% of max DAC
    DAC1CON0bits.EN = 1;    // 1 --> DAC enabled

    // SPECTROMETER PINS
    TRISCbits.TRISC0 = 0;   // ST @ RC0 is output

    // OTHER INITS
    // edit : 2025-2-16
    ANSELAbits.ANSA2 = 0;   // RA2 is analog input by default, make it digital
    TRISAbits.TRISA2 = 1;   // RA2 is input
    WPUAbits.WPUA2 = 1;     // activate weak-pull-up on RA2 input
    INTCONbits.GIE = 1;     // enable all active interrupts
    LED = 0;
    integ_time = 10;        // default integration time is 10 ms
    GetSpectrum(0);         // read one spectrum to init the sensor
    blinkLED(2);

    // MAIN LOOP
    // edit : 2025-2-7
    /*  command set: 
        'S' read spectrum and send a single pre selected channel
        'G' send a single pre selected channel from already measured spectrum
        'A' read spectrum and send all channels
        'L' flash LED pre set number of times
        'I' set one step higher integration time (after max value returns to min value).
        'R' reset instrument
        'V' read instrument firmware version
        'H' change LED intensity to pre set value
        extra test commands:
        'E' read directly ADC (no spectrometer reading)  
        'F' check fixed voltage reference (FVR) status
        'W' read input state '0' or '1'
        'M' Toggle multi sampling, default one sample
     */
    while (1) {

        ms_delay(5);

        if (PIR3bits.RC1IF) { // UART1 receive buffer data available
            command = getU1();

            switch (command) {

                    // First measure a new spectrum and then 
                    // Send the value of a single pre selected channel.
                    // Notice, the request is for channel number 1...288, that is index + 1.
                    // edit : 2025-2-7
                case 'S':

                    putU1(command);
                    putU1('\n');
                    putU1('\r');

                    // Store channel number 1...288 in to i, then reset numBuffer
                    i = (uint16_t) atoi(numBuffer);
                    numBuffer[0] = '\n';
                    numBuffer[1] = '\n';
                    numBuffer[2] = '\n';
                    numBuffer[3] = '\n';
                    b_index = 0;

                    ms_delay(50);
                    //getSpectrum();          // updates ADCValues[]
                    GetSpectrum(isMultiSampling);

                    // print channel number (1...288)
                    intToArray(i, outputNumbers);
                    for (j = 0; j < 4; j++)
                        putU1(outputNumbers[j] + '0');
                    putU1(' ');

                    // print channel value 
                    intToArray(ADCValues[i - 1], outputNumbers);
                    for (j = 0; j < 4; j++)
                        putU1(outputNumbers[j] + '0');
                    putU1('\n');
                    putU1('\r');

                    break;

                    // Send one pre-selected value from already measured spectrum.
                    // edit 2025-2-7
                case 'G':
                    putU1(command);
                    putU1('\n');
                    putU1('\r');

                    // channel number 1...288
                    i = (uint16_t) atoi(numBuffer);
                    numBuffer[0] = '\n';
                    numBuffer[1] = '\n';
                    numBuffer[2] = '\n';
                    numBuffer[3] = '\n';
                    b_index = 0;

                    ms_delay(50);

                    // print channel number (1...288)
                    intToArray(i, outputNumbers);
                    for (j = 0; j < 4; j++)
                        putU1(outputNumbers[j] + '0');
                    putU1(' ');

                    // print channel value
                    intToArray(ADCValues[i - 1], outputNumbers);
                    for (j = 0; j < 4; j++)
                        putU1(outputNumbers[j] + '0');
                    putU1('\n');
                    putU1('\r');

                    break;

                    // read a full spectrum
                    // edit : 2025-2-8
                    // todo : check if delay is necessary
                case 'A':
                    putU1(command);
                    putU1('\n');
                    putU1('\r');

                    ms_delay(50); // let UART operations finish

                    intToArray(GetSpectrum(isMultiSampling), outputNumbers);

                    for (j = 0; j < 4; j++)
                        putU1(outputNumbers[j] + '0');
                    putU1('\n');
                    putU1('\r');

                    for (i = 0; i < CH_COUNT; i++) {
                        // print channel number (1...288)
                        intToArray(i + 1, outputNumbers);
                        for (j = 0; j < 4; j++)
                            putU1(outputNumbers[j] + '0');
                        putU1(' ');
                        // print channel value
                        intToArray(ADCValues[i], outputNumbers);
                        for (j = 0; j < 4; j++)
                            putU1(outputNumbers[j] + '0');
                        putU1('\n');
                        putU1('\r');
                    }

                    break;

                    // toggle LED X times
                    // 27.2.2021
                case 'L':
                    putU1(command);
                    putU1('\n');
                    putU1('\r');
                    // Note! atoi uses numbers before '\n' or all numbers in array
                    if (b_index != 0) {
                        blinkLED((uint8_t) atoi(numBuffer));
                        numBuffer[0] = '\n';
                        numBuffer[1] = '\n';
                        numBuffer[2] = '\n';
                        numBuffer[3] = '\n';
                        b_index = 0;
                    }
                    break;

                    // set integration time
                    // edit : 2023-12-29
                case 'I':
                    putU1(command);
                    putU1('\n');
                    putU1('\r');

                    // Read input buffer for sensor integration time.
                    if (b_index > 0)
                        integ_time = (uint16_t) atoi(numBuffer);

                    // clear & reset input buffer
                    numBuffer[0] = '\n';
                    numBuffer[1] = '\n';
                    numBuffer[2] = '\n';
                    numBuffer[3] = '\n';
                    b_index = 0;

                    intToArray(integ_time, outputNumbers);

                    for (i = 0; i < 4; i++)
                        putU1(outputNumbers[i] + '0');
                    putU1(' ');
                    break;

                    // reset and init
                    // edit : 2025-1-30
                case 'R':
                    // zero led
                    LED = 0;

                    // init spectrometer integration time
                    integ_time = 10;

                    // reset buffer for inputs
                    numBuffer[0] = '\n';
                    numBuffer[1] = '\n';
                    numBuffer[2] = '\n';
                    numBuffer[3] = '\n';
                    b_index = 0;

                    // blink 
                    blinkLED(2);
                    break;

                    // send firmware version number
                    // edit 8.4.2022
                case 'V':
                    putU1(command);
                    putU1('\n');
                    putU1('\r');
                    putU1(fVersion[0] + '0');
                    putU1(fVersion[1] + '0');
                    putU1(fVersion[2] + '0');
                    putU1(fVersion[3] + '0');
                    break;

                    // set LED intensity to preset (5-bit) value
                    // edit: 2025-2-7
                    // todo: return actual LED value as done with integration time
                case 'H':
                    putU1(command);
                    putU1('\n');
                    putU1('\r');

                    // led intensity value 0...31
                    source_int = (uint8_t) atoi(numBuffer);
                    SourceIntensity(source_int);
                    numBuffer[0] = '\n';
                    numBuffer[1] = '\n';
                    numBuffer[2] = '\n';
                    numBuffer[3] = '\n';
                    b_index = 0;

                    // send back the received value
                    intToArray(source_int, outputNumbers);
                    for (i = 0; i < 4; i++)
                        putU1(outputNumbers[i] + '0');
                    putU1(' ');
                    break;

                    // read a value
                    // edit : 10.12.2021
                case 'E':
                    putU1(command);
                    putU1('\n');
                    putU1('\r');
                    intToArray(ReadADC(), outputNumbers);
                    putU1(outputNumbers[0] + '0');
                    putU1(outputNumbers[1] + '0');
                    putU1(outputNumbers[2] + '0');
                    putU1(outputNumbers[3] + '0');
                    break;

                    // check Fixed Voltage Reference status    
                case 'F':
                    putU1(command);

                    if (FVRCONbits.FVRRDY == 1)
                        putU1('Y');
                    else
                        putU1('N');

                    putU1('\n');
                    putU1('\r');
                    break;

                    // Send back the state of input RA2 as 'W0' or 'W1
                    // edit: 2024-3-28
                case 'W':
                    putU1(command);
                    if (RA2 == 1)
                        putU1('1');
                    else
                        putU1('0');
                    putU1('\n');
                    putU1('\r');
                    break;

                    // Toggle multi sampling setting and return its new state value.    
                    // edit: 2025-1-29    
                case 'M':
                    putU1(command);
                    if (isMultiSampling == 0) {
                        isMultiSampling = 1;
                        putU1('1');
                    } else {
                        isMultiSampling = 0;
                        putU1('0');
                    }
                    putU1('\n');
                    putU1('\r');
                    break;


                    // default, capture possible numbers for later use    
                default:
                    if (command >= '0' && command <= '9' && b_index < 4) {
                        numBuffer[b_index] = command;
                        b_index++;
                    }
                    break;
            }
        }
    }
    return;
}

