/* Copyright (c) 2024 Coded Devices Oy
 * 
 * File: mini_spec.c
 * 
 * Ver and date:
 * 
 *        1.0.3.1 / 2024-3-28
 *        - added RA2 as input and command 'W' to read its state
 *        - time delays in 'S', 'G' and 'A' commands set t 50 ms.
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
 * TODO : Replace my own intToArray with standard itoa.
 *        Optimize the ADC accuracy by modifying experimental function expReadADC.
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
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled

#include <xc.h>
#include <stdint.h>
#include <pic16f18426.h>

#define LED LATCbits.LATC3

// ** C12880MA pins **
// ST @ RC0 / pin 10
// TRG @ RA5 (INT0 input)
// VIDEO @ RC1 (ADC input)
// CLK @ RC2 (reference clock output)
// EOS ??
#define spec_ST         LATCbits.LATC0      // start pulse ST, PIC16 pin 10 -> C12880MA pin 8

// ** Channel count ** 
# define CH_COUNT    290            

// GLOBALS
static char fVersion[4] = {1, 0, 3, 1}; // FIRMWARE VERSION, [ <major_1>, <major_2>, <minor>, <patch>]
static uint16_t RTC;
int outputNumbers[4] = {0, 0, 0, 0};    // four digit representation of an adc value
int ADCValues[CH_COUNT];
static int TRG_counter;                 // must be signed integer
unsigned int integ_time;               

static int led_state;

// PROTOTYPES
void ms_delay(uint16_t msec);
void blinkLED(uint8_t count);
char getU1();
void putU1(char c);
void intToArray(int value);
int readADC();
int getSpectrum();
int ReadADC();
void SourceIntensity(int DAC_value); 

// INTERRUPT SERVICE ROUTINE
// edit : 17.12.2021
void __interrupt() myISR(){
    
    // INT0
    if(PIR0bits.INTF == 1){
        PIR0bits.INTF = 0;
        TRG_counter++;
        return;    
    }
    // Timer0
    if(PIR0bits.TMR0IF == 1){
        PIR0bits.TMR0IF = 0;
        RTC++;
        return;
    }
}

// DELAY
// edit : 2023-12-28
// desc : Timer0 overflow every 4/Fosc * prescaler * postscaler =
//        4 / 4 MHz * 1:4 * 1:1 = 1 ms
void ms_delay(uint16_t msec){
    msec = msec + RTC;
    while(msec != RTC)
        ;
}

// BLINK
// edit : 10.12.2021
void blinkLED(uint8_t blinks){
    
    while(blinks > 0){
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
char getU1(){
    char c = 0;
    
    while(!PIR3bits.RC1IF)      // wait for data
        ;
    
    if(RC1STAbits.FERR)        // framing error
        blinkLED(2);
                
    c = RC1REG;
    
    if(RC1STAbits.OERR){        // overrun error
        RC1STAbits.CREN = 0;
        blinkLED(3);
        RC1STAbits.CREN = 1;
    }
   
	return c;
}

// WRITE CHAR
// edit : 13.11.2021
void putU1(char c){
    while(TX1STAbits.TRMT == 0)  // wait until Transmit Shift Register is empty
        ;
	TX1REG = c;
}


// READ ADC VALUE
// edit : 2024-3-28
int ReadADC(){
    int result = 0;
    //uint8_t oldADCLK = ADCLK;
    ADCLK = 0x0F;           // 0x06 --> 3,5 us
                            // 0x0F --> 7,5 us
    
    ADCPCON0bits.CPON = 1;  // activate charge pump
    while(ADCPCON0 ==  0)   // wait until charge pump is ready
        ;
    ADCON0bits.GO = 1;      // start conversion
    while(ADCON0bits.GO == 1)
        ;
    ADCPCON0bits.CPON = 0;  // deactivate charge pump
    //ADCLK = oldADCLK;       // return AD clock value
    
    result = ADRESH << 8;
    return result + ADRESL;
}

/* func : getSpectrum
 * rev. : 18.3.2022
 *        Changes to previous version 17.12.2021 to verify that last channel is read correctly every time.
 * desc : Read one full spectrum, set control bits and get adc. Return number of channels.
 *        Sequence: 1) set ST-bit HIGH and wait for 10/Osc_frequency --> 10/125 kHz = 80 us
 *					2) set ST-bit LOW
 *					3) count 88 (or 86?) rising TRG edges
 *					4) take first reading right after 89th rising TRG edge
 *					5) take 287 readings in the same way right after rising TRG edge
 *					6) simultaneously with the 289th rising edge rises also EOS (end of scan) signal
 * 
 */
 int getSpectrum(){

	int old_TRG_count;  // must be signed integer
    int k;
    
    /* init with recognizable value */
    // 11.3.2022
    for(k=0; k<CH_COUNT; k++){
        ADCValues[k] = 0;
    }
	
    // integration period
    // edit 11.3.2022
	    
    spec_ST = 1;        // no inversion on production board ver 0.1 11.3.2022
    
    ms_delay(integ_time);
		    
	spec_ST = 0;        // no inversion 11.3.2022
    
    TRG_counter = 0;	// increased only inside INT0 interrupt handler
	while(TRG_counter != 86)    // earlier 88
		;
		 
	TRG_counter = -1;
    old_TRG_count = -1;
    while(old_TRG_count < CH_COUNT){     // 11.3.2022
		if(old_TRG_count != TRG_counter){
            old_TRG_count = TRG_counter;
            ADCValues[old_TRG_count] = ReadADC();
    	}
	}
   
    return old_TRG_count;               // 11.3.2022

 }

/* func : intToArray
 * rev  : 12.3.2018
 * desc : Populate 4-len outputNumbers global array with numbers of an ADC-reading.
 *        Use outputNumbers[] only for this purpose.
 * NOTE : Am I basicly rewriting std-function itoa()?
 */
void intToArray(int value){
	
	if(value > 999){
		outputNumbers[0] = value / 1000;
		value = value - outputNumbers[0] * 1000;
	}else
		outputNumbers[0] = 0;

	if(value > 99){
		outputNumbers[1] = value / 100;
		value = value - outputNumbers[1] * 100;
	}else
		outputNumbers[1] = 0;

	if(value > 9){
		outputNumbers[2] = value / 10;
		value = value - outputNumbers[2] * 10;
	}else
		outputNumbers[2] = 0;

	outputNumbers[3] = value;	
}

// func : SourceIntensity
// edit : 22.4.2022
// desc : 5-bit DAC outout is used to control intensity of the source (LED).
void SourceIntensity(int DAC_value)
{
    
    if (DAC_value > 35)
        DAC_value = 35;
    else
        if(DAC_value < 0)
            DAC_value = 0;
    
    DAC1CON1bits.DAC1R = DAC_value;
}

// MAIN
// edit : 6.5.2022
void main(void)
{
    // LOCAL VARIABLES
    //int delay = 0;
    char command = 0;
    //int ADCValue = 0;
    int i, j;
    int readingCount = 0;
    char numBuffer[4] = {'\n', '\n', '\n', '\n'};
    int b_index = 0;
    int source_int = 10; // LED intensity value 0...31
    
    // 4MHz INTERNAL OSCILLATOR
    // edit 17.12.2021
    OSCCON1bits.NDIV = 0x0; // no clock division
    OSCFRQbits.HFFRQ0 = 0;  // 010 is 4 MHz, 011 is 8 MHz
    OSCFRQbits.HFFRQ1 = 1;
    OSCFRQbits.HFFRQ2 = 0;
    
    // INT in RA5
    INTCONbits.INTEDG = 1;  // rising edge
    INTPPS = 0x5;           // RA5 code is 00101
    ANSELAbits.ANSA5 = 0;   // RA5 is digital I/O
    TRISAbits.TRISA5 = 1;   // RA5 is input
    PIR0bits.INTF = 0;      // reset flag
    PIE0bits.INTE = 1;      // enable int interrupt
    
    // CLKRCON (reference clock control) at C2
    CLKRCLKbits.CLKRCLK3 = 0;   // 0100 is MFINTOSC (32 kHz)
    CLKRCLKbits.CLKRCLK2 = 1;   // 0100 if Fosc
    CLKRCLKbits.CLKRCLK1 = 0;
    CLKRCLKbits.CLKRCLK0 = 0;
    
    CLKRCONbits.CLKRDIV0 = 0;   // 100 is division by 16 (--> 2 kHz with 32 kHz internal oscillator)
    CLKRCONbits.CLKRDIV1 = 0;   // 
    CLKRCONbits.CLKRDIV2 = 1;
    RC2PPS = 0x19;              //  is CLKR output code 01 1001
    TRISCbits.TRISC2  = 0;
    CLKRCONbits.CLKREN = 1;     // CLKR enable
    
    // TIMER0
    // edit : 10.12.2021
    T0CON1bits.T0CS = 0x02;     // b010, Timer0 clock source is Fosc/4
    T0CON1bits.T0CKPS = 0x02;   // b0010, Timer0 prescaler is 1:4
    T0CON0bits.OUTPS = 0x0;     // b0000, Timer0 postscaler is 1:1
    PIR0bits.TMR0IF = 0;        // reset Timer0 interrupt flag
    PIE0bits.TMR0IE = 1;        // enable TimerO interrupt
    T0CON0bits.T0EN = 1;        // enable Timer0 module
    
    // UART1
    // TX1 at RC4 & RX1 at RC5
    // edit : 16.11.2021
    TX1STAbits.BRGH = 1;    // High Baud Rate Select bit
    SPBRG = 0x19;           // 9600 with 4MHz oscillator
    
    
    RC4PPS = 0x0F;          // TX1 at pin RC4
    ANSELCbits.ANSC4 = 0;   // RC4 is digital
    TRISCbits.TRISC4 = 0;   // RC4 is output
    
    TX1STAbits.SYNC = 0;    // async operation
    TX1STAbits.TXEN = 1;    // enable EUSART transmitter
    
    // NEW REGISTER NAMES FOR RX1, use with DFP ver 1.23 and later  
    RX1PPSbits.RX1PPS4 = 1;        // RX1 at pin RC5
    RX1PPSbits.RX1PPS3 = 0;
    RX1PPSbits.RX1PPS2 = 1;
    RX1PPSbits.RX1PPS1 = 0;
    RX1PPSbits.RX1PPS0 = 1;
       
    // OLD REGISTER NAMES FOR RX1, use with DFP ver 1.15 and older
    /*
    RX1DTPPSbits.RX1DTPPS4 = 1;        // RX1 at pin RC5
    RX1DTPPSbits.RX1DTPPS3 = 0;
    RX1DTPPSbits.RX1DTPPS2 = 1;
    RX1DTPPSbits.RX1DTPPS1 = 0;
    RX1DTPPSbits.RX1DTPPS0 = 1;
    */
    
    ANSELCbits.ANSC5 = 0;   // RC5 is digital (default analog i.e. 1)
    TRISCbits.TRISC5 = 1;   // RC5 is input
          
    RC1STAbits.CREN = 1;    // continuous receive enable
    RC1STAbits.SPEN = 1;    // serial port enable (receiver)
    
    // LED
    TRISCbits.TRISC3 = 0;
    
    // ADC at RC1
    // edit : 17.12.2021
    TRISCbits.TRISC1 = 1;   // disable output driver
    ANSELCbits.ANSC1 = 1;   // analog input
    ADPCH = 0x11;           // b010001, RC1 Positive Input Channel (PCH) selection bits
    
    ADCON0bits.CS = 0;      // ADC clock supplied by Fosc
    //ADCLK = 0x01;           // b000001,  --> Fosc/(2*(ADCLK[7:0]+1)) = Fosc/4 --> 1 us Tad
    ADCLK = 0x03;           // b000011,  --> Fosc/(2*(ADCLK[7:0]+1)) = Fosc/8 --> 2 us Tad
    //ADCLK = 0x04;           // b000100,  --> Fosc/(2*(ADCLK[7:0]+1)) = Fosc/10 --> 4 us Tad
                 
    ADREFbits.NREF = 0; 	// 0, negative ref voltage is Vss
    ADREFbits.PREF1 = 1;    // b11, positive reference is internal Fixed Voltage Reference (FVR) module
    ADREFbits.PREF0 = 1;
    FVRCONbits.ADFVR1 = 1;  // b11, ADC FVR buffer gain is 4x --> 4.096 V
    FVRCONbits.ADFVR0 = 1;  
    FVRCONbits.FVREN = 1;	// Fixed Voltage Reference (FVR) is enabled
    
    ADCON0bits.FM = 1;      // data is right justified
    ADCON0bits.CONT = 0;    // continuous operation disabled
    ADCON2bits.MD = 0;      // Legacy mode, no DSP
    ADCON0bits.ON = 1;      // ADC is enabled
    //ADCPCON0bits.CPON = 1;  // activate charge pump // 10.12.2021 remove if unnecessary
    
    // DAC for LED intensity control
    // uses earlier set FVR value
    // edit : 6.5.2022
    TRISAbits.TRISA0 = 1;   // RA0 is output (necessary?)
    
    FVRCONbits.CDAFVR1 = 1; // Comparator & DAC FVR buffer gain is 4x (4.096)
    FVRCONbits.CDAFVR0 = 1;
    
    DAC1CON0bits.PSS1 = 1;  // PSS = [1,0] DAC positive source is FVR
    DAC1CON0bits.PSS0 = 0;  // 
    DAC1CON0bits.NSS = 0;   // 0 --> DAC negative source is Vss (gnd)
    DAC1CON0bits.OE1 = 1;   // 1 --> DAC output on DAC1OUT1 pin (RA0 / #13)
    
    DAC1CON1bits.DAC1R = 0x10;  // 1 000 --> half of DAC positive source out
    DAC1CON0bits.EN = 1;    // 1 --> DAC enabled
    
    // SPECTROMETER PINS
    TRISCbits.TRISC0 = 0;   // ST @ RC0 is output
    
    // OTHER INITS
    
    ANSELAbits.ANSA2 = 0;   // RA2 is analog input by default, make it digital
    TRISAbits.TRISA2 = 1;   // RA2 is input
    WPUAbits.WPUA2 = 1;     // activate weak-pull-up on RA2 input
    
    // ver 13.5.2022
    INTCONbits.GIE = 1;     // enable all active interrupts
    led_state = 0;
    LED = 0;
    integ_time = 10;        // default integration time is 10 ms
    getSpectrum();          // read one spectrum to init the sensor
    blinkLED(2);
                    
    // MAIN LOOP
    // edit : 2024-3-27
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
     */
    while(1){
        
        ms_delay(5);
        
        // test input pin RA2
        /*
        if (PORTAbits.RA2 == 1)
            LED = 1;
        else
            LED = 0;
        */
        
        if(PIR3bits.RC1IF){			// UART1 receive buffer data available
			command = getU1();
            
            switch(command){
                
                // First measure a new spectrum and then 
                // send the value of a single pre selected channel.
                // edit : 2024-3-28
                case 'S':
                   
                    putU1(command);
					putU1('\n');
					putU1('\r');
                    
                    // channel number 1...288
                    i = atoi(numBuffer);
                    numBuffer[0] = '\n';
                    numBuffer[1] = '\n';
                    numBuffer[2] = '\n';
                    numBuffer[3] = '\n';
                    b_index = 0;
                    
                    /*
                    if (i < 1 || i > 288)   // unnecessary ?
                        i = 115;
                    */
                    
                    ms_delay(50);           // 100 worked well
                    getSpectrum();          // updates ADCValues[]
                    				
                    // print channel index
                    intToArray(i);          // updates outputNumbers[]
					for(j=0; j<4; j++)
                        putU1(outputNumbers[j] + '0');
					putU1(' ');        
					// print channel value
					intToArray(ADCValues[i]); 
					for(j=0; j<4; j++)
						putU1(outputNumbers[j] + '0');
					putU1('\n');
                    putU1('\r');
					                    
                    break;
                    
                // Send one pre-selected value from already measured spectrum.
                // edit 2024-3-27
                case 'G':
                    putU1(command);
                    putU1('\n');
					putU1('\r');
                    
                    // channel number 1...288
                    i = atoi(numBuffer);
                    numBuffer[0] = '\n';
                    numBuffer[1] = '\n';
                    numBuffer[2] = '\n';
                    numBuffer[3] = '\n';
                    b_index = 0;
                                       
                    ms_delay(50);          // 100 worked well
                    
                    // print channel index
                    intToArray(i);          // updates outputNumbers[]
					for(j=0; j<4; j++)
                        putU1(outputNumbers[j] + '0');
					putU1(' ');        
					
                    // print channel value
					intToArray(ADCValues[i]); 
					for(j=0; j<4; j++)
						putU1(outputNumbers[j] + '0');
					putU1('\n');
                    putU1('\r');
                    
                    break;
                
                
                    
                // read a full spectrum
                // edit : 2024-3-28
                // todo : check if delay is necessary
				case 'A':
					putU1(command);
					putU1('\n');
					putU1('\r');
                    
                    ms_delay(50);  // let UART operations finish
                                                           
                    readingCount = getSpectrum();
                    intToArray(readingCount);             
                    //itoa(outputNumbers, readingCount, 10);  // Causes error in mini_main program : "invalid literal for int() with base 10"
                    for(j=0; j<4; j++)
						putU1(outputNumbers[j] + '0');
                    putU1('\n');
                    putU1('\r');
                    
					for(i=0; i<CH_COUNT; i++){
						// print channel index
						intToArray(i);
						for(j=0; j<4; j++)
							putU1(outputNumbers[j] + '0');
						putU1(' ');
						// print channel value
						intToArray(ADCValues[i]);
						for(j=0; j<4; j++)
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
                    if (b_index != 0){
                        blinkLED(atoi(numBuffer));
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

                    // read input buffer for integ time
                    if (b_index > 0)
                        integ_time = atoi(numBuffer);

                    // clear & reset input buffer
                    numBuffer[0] = '\n';
                    numBuffer[1] = '\n';
                    numBuffer[2] = '\n';
                    numBuffer[3] = '\n';
                    b_index = 0;
                    
                    intToArray(integ_time);
                    
                    for(i=0; i<4; i++)
						putU1(outputNumbers[i] + '0'); 	
					putU1(' ');
                    

                    break;
                    
                // reset and init
                // edit : 6.5.2022
                case 'R':
                    // zero led
                    led_state = 0;
                    LED = 0;
                    
                    // init spectrometer itegration time
                    integ_time = 10;        // 10 ms
                    
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
                    putU1(fVersion[0]+'0');
                    putU1(fVersion[1]+'0');
                    putU1(fVersion[2]+'0');
                    putU1(fVersion[3]+'0');
                    break;
                    
                // set LED intensity to preset (5-bit) value
                // edit 22.4.2022
                case 'H':
                    putU1(command);
                    putU1('\n');
                    putU1('\r');
                    
                    // led intensity value 0...31
                    source_int = atoi(numBuffer);
                    SourceIntensity(source_int);
                    numBuffer[0] = '\n';
                    numBuffer[1] = '\n';
                    numBuffer[2] = '\n';
                    numBuffer[3] = '\n';
                    b_index = 0;
                    
                    break;
               
                // read a value
                // edit : 10.12.2021
				case 'E':
					putU1(command);
					putU1('\n');
					putU1('\r');
                    intToArray(ReadADC());
                    putU1(outputNumbers[0]+'0');
                    putU1(outputNumbers[1]+'0');
                    putU1(outputNumbers[2]+'0');
                    putU1(outputNumbers[3]+'0');   
                    break;
                                  
                // check Fixed Voltage Reference status    
                case 'F':
                    putU1(command);
                    
                    if(FVRCONbits.FVRRDY == 1)  
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
                    if(RA2 == 1)
                        putU1('1');
                    else
                        putU1('0');
                    putU1('\n');
                    putU1('\r');
                    break;
                    
                                
                // default, capture possible numbers for later use    
                default:
                    if (command>='0' && command<='9' && b_index < 4){
                        numBuffer[b_index] = command; // ver 25.2.2021
                        b_index++;
                    }
                    break;               
            }
        }
    }
    return;
}

