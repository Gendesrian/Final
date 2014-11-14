// ************************************************************************** //
// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.

//************************** H-BRIDGE PIN MAPPING ****************************//
//pin10 is enable to pin1h and pin9h
//pin7 goes to pin2h
//pin14 goes to pin7h
//right wheel red to pin3h
//right wheel black to pin6h
//left wheel red to pin14h
//left wheel black to pin11h
//+5v to pin8h and pin16h
//pin21 goes to pin10h
//pin22 goes to pin15h
//ground pin4h,5h,12h,13h
//************************ END H-BRIDGE PIN MAPPING **************************//

// Pins used
//P2 used for AN0 Potentiameter Input
//P3 used for AN1 Left PhotoTransistor
//P4 used for AN2 Middle PhotoTransistor
//P5 used for AN3 Right PhotoTransistor
//P6 used for AN4 Barcode PhotoTransistor
//P7 used for RP3 OC3R/RS ground Right Motor // swapping pin for Reverse
//P10 used for RA3 H-bridge Enable
//P14 used for RP5 OC1R/RS pulses Right Motor // swapping pin for Reverse
//P15 used for LCD Enable
//P16 used for LCD RS
//P17 used for RP8 UART1 TX Output
//P18 used for RP9 UART1 RX Input
//P21 used for RP10 OC2R/RS pulses Left Motor // swapping pin for Reverse
//P22 used for RP11 OC3R/RS ground Left Motor // swapping pin for Reverse
//P23 used for LCD Data/Control
//P24 used for LCD Data/Control
//P25 used for LCD Data/Control
//P26 used for LCD Data/Control

#include "p24fj64ga002.h"
#include <stdio.h>
#include "lcd.h"

#define rightBaseSpeed 1023
#define leftBaseSpeed 1023
#define base 1023


// ************************************************************************** //
// Configuration bits for CONFIG1 settings.
//
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
//
// These settings are appropriate for debugging the PIC microcontroller. If you need to
// program the PIC for standalone operation, change the COE_ON option to COE_OFF.

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &
		 BKBUG_ON & COE_ON & ICS_PGx1 &
		 FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )



// ************************************************************************** //
// Configuration bits for CONFIG2 settings.
// Make sure "Configuration Bits set in code." option is checked in MPLAB.

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD
        & OSCIOFNC_OFF & IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )



#define XTFREQ          7372800         	  // On-board Crystal frequency
#define PLLMODE         4               	  // On-chip PLL setting (Fosc)
#define FCY             (XTFREQ*PLLMODE)/2    // Instruction Cycle Frequency (Fosc/2)

#define BAUDRATE        9600
#define BRGVAL          ((FCY/BAUDRATE)/16)-1


    volatile int state = 0;                                                     // Initialize variable for state machine
    volatile int nextState = 1;                                                 // Initialize variable for next state to go to
    volatile int previousState = 1;                                             // Initialize variable for previous run direction state

int main(void){

// ****************************** INITIALIZE ******************************** //

//    unsigned long int temp;                                                     // Initialize temp variable for calc ADC_value
//   int ADC_value;                                                              // Initialize integer to hold average of adcBuff sampling
    unsigned int adcBuff[16], i =0;                                             // Initalize buffer for sampling
    unsigned int * adcPtr;
    int receivedChar;                                                           // Initialize User Char Variable
    int mode = 0;                                                               // Initialize Mode Variable
    int right = 0;                                                              // Initialize Right Reduction Variable
    int left = 0;                                                               // Initialize Left Reduction Variable
    int inc = -1;
    int sum = 0;
    int motorSpeed = 0;
    int rightMotorSpeed = 0;
    int leftMotorSpeed = 0;
    int error = 0;
    int base = 0;
    int lastError;


    OC1CON = 0x000E;                                                            // Set OC1CON to PWM w/o protection
    OC2CON = 0x000E;                                                            // Set OC2CON to PWM w/o protection
    OC3CON = 0x000E;                                                            // Set OC3CON to PWM w/o protection

    OC1R = 0;                                                                   // Initialize duty cycle for OC1R to 0%
    OC2R = 0;                                                                   // Initialize duty cycle for OC2R to 0%
    OC3R = 0;                                                                   // Initialize duty cycle for OC3R to 0%
 

    T3CON = 0x8020;                                                             // Turn on and set prescaler to 64
    TMR3 = 0;                                                                   // Reset Timer 3 to 0
    PR3 = 512;                                                                  // 2.2 ms timer

    IFS0bits.T3IF = 0;                                                          // Put down interupt flag
    IEC0bits.T3IE = 0;                                                          // Do not enable ISR

    LCDInitialize( );                                                           // Initialize LCD

    //////////RECONFIGURE THIS///////////////////
    AD1PCFG &= 0xFFE0;                                                          // AN0-AN4 input pin is analog(0), rest all to digital pins(1)
    //AD1CON2 = 0x003C;                                                           // Sets SMPI(sample sequences per interrupt) to 1111, 16th sample/convert sequence

    AD1CON2 = 0x0015;                                                           //set for 5 buffers
    AD1CON3 = 0x0D01;                                                           // Set SAMC<12:8>(Auto-Sampe Time Bits(TAD)) =  13, ADCS<7:0> = 1 -> 100ns conversion time
    AD1CON1 = 0x20E4;                                                           // ADSIDL<13> = 1, SSRC<7-5> = 111(conversion trigger source select - auto convert)
   // AD1CHS = 0;                                                                 // Configure input channels, connect AN0 as positive input
    AD1CSSL = 0;                                                                // No inputs are scanned
    AD1CON1bits.ADON = 1;                                                       // Turn ADC on

    TRISAbits.TRISA3 = 0;                                                       // RA3 = pin 10 as output
    PORTAbits.RA3 = 0;                                                          // Turn H-Bridge on
    
  
//////////////////////////////UART Configuration////////////////////////////////

	RPINR18bits.U1RXR = 9;                                                  // RP9 used for UART1 RX Input
	RPOR4bits.RP8R = 3;                                                     // RP8 used for UART1 TX Output
	U1BRG  = BRGVAL;
	U1MODE = 0x8000;
	U1STA  = 0x0440;                                                        // Reset status register and enable TX & RX
	IFS0bits.U1RXIF = 0;                                                    // Clear UART RX interrupt flag
	printf("\n\n\nROBOT ONLINE\n\n\n");
	printf("Enter Mode. (A)utonomous or (C)ontrolled: ");


// **************************** INITIALIZE END ****************************** //

        OC1RS = 0;
        OC2RS = 0;


    while(1){
        if(IFS0bits.U1RXIF == 1) {                                              // If character received
            receivedChar = U1RXREG;                                             // Assign entered char to received Char

            if (mode == 0){                                                     // Idle Mode
                if( receivedChar == 'A' || receivedChar == 'a') {               // Autonomous Mode Selected
                    printf("%c\n\r", receivedChar);                             // Print receivedChar
                    printf("Autonomous Mode Selected!\n");                      // Print a confirmation message.
                    LCDMoveCursor(0,0);                                         // Move cursor to start of line 1
                    LCDPrintString("Auto");                                     // Print Auto to the LCD
                    printf("Press (T)oggle to start/stop or (Q)uit: ");         // Print Usage Statement
                    mode = 1;
                }

                else if ( receivedChar == 'C' || receivedChar == 'c'){          // Contorlled Mode Selected
                    printf("%c\n\r", receivedChar);                             // Print receivedChar
                    printf("Controlled Mode Selected!\n");                      // Print a confirmation message.
                    LCDMoveCursor(0,0);                                         // Move cursor to start of line 1
                    LCDPrintString("Control");                                  // Print value2 to the lcd
                    printf("Use (1) to Start,(2) to reverse, (3) to stop, or (Q)uit.\n");
                    printf("Use (4) to veer Left. Use (6) to veer Right.\n");
                    mode = 2;
                }

                else if (receivedChar != 'A' || receivedChar != 'C'
                        || receivedChar != 'Q' || receivedChar != 'T'
                        || receivedChar != 'a' || receivedChar != 'c'
                        || receivedChar != 'q' || receivedChar != 't') {        // Invalid Mode Case
                        printf("\n\n\nInvalid Mode Selected!\n\n\n");           // Display error message.
                       printf("Enter Mode. (A)utonomous or (C)ontrolled: ");    // Reprint Usage Statement
		}
            }

            if(mode == 1){                                                      // Autonomous Mode
                if(receivedChar == 'T' || receivedChar == 't'){                 // Toggle Case
                    state = 4;                                                  // Toggle Forward/Idle/Reverse
                }
                if(receivedChar == 'Q' || receivedChar == 'q'){                 // Quit Case
                    OC1RS = 0;                                                  // Turn off Left Motor
                    OC2RS = 0;                                                  // Turn off Right Motor
                    mode = 0;                                                   // Set Mode to Idle Mode
                    LCDClear();                                                 // Clear LCD
                    printf("\n\n\nEnding (A)utonomous!\n\n\n");                 // Print Quit Confirmation
                    printf("Enter Mode. (A)utonomous or (C)ontrolled: ");       // Reprint Usage Statement
                }
                if(receivedChar == 'A' || receivedChar == 'a'){
                }
            }

            if(mode == 2){                                                      // Contolled Mode
                if(receivedChar == '1'){                                        // Forward Case
                    RPOR2bits.RP5R = 18;                                        // Pin14 used for OC1 pulses
                    RPOR1bits.RP3R = 20;                                        // Pin7 used for OC1 ground
                    RPOR5bits.RP10R = 19;                                       // Pin21 used for OC2 pulse
                    RPOR5bits.RP11R = 20;                                       // Pin22 used for OC2 ground
                    right = 1;                                                  // Initalize right reduction variable
                    left = 1;                                                   // Initalize left reduction variable
                    OC1RS = 1021;                                               // Set right Duty Cycle to 100%
                    OC2RS = 1021;                                               // Set left Duty Cycle to 100%
                }

                if(receivedChar == '3'){                                        // Idle Case
                    OC1RS = 0;                                                  // Stop right motor
                    OC2RS = 0;                                                  // Stop left motor
                }

                if(receivedChar == '6'){                                        // Right Turn
                    right = right + 340;                                        // Incriment right motor reduction
                    left = left - 340;                                          // Decriment left motor reduction
                    if (right > 1020){                                          // If right incremented to high
                        right = 1020;                                           // Set to max
                    }
                    if (left < 0){                                              // If left Decrimented to low
                        left = 1;                                               // Set to min
                    }

                    OC1RS = 1020 - right;                                       // Set right motor duty cycle
                    OC2RS = 1020 - left;                                        // Set left motor duty cycle
                }

                if(receivedChar == '4'){                                        // Left Turn
                    right = right - 340;                                        // Decriment right motor reduction
                    left = left + 340;                                          // Incriment left motor reduction
                    if (right < 0){                                             // If right Decrimented to low
                        right = 1;                                              // Set to min
                    }
                    if (left > 1020){                                           // If left Incrimented to high
                        left = 1020;                                            // Set to max
                    }

                    OC1RS = 1020 - right;                                       // Set right motor duty cycle
                    OC2RS = 1020 - left;                                        // Set left motor duty cycle
                }

                if(receivedChar == '2'){                                        // Reverse Case
                    RPOR2bits.RP5R = 20;                                        // Pin14 used for OC1 pulse
                    RPOR1bits.RP3R = 18;                                        // Pin7 used for OC1 ground
                    RPOR5bits.RP10R = 20;                                       // Pin21 used for OC2 pulse
                    RPOR5bits.RP11R = 19;                                       // Pin22 used for OC2 ground
                    right = 1;                                                  // Initialize right reduction value
                    left = 1;                                                   // Initialize left reduction value
                    OC1RS = 1021;                                               // Set right Duty Cycle to 100%
                    OC2RS = 1021;                                               // Set left Duty Cycle to 100%
                }

                if(receivedChar == 'Q' || receivedChar == 'q'){                 // Quit Case
                    right = 1;                                                  // Reset right reduction
                    left = 1;                                                   // Reset left reduction
                    OC1RS = 0;                                                  // Stop right motor
                    OC2RS = 0;                                                  // Stop left motor
                    mode = 0;                                                   // Set Mode to Idle
                    LCDClear();                                                 // Clear LCD
                    printf("\n\n\nEnding (C)ontrolled!\n\n\n");                 // Print Quit Confirmation
                    printf("Enter Mode. (A)utonomous or (C)ontrolled: \n");     // Reprint Usage Statement
                }

            }
	IFS0bits.U1RXIF = 0;                                            	// Clear the UART RX interrupt flag

        }

        if (mode == 1){                                                         // Autonomous Mode Operation
 
// ******************************* SAMPLING ********************************* //

            while(!IFS0bits.AD1IF);
             adcPtr = (unsigned int *)(&ADC1BUF0);
             IFS0bits.AD1IF = 0;
             sum = 0;
             for (i = 0; i < 4; i++ ) {
                 AD1CHS = inc + 1;
                 adcBuff[i] = *adcPtr++;
             }

             sum = adcBuff[2] + adcBuff[3] - adcBuff[1];
             inc = -1;
             // implement adcBuff[4]  for barcode



// ***************************** SAMPLING END ******************************* //



// ***************************** CALCULATIONS ******************************* //
             /* // 0 is tape
                // 1 is not tape
             if(sum == 0) {
                 //go straight
                 //OC1RS = base;
                 //OC2RS = base;
             }

             if (far left is zero ) {
                turn left;
             }

             if (far right is zero) {
                turn right;
              }

              if (far left and far right = 1) {
                PID MODE
              }

              if (far left and right = 0) {
                counter ++;


                  //checking counter
                  if (counter < 3) {
                    turn right 90*;
                  }

                  if (counter == 3) {
                    turn 180;
                  }

                  if (counter > 3) {
                    turn left 90;
                  }
              }
             */


             //PID MODE
             error = base - sum;
             motorSpeed = error + (error-lastError);
             lastError = error;

             OC1RS = rightBaseSpeed + motorSpeed;                               //right motor speed
             OC2RS = leftBaseSpeed + motorSpeed;                                // left motor speed

              printf("\nadcBuff[1] = %d\n", adcBuff[1]);
              printf("\nadcBuff[2] = %d\n", adcBuff[2]);
              printf("\nadcBuff[3] = %d\n", adcBuff[3]);

              printf("\nError = %d\n", error);
              printf("\nlastError = %d\n", lastError);
              printf("\nSum = %d\n", sum);
              printf("\nMotorSpeed = %d\n", motorSpeed);


//                OC1RS =  1023 - ADC_value;                                      // Load OC1RS buffer with the opposite of OC2RS
//                OC2RS = ADC_value;                                              // Load OC2RS buffer with the value of ADC_value
//
//                if (ADC_value >= 512 ){                                         // If pot more than 1/2
//                oc1Temp = (OC1RS * 100) / 512;                                  // Calculate duty cycle of OC1RS and store to oc1Temp
//                oc2Temp = 100;                                                  // Store duty cycle of 100% to oc2Temp
//                }
//
//                if (ADC_value <= 511){                                          // If pot lesss than 1/2
//                oc1Temp = 100;                                                  // Store dutycycle of 100% to oc1Temp
//                oc2Temp = (OC2RS * 100) / 512;                                  // Calculate duty cycle of OC2RS and store to oc2Temp
//                }
// *************************** CALCULATIONS END ***************************** //


/////////////////////////////////NEW CODE///////////////////////////////////////
            // REMEMBER 1 is black tape
            // REMEMBER 0 is No tape

            // Use ADC for barcode?????
            // Consider Using Change notification for Off center

//            if ( not found){
//                OC1RS = 1023;
//                OC2RS = 1023;
//            }
//
//            if ( line found) {
//
//                if ( middle sensor == 1 && left sensor == 0 && right sensor == 0){//No Correction Req
//                check for barcode
//                }
//
//                if ( middle sensor == 0){                                       //Correction Req
//                    if (left sensor == 1 ){                                     //To Far Right
//                        reduce OC2RS                                            //slow left motor
//                    }
//                    if (right sensor == 1){                                     //To Far Left
//                        reduce OC1RS                                            //slow right motor
//                    }
//                }
//
//                if ( middle sensor == 1 && left sensor == 0 && right sensor ==1){//Loop case right turn
//                    turn 90 degrees right
//                }
//
//                if ( middle sensor == 1 && left sesnor == 1 && right sensor == 0){//Loop case Left turn
//                    turn 90 degrees left
//                }
//
//                if ( middle sensor == 1 && left sesnor == 1 && right sensor == 1){//End of Track case
//                    turn 180 degrees
//                }
//            }
/////////////////////////////////END NEW CODE///////////////////////////////////
        }   
    }
        return;
}

void _ISR_ADC1Interrupt(void) {                                                 // ISR for Analog to Digital conversion

    AD1CON1bits.ASAM = 0;                                                       //
    IFS0bits.AD1IF = 0;                                                         // Put down ISR flag

}
