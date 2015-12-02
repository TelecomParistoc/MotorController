/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB� Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB� Code Configurator - v2.25.2
        Device            :  PIC18F46K22
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set LCODER aliases
#define LCODER_TRIS               TRISB0
#define LCODER_LAT                LATB0
#define LCODER_PORT               PORTBbits.RB0
#define LCODER_WPU                WPUB0
#define LCODER_ANS                ANSB0
#define LCODER_SetHigh()    do { LATB0 = 1; } while(0)
#define LCODER_SetLow()   do { LATB0 = 0; } while(0)
#define LCODER_Toggle()   do { LATB0 = ~LATB0; } while(0)
#define LCODER_GetValue()         PORTBbits.RB0
#define LCODER_SetDigitalInput()    do { TRISB0 = 1; } while(0)
#define LCODER_SetDigitalOutput()   do { TRISB0 = 0; } while(0)

#define LCODER_SetPullup()    do { WPUB0 = 1; } while(0)
#define LCODER_ResetPullup()   do { WPUB0 = 0; } while(0)
#define LCODER_SetAnalogMode()   do { ANSB0 = 1; } while(0)
#define LCODER_SetDigitalMode()   do { ANSB0 = 0; } while(0)
// get/set RCODER aliases
#define RCODER_TRIS               TRISB1
#define RCODER_LAT                LATB1
#define RCODER_PORT               PORTBbits.RB1
#define RCODER_WPU                WPUB1
#define RCODER_ANS                ANSB1
#define RCODER_SetHigh()    do { LATB1 = 1; } while(0)
#define RCODER_SetLow()   do { LATB1 = 0; } while(0)
#define RCODER_Toggle()   do { LATB1 = ~LATB1; } while(0)
#define RCODER_GetValue()         PORTBbits.RB1
#define RCODER_SetDigitalInput()    do { TRISB1 = 1; } while(0)
#define RCODER_SetDigitalOutput()   do { TRISB1 = 0; } while(0)

#define RCODER_SetPullup()    do { WPUB1 = 1; } while(0)
#define RCODER_ResetPullup()   do { WPUB1 = 0; } while(0)
#define RCODER_SetAnalogMode()   do { ANSB1 = 1; } while(0)
#define RCODER_SetDigitalMode()   do { ANSB1 = 0; } while(0)
// get/set IN2A aliases
#define IN2A_TRIS               TRISB2
#define IN2A_LAT                LATB2
#define IN2A_PORT               PORTBbits.RB2
#define IN2A_WPU                WPUB2
#define IN2A_ANS                ANSB2
#define IN2A_SetHigh()    do { LATB2 = 1; } while(0)
#define IN2A_SetLow()   do { LATB2 = 0; } while(0)
#define IN2A_Toggle()   do { LATB2 = ~LATB2; } while(0)
#define IN2A_GetValue()         PORTBbits.RB2
#define IN2A_SetDigitalInput()    do { TRISB2 = 1; } while(0)
#define IN2A_SetDigitalOutput()   do { TRISB2 = 0; } while(0)

#define IN2A_SetPullup()    do { WPUB2 = 1; } while(0)
#define IN2A_ResetPullup()   do { WPUB2 = 0; } while(0)
#define IN2A_SetAnalogMode()   do { ANSB2 = 1; } while(0)
#define IN2A_SetDigitalMode()   do { ANSB2 = 0; } while(0)
// get/set IN2B aliases
#define IN2B_TRIS               TRISB3
#define IN2B_LAT                LATB3
#define IN2B_PORT               PORTBbits.RB3
#define IN2B_WPU                WPUB3
#define IN2B_ANS                ANSB3
#define IN2B_SetHigh()    do { LATB3 = 1; } while(0)
#define IN2B_SetLow()   do { LATB3 = 0; } while(0)
#define IN2B_Toggle()   do { LATB3 = ~LATB3; } while(0)
#define IN2B_GetValue()         PORTBbits.RB3
#define IN2B_SetDigitalInput()    do { TRISB3 = 1; } while(0)
#define IN2B_SetDigitalOutput()   do { TRISB3 = 0; } while(0)

#define IN2B_SetPullup()    do { WPUB3 = 1; } while(0)
#define IN2B_ResetPullup()   do { WPUB3 = 0; } while(0)
#define IN2B_SetAnalogMode()   do { ANSB3 = 1; } while(0)
#define IN2B_SetDigitalMode()   do { ANSB3 = 0; } while(0)
// get/set IN1A aliases
#define IN1A_TRIS               TRISB4
#define IN1A_LAT                LATB4
#define IN1A_PORT               PORTBbits.RB4
#define IN1A_WPU                WPUB4
#define IN1A_ANS                ANSB4
#define IN1A_SetHigh()    do { LATB4 = 1; } while(0)
#define IN1A_SetLow()   do { LATB4 = 0; } while(0)
#define IN1A_Toggle()   do { LATB4 = ~LATB4; } while(0)
#define IN1A_GetValue()         PORTBbits.RB4
#define IN1A_SetDigitalInput()    do { TRISB4 = 1; } while(0)
#define IN1A_SetDigitalOutput()   do { TRISB4 = 0; } while(0)

#define IN1A_SetPullup()    do { WPUB4 = 1; } while(0)
#define IN1A_ResetPullup()   do { WPUB4 = 0; } while(0)
#define IN1A_SetAnalogMode()   do { ANSB4 = 1; } while(0)
#define IN1A_SetDigitalMode()   do { ANSB4 = 0; } while(0)
// get/set IN1B aliases
#define IN1B_TRIS               TRISB5
#define IN1B_LAT                LATB5
#define IN1B_PORT               PORTBbits.RB5
#define IN1B_WPU                WPUB5
#define IN1B_ANS                ANSB5
#define IN1B_SetHigh()    do { LATB5 = 1; } while(0)
#define IN1B_SetLow()   do { LATB5 = 0; } while(0)
#define IN1B_Toggle()   do { LATB5 = ~LATB5; } while(0)
#define IN1B_GetValue()         PORTBbits.RB5
#define IN1B_SetDigitalInput()    do { TRISB5 = 1; } while(0)
#define IN1B_SetDigitalOutput()   do { TRISB5 = 0; } while(0)

#define IN1B_SetPullup()    do { WPUB5 = 1; } while(0)
#define IN1B_ResetPullup()   do { WPUB5 = 0; } while(0)
#define IN1B_SetAnalogMode()   do { ANSB5 = 1; } while(0)
#define IN1B_SetDigitalMode()   do { ANSB5 = 0; } while(0)
// get/set INTOUT aliases
#define INTOUT_TRIS               TRISC0
#define INTOUT_LAT                LATC0
#define INTOUT_PORT               PORTCbits.RC0
#define INTOUT_SetHigh()    do { LATC0 = 1; } while(0)
#define INTOUT_SetLow()   do { LATC0 = 0; } while(0)
#define INTOUT_Toggle()   do { LATC0 = ~LATC0; } while(0)
#define INTOUT_GetValue()         PORTCbits.RC0
#define INTOUT_SetDigitalInput()    do { TRISC0 = 1; } while(0)
#define INTOUT_SetDigitalOutput()   do { TRISC0 = 0; } while(0)

// get/set P2A aliases
#define P2A_TRIS               TRISC1
#define P2A_LAT                LATC1
#define P2A_PORT               PORTCbits.RC1
#define P2A_SetHigh()    do { LATC1 = 1; } while(0)
#define P2A_SetLow()   do { LATC1 = 0; } while(0)
#define P2A_Toggle()   do { LATC1 = ~LATC1; } while(0)
#define P2A_GetValue()         PORTCbits.RC1
#define P2A_SetDigitalInput()    do { TRISC1 = 1; } while(0)
#define P2A_SetDigitalOutput()   do { TRISC1 = 0; } while(0)

// get/set P1A aliases
#define P1A_TRIS               TRISC2
#define P1A_LAT                LATC2
#define P1A_PORT               PORTCbits.RC2
#define P1A_ANS                ANSC2
#define P1A_SetHigh()    do { LATC2 = 1; } while(0)
#define P1A_SetLow()   do { LATC2 = 0; } while(0)
#define P1A_Toggle()   do { LATC2 = ~LATC2; } while(0)
#define P1A_GetValue()         PORTCbits.RC2
#define P1A_SetDigitalInput()    do { TRISC2 = 1; } while(0)
#define P1A_SetDigitalOutput()   do { TRISC2 = 0; } while(0)

#define P1A_SetAnalogMode()   do { ANSC2 = 1; } while(0)
#define P1A_SetDigitalMode()   do { ANSC2 = 0; } while(0)
// get/set SCL1 aliases
#define SCL1_TRIS               TRISC3
#define SCL1_LAT                LATC3
#define SCL1_PORT               PORTCbits.RC3
#define SCL1_ANS                ANSC3
#define SCL1_SetHigh()    do { LATC3 = 1; } while(0)
#define SCL1_SetLow()   do { LATC3 = 0; } while(0)
#define SCL1_Toggle()   do { LATC3 = ~LATC3; } while(0)
#define SCL1_GetValue()         PORTCbits.RC3
#define SCL1_SetDigitalInput()    do { TRISC3 = 1; } while(0)
#define SCL1_SetDigitalOutput()   do { TRISC3 = 0; } while(0)

#define SCL1_SetAnalogMode()   do { ANSC3 = 1; } while(0)
#define SCL1_SetDigitalMode()   do { ANSC3 = 0; } while(0)
// get/set SDA1 aliases
#define SDA1_TRIS               TRISC4
#define SDA1_LAT                LATC4
#define SDA1_PORT               PORTCbits.RC4
#define SDA1_ANS                ANSC4
#define SDA1_SetHigh()    do { LATC4 = 1; } while(0)
#define SDA1_SetLow()   do { LATC4 = 0; } while(0)
#define SDA1_Toggle()   do { LATC4 = ~LATC4; } while(0)
#define SDA1_GetValue()         PORTCbits.RC4
#define SDA1_SetDigitalInput()    do { TRISC4 = 1; } while(0)
#define SDA1_SetDigitalOutput()   do { TRISC4 = 0; } while(0)

#define SDA1_SetAnalogMode()   do { ANSC4 = 1; } while(0)
#define SDA1_SetDigitalMode()   do { ANSC4 = 0; } while(0)
// get/set TX1 aliases
#define TX1_TRIS               TRISC6
#define TX1_LAT                LATC6
#define TX1_PORT               PORTCbits.RC6
#define TX1_ANS                ANSC6
#define TX1_SetHigh()    do { LATC6 = 1; } while(0)
#define TX1_SetLow()   do { LATC6 = 0; } while(0)
#define TX1_Toggle()   do { LATC6 = ~LATC6; } while(0)
#define TX1_GetValue()         PORTCbits.RC6
#define TX1_SetDigitalInput()    do { TRISC6 = 1; } while(0)
#define TX1_SetDigitalOutput()   do { TRISC6 = 0; } while(0)

#define TX1_SetAnalogMode()   do { ANSC6 = 1; } while(0)
#define TX1_SetDigitalMode()   do { ANSC6 = 0; } while(0)
// get/set RX1 aliases
#define RX1_TRIS               TRISC7
#define RX1_LAT                LATC7
#define RX1_PORT               PORTCbits.RC7
#define RX1_ANS                ANSC7
#define RX1_SetHigh()    do { LATC7 = 1; } while(0)
#define RX1_SetLow()   do { LATC7 = 0; } while(0)
#define RX1_Toggle()   do { LATC7 = ~LATC7; } while(0)
#define RX1_GetValue()         PORTCbits.RC7
#define RX1_SetDigitalInput()    do { TRISC7 = 1; } while(0)
#define RX1_SetDigitalOutput()   do { TRISC7 = 0; } while(0)

#define RX1_SetAnalogMode()   do { ANSC7 = 1; } while(0)
#define RX1_SetDigitalMode()   do { ANSC7 = 0; } while(0)
// get/set CCP4 aliases
#define CCP4_TRIS               TRISD1
#define CCP4_LAT                LATD1
#define CCP4_PORT               PORTDbits.RD1
#define CCP4_ANS                ANSD1
#define CCP4_SetHigh()    do { LATD1 = 1; } while(0)
#define CCP4_SetLow()   do { LATD1 = 0; } while(0)
#define CCP4_Toggle()   do { LATD1 = ~LATD1; } while(0)
#define CCP4_GetValue()         PORTDbits.RD1
#define CCP4_SetDigitalInput()    do { TRISD1 = 1; } while(0)
#define CCP4_SetDigitalOutput()   do { TRISD1 = 0; } while(0)

#define CCP4_SetAnalogMode()   do { ANSD1 = 1; } while(0)
#define CCP4_SetDigitalMode()   do { ANSD1 = 0; } while(0)
// get/set CCP5 aliases
#define CCP5_TRIS               TRISE2
#define CCP5_LAT                LATE2
#define CCP5_PORT               PORTEbits.RE2
#define CCP5_ANS                ANSE2
#define CCP5_SetHigh()    do { LATE2 = 1; } while(0)
#define CCP5_SetLow()   do { LATE2 = 0; } while(0)
#define CCP5_Toggle()   do { LATE2 = ~LATE2; } while(0)
#define CCP5_GetValue()         PORTEbits.RE2
#define CCP5_SetDigitalInput()    do { TRISE2 = 1; } while(0)
#define CCP5_SetDigitalOutput()   do { TRISE2 = 0; } while(0)

#define CCP5_SetAnalogMode()   do { ANSE2 = 1; } while(0)
#define CCP5_SetDigitalMode()   do { ANSE2 = 0; } while(0)

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);

#endif // PIN_MANAGER_H
/**
 End of File
 */