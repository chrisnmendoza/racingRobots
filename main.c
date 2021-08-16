/*===================================CPEG222====================================
 * Program:		Project 5: Robot Racing
 * Authors: 	Christopher-Neil Mendoza
 * Date: 		11/17/2019
 * Code for CPEG222 Project 5: Racing Robots
 * 
==============================================================================*/
/*------------------ Board system settings. PLEASE DO NOT MODIFY THIS PART ----------*/
#ifndef _SUPPRESS_PLIB_WARNING          //suppress the plib warning during compiling
    #define _SUPPRESS_PLIB_WARNING      
#endif
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
/*----------------------------------------------------------------------------*/
     
#include <xc.h>   //Microchip XC processor header which links to the PIC32MX370512L header
#include <sys/attribs.h>
#include "config.h"
#include "swt.h"
#include "utils.h"
#include "lcd.h"
#include "led.h"
#include <stdio.h>
/* --------------------------- Forward Declarations-------------------------- */
void PWM_Config(void);
void Timer3Config(void);
void OC_Config(void);
/* -------------------------- Definitions------------------------------------ */
#define SYS_FREQ    (80000000L) // 80MHz system clock
int LD_position = 0x01;         // Initially turn on LD0
int buttonLock = 0;             // Variable to "lock" the button
int delay = 100;                // Variable to set delay to 100 ms
#define SW0 PORTFbits.RF3
#define SW1 PORTFbits.RF5
#define SW2 PORTFbits.RF4
#define SW3 PORTDbits.RD15
#define SW4 PORTDbits.RD14
#define SW5 PORTBbits.RB11
#define SW6 PORTBbits.RB10
#define SW7 PORTBbits.RB9
#define led0 LATAbits.LATA0
#define led1 LATAbits.LATA1
#define led2 LATAbits.LATA2
#define led3 LATAbits.LATA3
#define led4 LATAbits.LATA4
#define led5 LATAbits.LATA5
#define led6 LATAbits.LATA6
#define led7 LATAbits.LATA7
#define btnr PORTBbits.RB8
#define btnc PORTFbits.RF0

/* ----------------------------- Main --------------------------------------- */
int main(void){
    DDPCONbits.JTAGEN = 0;      // Statement is required to use Pin RA0 as IO
    LED_Init();
    SWT_Init();
    LCD_Init();
    Timer3Config();
    OC_Config();
    PWM_Config();
    //*******NOTE: for pulse width = 1.5 ms, ocxrs = 1875
    LCD_WriteStringAtPos("Team 25: Beemo",0,0);
    LCD_WriteStringAtPos("STP          STP",1,0);
    while(1){
        if(SW0 && !SW1){    //Right Motor FWD CW
            LCD_WriteStringAtPos("FWD",1,0);
            led3=led2=1;
            led1=led0=0;
        }
        else if(!SW0 && SW1){   //Right Motor CCW
            LCD_WriteStringAtPos("REV",1,0);
            led1=led0=1;
            led3=led2=0;
        }
        else if(SW0 ^ !SW1){    //Right Motor STP
            LCD_WriteStringAtPos("STP",1,0);
            led0=led1=led2=led3=0;
        }
        if(SW6 && !SW7){
            LCD_WriteStringAtPos("FWD",1,13);   //Left Motor CCW
            led3=led2=0;
            led1=led0=1;
        }
        else if(!SW6 && SW7){
            LCD_WriteStringAtPos("REV",1,13);   //Left Motor CW
            led1=led0=0;
            led3=led2=1;
        }
        else if(SW6 ^ !SW7){
            LCD_WriteStringAtPos("STP",1,13);   //Left Motor STP
            led0=led1=led2=led3=0;
        }
    }
    return 0;
}
/* ------------------------ End of Main --------------------------------------*/

void Timer3Config(void){
    PR3 = (int)((float)(10000000)/(50*8)-1);     //Sets period so that timer activates at 50Hz given prescalar == 8
    TMR3 = 0; //Initialize count to 0
    T3CONbits.TCKPS = 3; //Prescalar = 8
    T3CONbits.TGATE = 0; //Not gated input (default)
    T3CONbits.TCS = 0; //PCBLK input (default)
    T3CONbits.ON = 1;//Turn on timer 2
    IPC3bits.T3IP = 7; //sets priority
    IPC3bits.T3IS = 3; //set subpriority
    IFS0bits.T3IF = 0; //clears flag
    IEC0bits.T3IE=1; //enable interrupt
    macro_enable_interrupts(); //enable cpu interrupt
}

void OC_Config(void){
    OC4CONbits.ON=0;
    OC4CONbits.OCM=6;
    OC4CONbits.OCTSEL=1;
    OC4CONbits.ON=1;
    OC5CONbits.ON=0;
    OC5CONbits.OCM=6;
    OC5CONbits.OCTSEL=1;
    OC5CONbits.ON=1;
    OC5RS=1875;
    OC4RS=1875;
}

void PWM_Config(void){
    TRISBbits.TRISB8 = 0;
    TRISAbits.TRISA15 = 0;
    ANSELBbits.ANSB8 = 0;
    RPB8R = 0x0B; //1011 = OC5
    RPA15R = 0x0B; //1011 = OC4
}
