/* 
 * File:   HubMain.c
 * Author: nikolas
 *
 * Created on December 12, 2017, 11:07 AM
 */

/* This is the hub main code of a two part Hub-Controller project
 * The hub would be responsible to receive commands from the controller and do 
 * task accordingly. The code would do the following:
 * - Receive sync from controller microcontroller through SPI
 * - On mode 1, The HUB would: Read temperature from the temperature sensor,
 *   adjust LED hue according on the temperature (Hot = Red, Cold = blue).
 *   This would be done using PWM cycle
 *   also Count down timer, bulb should turn off when timer gets to 0
 * - On mode 2, The HUB would : Adjust LED Intensity according to the command
 *   Turn Bulb on off according to command 
 */
//Include libraries
#include <p18cxxx.h>
#include <stdio.h>
#include <stdlib.h>
#include "LCDroutinesEasyPic.h"
#include <adc.h>
#include <math.h>
#include <delays.h>
#include <string.h>
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

//Declare variables
char LCDstart1[] = {0x80,'M','O','D','E','=','1',' ',' ','T','E','M','P','=','0','0','C',0x00};
char LCDstart2[] = {0xC0,'T','I','M','E','R','=','0','0',':','0','0',0x00};
char mode = 1; //mode at which the hub is working on 


//Declare Functions
void Initial(void);
void InitialADC(void);
void ReadTemp(void);
void InitialTimer(void);
void TMR0Handler(void);
void InitialPWM(void);
void ChangePWM(void);
/*
 * 
 */
void main(void) {
    Initial();
    while(1){
        ReadTemp();
        Delay10KTCYx(10);
    }
}

void Initial(){
    // Configure the LCD pins for output. Defined in LCDRoutines.h
    LCD_RS_TRIS   = 0;              // 
    LCD_E_TRIS    = 0;
    LCD_DATA_TRIS = 0b11000000;     // Note the LCD is only on the upper nibble
                                    // The lower nibble is all inputs
    LCD_DATA_LAT = 0;           // Initialize LCD data LAT to zero
    // Initialize the LCD
    InitLCD();
    
    INTCONbits.GIEH=1;  //Enable high priority interrupt
    INTCONbits.GIEL=1;  //Enable low priority interrupt
    RCONbits.IPEN = 1;  // Enable priority levels
    
    InitialADC();       //Initialize ADC for temperature sensor
    
    InitialTimer();     //Initialize timer 0 for the use of bulb timer
    
    InitialPWM();
    
    DisplayV(LCDstart1);    //Display LCD initial interface
    DisplayV(LCDstart2);
}

//Using CCP 4, 5, and 6 for generating PWM signal that would control the i
//Intensity of R, G and B LED on the RGB LED.
void InitialPWM(){
    
}

void InitialADC(){
    
}

void InitialTimer(){
    T0CON = 0b00000111;             //Set a prescaler of 1:256
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupt
    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.TMR0IF = 0;          //Clear flag
    TMR0H=0xC2;                     //write 65536-15625
    TMR0L=0xF7;                     //total of exactly 1 second timer
    T0CONbits.TMR0ON=1;             //enable timer
}

void ReadTemp(){
    
}

void ChangePWM(){
    
}

//High priority interrupt
#pragma interrupt HiPriISR
void HiPriISR() {
    
}	// Supports retfie FAST automatically

//low priority interrupt
#pragma interruptlow LoPriISR nosave=TBLPTR, TBLPTRU, TABLAT, PCLATH, PCLATU, PROD, section("MATH_DATA")//, section(".tmpdata")
void LoPriISR() {
    while(1) {
        if(INTCONbits.TMR0IF) {
            TMR0Handler();
            continue;
        }
        break;
    }
}

void TMR0Handler(){
    if (mode == 1){
        
    }
    INTCONbits.TMR0IF = 0;  //Clear TMR0 interrupt flag
}