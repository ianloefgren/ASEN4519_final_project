/* 
 * File:   ControllerMain.c
 * Author: nikolas
 *
 * Created on December 6, 2017, 2:34 PM
 */

//Description: This is the main code for the Home Hub project. This part of code
//would be the code for the Home hub controller. The code would do the following
//  -Specify 2 modes of control: In first mode, LED hue dependent on temperature
//   light bulb on of is controlled by user specified timer
//   in second mode, led hue is controlled using potentiometer, and lightbulb is
//   controlled manually using a button
//  -Read Potentiometer, RPG, change mode button, select button, and lightbulb 
//   light bulb on off button


//Include libraries
#include <p18cxxx.h>
#include <stdio.h>
#include <stdlib.h>
#include "LCDroutinesEasyPic.h"
#include <adc.h>
#include <math.h>
#include <string.h>
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

//Defining functions
void HiPriISR(void);
void LoPriISR(void);
void Initial(void);
void InitialADC(void);
void InitialSPI(void);
void InitialRPG(void);
void ReadButtons(void);
void RPGHandler(void);


//Defining ports and Global Variables
#define buttonSelect PORTHbits.RH0  //button for enter/select
#define buttonMode PORTHbits.RH1    //button to change mode
#define buttonLight PORTHbits.RH2   //button to turn light bulb on off on mode 2
#define RPGdata PORTEbits.RE4       //RPG data line
char LCDstart1[] = {0x80,'M','O','D','E','=','1',' ',' ','L','E','D','=','A','U','T','O',0x00};
char LCDstart2[] = {0xC0,'T','I','M','E','R','=','0','0',':','0','0',0x00};
char LCDbuff[10]={0,0,0,0,0,0,0,0,0,0};
char mode =1;       //mode flag, 1 for mode 1 and 2 for mode 2
char butflag = 0;   //flag to check if a button has been clicked
char butbuf = 0;    //flag for storing button values
int value = 0;      //Variable to store ADC conversion
int LED = 0;        //variable to store led brightness percentage



#pragma code highVector=0x08
void atHighVector(void)
{
 _asm GOTO HiPriISR _endasm
}
#pragma code

#pragma code lowVector=0x18
void atLowVector(void)
{
 _asm GOTO LoPriISR _endasm
}
#pragma code

/*
 * 
 */
void main(void) {
    Initial();          //Initializing function and registers
    while(1){
        ConvertADC();       //Make ADC conversion
        
        ReadButtons();
        
        while (BusyADC()){ //wait until conversion is over
        }
        value = ReadADC(); //read the value converted
        LED = value/4096*100;//convert into percent
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
    
    //Initialize all  stuff
    InitialADC();
    
    //Initialize all SPI stuff
    InitialSPI();
    
    //Initialize all RPG stuff
    InitialRPG();    
    
    TRISH=0B11111111; //Set all H as input (only going to use 0 1 and 2) for buttons
    TRISD=0B00000000;
    
    DisplayV(LCDstart1);    //Display LCD initial interface
    DisplayV(LCDstart2);
}

//Using ECCP module CCP1 as hardware interrupt because cannnot use regullar 
//hardware interrupt since pin RB0 - RB3 are used for LCD
void InitialRPG(){
    PIE3bits.CCP1IE=1;  //Enable ccp 1 interrupt
    IPR3bits.CCP1IP=0;  //Set to low priority interrupt
    CCP1CON=0b00000101; //Set as capture mode, capture at every rising edge
    CCPTMRS0=0b00000000;//Set timer, not really used
    TRISEbits.TRISE5=1; //Pin RE5 is used as the input
    TRISEbits.TRISE4=1;  //RPGdata as input
    PIR3bits.CCP1IF=0;  //Clear CCP1 interupt flag
}

//Initializing ADC for the use of potentiometer. This potentiometer would read
//its value and control an LED through PWM on the other PIC microcontroller
void InitialADC(){
    ADCON0=00011000;      //Select AN6 (RF1) to use P3 potentiometer
    ADCON1=0B00110000;    //ECCP2,Internal Vref+,AVSS, AVss
    ADCON2=0B10101101;    //Right Justified, 12 TAD, Fosc/16
    
}

void InitialSPI(){
    
}

//Read all the buttons and do commands, button will be triggered on release
void ReadButtons(){
    butbuf = PORTH;     //store porth input in a variable
    if (butbuf){         //Check if Porth not 0, a button has been pressed
        butflag=butbuf;  //Set flag on the responding pressed button
    }
    if(!butbuf && butflag){  //check if any flag is on while button not pressed
        if(butflag & 1){        //check if buttonselect that triggers
            
        }
        else if(butflag & 2){   //check if buttonMode that triggers
            if(mode == 1){      //if mode 1 change to mode 2,
                mode++;
                LATDbits.LATD1=1;
                sprintf(LCDbuff,"%c%c%c",0x85,2,0x00);
                DisplayV(LCDbuff);
                sprintf(LCDbuff,"%c%i%%c",0x8C,LED,0x00);
                
            }
            else{
                mode--;         //and if mode 2 change to mode 1
            }
        }
        else if(butflag & 4){   //check if buttonLight that triggers
            
        }
        else if(butflag & 8){   //check if
            
        }
        butflag=0;
    }  
}

//High priority interrupt
#pragma interrupt HiPriISR
void HiPriISR() {
    
}	// Supports retfie FAST automatically

//low priority interrupt
#pragma interruptlow LoPriISR nosave=TBLPTR, TBLPTRU, TABLAT, PCLATH, PCLATU, PROD, section("MATH_DATA")//, section(".tmpdata")
void LoPriISR() {
    while(1) {
        if( PIR3bits.CCP1IF ) {
            RPGHandler();
            continue;
        }
        break;
    }
}

void RPGHandler() {
    if (mode == 1){      //check if on mode 1 (light bulb timer mode)
        if (RPGdata){   //
            
        }
        else{
            
        }
    }
    PIR3bits.CCP1IF=0;  //clear ccp1 interrupt flag
}