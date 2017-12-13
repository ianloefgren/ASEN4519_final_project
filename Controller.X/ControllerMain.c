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
void DisplayLCD(void);


//Defining ports and Global Variables
#define buttonSelect PORTJbits.RJ0  //button for enter/select
#define buttonMode PORTJbits.RJ1    //button to change mode
#define buttonLight PORTJbits.RJ2   //button to turn light bulb on off on mode 2
#define RPGdata PORTEbits.RE1       //RPG data line
char LCDstart1[] = {0x80,'M','O','D','E','=','1',' ',' ','L','E','D','=','A','U','T','O',0x00};
char LCDstart2[] = {0xC0,'T','I','M','E','R','=','0','0',':','0','0',0x00};
char LCDbuff[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};
char mode =1;       //mode flag, 1 for mode 1 and 2 for mode 2
char butflag = 0;   //flag to check if a button has been clicked
char butbuf = 0;    //flag for storing button values
float value = 0;      //Variable to store ADC conversion
float LEDtemp = 0;  //temporary LED calculation
int LED = 0;        //variable to store led brightness percentage
char bulb = 0;      //v=bulb on off flag
unsigned long bounce = 0;    //bounce to store timer value
unsigned long millis = 0;
int time = 0;       //variable to store the timer value
char timepos = 0xC6;//position of the timer
unsigned long blink = 0;//variable to store blinking effect timer
char blinkf = 0;        //flag that tells that timer position needs to blink
char LCDtime[5]={0x30,0x30,':',0x30,0x30};  


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
        
        DisplayLCD();
        
        while (BusyADC()){ //wait until conversion is over
        }
        value = ReadADC(); //read the value converted
        LEDtemp= (float)value*100/4096;
        LED = (int)LEDtemp;//convert into int
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
    
    TRISJ=0B11111111; //Set all J as input (only going to use 0 1 and 2) for buttons
    TRISD=0B00000000;
    
    DisplayV(LCDstart1);    //Display LCD initial interface
    DisplayV(LCDstart2);
}

//Using ECCP module CCP1 as hardware interrupt because cannnot use regullar 
//hardware interrupt since pin RB0 - RB3 are used for LCD
void InitialRPG(){
    /*      Older solution, using ECCP1
    PIE3bits.CCP1IE=1;  //Enable ccp 1 interrupt
    IPR3bits.CCP1IP=0;  //Set to low priority interrupt
    CCP1CON=0b00000101; //Set as capture mode, capture at every rising edge
    CCPTMRS0=0b00000000;//Set timer, not really used
    TRISEbits.TRISE5=1; //Pin RE5 is used as the input
    TRISEbits.TRISE4=1;  //RPGdata as input
    PIR3bits.CCP1IF=0;  //Clear CCP1 interupt flag
    TRISE=0xFF; //all input for now
     */
    //Using CCP6 instead because the pin is more well defined at RE6
    CCP6CON=0B00000101; //Capture mode every rising edge
    CCPTMRS1=0b00000000;//Select timer, not really used
    TRISE=0xFF;         //All input on E
    PIR4bits.CCP6IF=0;  //Clear Interupt flag
    IPR4bits.CCP6IP=0;  //Set to low priority
    PIE4bits.CCP6IE=1;  //Enable Interrupt
    
    //Using Timer0 as a RPG debouncer since the RPG has shown to be noisy
    T0CON = 0b00000000; //Set a prescaler of 1:2
    INTCON2bits.TMR0IP = 1;         // Assign high priority to TMR0 interrupt
    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.TMR0IF = 0;          //Clear flag
    TMR0H=0xB1;                     //write 65536-20000
    TMR0L=0xE0;
    T0CONbits.TMR0ON = 1;           // Turning on TMR0
}

//Initializing ADC for the use of potentiometer. This potentiometer would read
//its value and control an LED through PWM on the other PIC microcontroller
void InitialADC(){
    TRISFbits.TRISF1=1;   //set RF1 as input
    ADCON0=0B00011000;      //Select AN6 (RF1) to use P3 potentiometer
    ADCON1=0B00110000;    //ECCP2,Internal Vref+,AVSS, AVss
    ADCON2=0B10101101;    //Right Justified, 12 TAD, Fosc/16
    ADCON0bits.ADON=1;    //Turn on ADC
}

void InitialSPI(){
    
}

//Read all the buttons and do commands, button will be triggered on release
void ReadButtons(){
    butbuf = PORTJ;     //store portJ input in a variable
    if (butbuf){         //Check if PortJ not 0, a button has been pressed
        butflag=butbuf;  //Set flag on the responding pressed button
    }
    if(!butbuf && butflag){  //check if any flag is on while button not pressed
        if(butflag & 2){   //check if buttonMode that triggers
            if(mode == 1){      //if mode 1 change to mode 2,
                mode++;
                sprintf(LCDbuff,"%c%c%c",0x85,0x32,0x00);
                DisplayV(LCDbuff);
                if ( bulb == 1){
                    sprintf(LCDbuff,"%cBulb ON   %c",0xC6,0x00);
                }
                else{
                    sprintf(LCDbuff,"%cBulb Off  %c",0xC6,0x00);
                }
                DisplayV(LCDbuff);
            }
            else{
                mode--;         //and if mode 2 change to mode 1                
                DisplayV(LCDstart1);
                sprintf(LCDbuff,"%c%c%c:%c%c   %c",0XC6,LCDtime[0],LCDtime[1],LCDtime[3],LCDtime[4],0x00);
                DisplayV(LCDbuff);
            }
        }
        else if(butflag & 4){   //check if buttonLight that triggers
            if(mode == 2){
                if (bulb){
                    bulb--;
                    sprintf(LCDbuff,"%cBulb Off  %c",0xc6,0x00);                    
                }
                else{
                    bulb++;
                    sprintf(LCDbuff,"%cBulb On   %c",0xc6,0x00);
                }
                DisplayV(LCDbuff);
            }
        }
        else if(butflag & 8){   //check if buttonsync triggered 
            
        }
        else if((butflag & 128) && mode == 1 && timepos < 0xCA ){ // Check if button right <7> has been triggered
            timepos++;
            if (timepos == 0xC8){
                timepos++;          //increase again if in ':' position
            }
            sprintf(LCDbuff,"%c %c",timepos,0x00);
            DisplayV(LCDbuff);
            blink = millis;                   
            blinkf=1;            
        }
        else if((butflag & 64) && mode == 1 && timepos > 0xC6){  // Check if button left <6> has been triggered
            timepos--;
            if (timepos == 0xC8){
                timepos--;          //decrease again if in ':' position
            }
            sprintf(LCDbuff,"%c %c",timepos,0x00);
            DisplayV(LCDbuff);
            blink = millis;                   
            blinkf=1;
        }
        butflag=0;
    }  
}

// This subroutine is for LCD display that needs to be constantly updated
void DisplayLCD(){
    if (mode == 2){         //update LED percentage on the LCD
        if(LED >=100){
            sprintf(LCDbuff,"%c%i%%c",0x8C,LED,0x00);
        }
        else if(LED>=10){
            sprintf(LCDbuff,"%c %i%%c",0x8C,LED,0x00);
        }
        else{
            sprintf(LCDbuff,"%c  %i%%c",0x8C,LED,0x00);
        }
        DisplayV(LCDbuff);
    }
    if (blinkf == 1 && millis-blink >= 40){ // Blink back the value after some timer
        blinkf = 0;
        sprintf(LCDbuff,"%c%c%c",timepos,LCDtime[timepos-0xC6],0x00);
        DisplayV(LCDbuff);
    }
}

//High priority interrupt
#pragma interrupt HiPriISR
void HiPriISR() {
    millis++;               //increase timer
    TMR0H=0xB1;             //write 65536-20000
    TMR0L=0xE0;                        
    INTCONbits.TMR0IF=0;    //clear interrupt flag
}	// Supports retfie FAST automatically

//low priority interrupt
#pragma interruptlow LoPriISR nosave=TBLPTR, TBLPTRU, TABLAT, PCLATH, PCLATU, PROD, section("MATH_DATA")//, section(".tmpdata")
void LoPriISR() {
    while(1) {
        if( PIR4bits.CCP6IF && millis - bounce >= 20 ) {
            RPGHandler();
            bounce = millis;
            continue;
        }
        PIR4bits.CCP6IF=0;  //clear ccp6 interrupt flag
        break;
    }
}

void RPGHandler() {
    if (mode == 1){      //check if on mode 1 (light bulb timer mode)
        if (RPGdata){   //
            if ((timepos == 0xC6 || timepos == 0xC7 || timepos == 0xCA) && LCDtime[timepos - 0xC6] < 0x39){
                LCDtime[timepos - 0xC6]++;
                sprintf(LCDbuff,"%c%c%c",timepos,LCDtime[timepos-0xC6],0x00);
                DisplayV(LCDbuff);                        
            }
            else if (LCDtime[timepos-0xC6] < 0x35){
                LCDtime[timepos - 0xC6]++;
                sprintf(LCDbuff,"%c%c%c",timepos,LCDtime[timepos-0xC6],0x00);
                DisplayV(LCDbuff);
            }
        }
        else{
            if (LCDtime[timepos-0xC6] > 0x30){
                LCDtime[timepos - 0xC6]--;
                sprintf(LCDbuff,"%c%c%c",timepos,LCDtime[timepos-0xC6],0x00);
                DisplayV(LCDbuff);
            }
        }
    }
}