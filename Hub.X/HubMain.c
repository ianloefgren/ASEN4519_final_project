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
#include <string.h>
#include <p18f87k22.h>
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

//Declare variables
#define Bulb PORTDbits.RD0 = 0; //relay that turn bulb on/off
char LCDstart1[] = {0x80,'M','O','D','E','=','1',' ',' ','T','E','M','P','=','0','0','C',0x00};
char LCDstart2[] = {0xC0,'T','I','M','E','R','=','0','0',':','0','0',0x00};
char mode = 1; //mode at which the hub is working on 
char LCDbuff[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};
float value = 0; 
float tempbuf = 0;
int tempbuf2 = 0;
int temp = 0;
int lasttemp = 0;
int timers = 0; //valuie to store timer 
char LCDtime[5]={0x30,0x30,':',0x30,0x30};
float LCDtimebuf1 = 0;  //Buffers for calculation
int LCDtimebuf2 = 0;
char timeflag = 0;

//Declare Functions
void HiPriISR(void);
void LoPriISR(void);
void Initial(void);
void InitialADC(void);
void ReadTemp(void);
void InitialTimer(void);
void TMR0Handler(void);
void InitialPWM(void);
void ChangePWM(void);
void DisplayLCD(void);


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
    Initial();
    while(1){
        ConvertADC();       //Make ADC conversion
        
        DisplayLCD();
        
        while (BusyADC()){ //wait until conversion is over
        }
        value = ReadADC();
        tempbuf = (float)value*3.3*100/4096; //convert to degree C
        temp=(int) tempbuf;
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
    
    TRISDbits.TRISD0 = 0;   //set bulb as output
    
    DisplayV(LCDstart1);    //Display LCD initial interface
    DisplayV(LCDstart2);
}

void configureUART(){
    //The UART Config are set manually to help understand what each register do
    
    TRISCbits.TRISC7=1; //set c7 as input (RX1)
    TRISCbits.TRISC6=0; //set c6 as output(TX1)
    TXSTA1bits.TX9=0;   //set as 8 bit transimission
    TXSTA1bits.TXEN=1;  //transmit enable bit
    TXSTA1bits.SYNC=0;  //Asynchronous mode
    TXSTA1bits.SENDB=0; //
    TXSTA1bits.TX9D=0;  //Parity none
    RCSTA1bits.CREN=1;  //Continuous receive enables
    RCSTA1bits.RX9=0;
    RCSTA1bits.SPEN=1;  //Serial port enable bit for EUSART1
    //Configure the baud rate generator to use a baud rate of 19200 bps
    //decided to go with low speed baudrate with SPBRG = 12 to get 19230 bps(0.15% error)
    BAUDCONbits.BRG16=0;//8 bit mode, SPBRGH ignored
    TXSTAbits.BRGH=0;   //Low speed mode
    SPBRG1=12;          //set serial port baudrate generator value   
    //seting up timer timer 1 to output
    PIR1bits.TMR1IF=0;  //clear timer 1 interrupt flag  
    TMR1H=0x3C;         //count to 50000
    TMR1L=0xB0;         
    T1CON=0B00110001;   //set prescaler of 1:8, enable timer
    //setup USART 1 receive interrupt
    PIE1bits.RC1IE=1;   //enable receive 1 interrupt
    IPR1bits.RC1IP=0;   //set low priority interrupt
    PIR1bits.RC1IF=0;   //clear receive 1 interrupt flag
    TRISF=0;            //set F as all output
}

//Using CCP 6, 7, and 8 for generating PWM signal that would control the i
//Intensity of R, G and B LED on the RGB LED. This module are chosen because the
//pins has outputs that's near to each other
void InitialPWM(){
    CCPTMRS1 = 0B00000000;  //CCP 6 based on TMR 2
    CCPTMRS2 = 0B00000000;
    TRISE = 0;  //PORT E are all outputs
    T2CON = 0b00000111;   //1:1 Post and Pre Scale, turn on Timer 2
    PR2 = 0B11111111;       //2^8 = 256, for the period register
    //CCP6 FOR Red LED pin on RE6
    CCP6CON = 0B00111100;   //Set to PWM mode
    CCPR6L = 255;    //duty cycle of 50 %
    //CCP7 for Green LED pin on RE5
    CCP7CON = 0B00001100;   //
    CCPR7L = 0;
    //CCP8 for Blue LED pin on RE4
    CCP8CON = 0B00111100;   //
    CCPR8L = 255;
}

void InitialADC(){
    TRISAbits.TRISA3=1;     //Set A1 as input
    ADCON0=0B00001100;      //Select AN3 (RA1) to use LM35 Sensor
    ADCON1=0B00110000;    //ECCP2,Internal Vref+,AVSS, AVss
    ADCON2=0B10101101;    //Right Justified, 12 TAD, Fosc/16
    ADCON0bits.ADON=1;    //Turn on ADC
}

void InitialTimer(){
    T0CON = 0b00000111;             //Set a prescaler of 1:256
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupt
    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.TMR0IF = 0;          //Clear flag
    TMR0H=0xC2;                     //write 65536-15625
    TMR0L=0xF7;                     //total of exactly 1 second timer
    T0CONbits.TMR0ON=1;             //enable timer
    
    timers = 20;
}

void DisplayLCD(){
    sprintf(LCDbuff,"%c%i%c",0x8D,temp,0x00);
    DisplayV(LCDbuff);
    if (timeflag == 1){
        LCDtimebuf1 = (float)timers/60;
        LCDtimebuf2 = (int)LCDtimebuf1;
        if (LCDtimebuf2 >= 10){
            sprintf(LCDbuff,"%c%i%c",0XC6,LCDtimebuf2,0x00);
        }                
        else{
            sprintf(LCDbuff,"%c0%i%c",0XC6,LCDtimebuf2,0x00);
        }        
        DisplayV(LCDbuff);
        LCDtimebuf2 = (int) timers % 60;
        if (LCDtimebuf2 >= 10){
            sprintf(LCDbuff,"%c%i%c",0XC9,LCDtimebuf2,0x00);
        }                
        else{
            sprintf(LCDbuff,"%c0%i%c",0XC9,LCDtimebuf2,0x00);
        }
        DisplayV(LCDbuff);
        if(!timers){
            LATDbits.LATD0=0;
        }
        timeflag = 0;
    }
}

void ChangePWM(){
    if (lasttemp != temp && mode == 1){
        tempbuf2 = (int)(temp - 22)/8*255;
        CCPR6L = (char)tempbuf2;
        tempbuf2 = 255-tempbuf2;
        CCPR8 = (char)tempbuf2;
        lasttemp = temp;
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
        if(INTCONbits.TMR0IF) {
            TMR0Handler();
            continue;
        }
        
        break;
    }
}

void TMR0Handler(){
    if (mode == 1 && timers){
        timers--;
        timeflag = 1;
    }
    TMR0H=0xC2;                     //write 65536-15625
    TMR0L=0xF7;                     //total of exactly 1 second timer
    INTCONbits.TMR0IF = 0;  //Clear TMR0 interrupt flag
}