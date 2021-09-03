/*
 * File:   main.c
 * Author: Pablo Herrarte y Andy Bonilla
 * Ejemplo de uso de I2C Master
 * Created on 19 de agosto de 2021
 */
//*****************************************************************************
// Palabra de configuraci n
//*****************************************************************************
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//*****************************************************************************
// Definici n e importaci n de librer as
//*****************************************************************************
#include <stdint.h>
#include <pic16f887.h>
#include "I2C.h"
#include "Osc_config.h"
#include "ASCII.h"
#include "LCD.h"
#include <xc.h>
//*****************************************************************************
// Definici n de variables
//*****************************************************************************
#define _XTAL_FREQ 8000000
//-------DIRECTIVAS DEL COMPILADOR
#define _XTAL_FREQ 8000000
#define RS RD2
#define EN RD3
#define D4 RD4
#define D5 RB5
#define D6 RD6
#define D7 RD7
//-------VARIABLES DE PROGRAMA
unsigned int a;
uint8_t PARKH;  //Parqueos habilitados
uint8_t NUM;
uint8_t BASURA;
uint8_t DIA;    //Dia de la semana para habilitar el motor servo
uint8_t HORA;   //Hora para las luces del parqueo
uint8_t MIN;    //Minutos 
uint8_t TEMP;    //Temperatura 
uint8_t C1;     //
uint8_t C2;
uint8_t C3;
uint8_t UH;
uint8_t DH;
uint8_t UM;
uint8_t DM;
uint8_t CERRADO;
uint8_t con;
uint8_t temperatura;
//*****************************************************************************
// Definici n de funciones para que se puedan colocar despu s del main de lo 
// contrario hay que colocarlos todas las funciones antes del main
//*****************************************************************************
void setup(void);
void LECT1(void);
const char* conver(void); //Datos que recibirá la LCD
const char* conver1(void);
//*****************************************************************************
// Main
//*****************************************************************************
void main(void) {
    setup();
    Lcd_Init();
    Lcd_Clear();
    while(1){
        Lcd_Clear();
        Lcd_Set_Cursor(1,1);    //Cursor en primera línea
        Lcd_Write_String(conver1()); //Escribir Día hora y minutos
        Lcd_Set_Cursor(2,1);    //Cursor en segunda línea
        Lcd_Write_String(conver());    //Escribir los datos de parqueos hábiles
        LECT1();        //Conversión de datos
        
        I2C_Master_Start();     //escribe el dato de parqueos hábiles del pic que controla los parqueos
        I2C_Master_Write(0x50);
        I2C_Master_Write(0);
        I2C_Master_Stop();
        __delay_ms(200);
       
        I2C_Master_Start();
        I2C_Master_Write(0x51);
        PARKH = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(200);
        
        I2C_Master_Start();     //escribe al pic de los motores que está cerrado
        I2C_Master_Write(0x60);
        I2C_Master_Write(CERRADO);
        I2C_Master_Stop();
        __delay_ms(10);
       
        I2C_Master_Start();
        I2C_Master_Write(0x61);
        temperatura = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(10);
        
        I2C_Master_Start();     //Escribe las horas del sensor DS3231
        I2C_Master_Write(0xD0);
        I2C_Master_Write(0x02);
        I2C_Master_Stop();
        __delay_ms(10);
       
        I2C_Master_Start();
        I2C_Master_Write(0xD1);
        HORA = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(10);
        
        I2C_Master_Start();     //Escribe el día del sensor DS3231
        I2C_Master_Write(0xD0);
        I2C_Master_Write(0x03);
        I2C_Master_Stop();
        __delay_ms(10);
       
        I2C_Master_Start();
        I2C_Master_Write(0xD1);
        DIA = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(10);
        
        I2C_Master_Start();     //Escribe los minutos
        I2C_Master_Write(0xD0);
        I2C_Master_Write(0x01);
        I2C_Master_Stop();
        __delay_ms(10);
       
        I2C_Master_Start();
        I2C_Master_Write(0xD1);
        MIN = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(10);
       
    }
    return;
}
//*****************************************************************************
// Funci n de Inicializaci n
//*****************************************************************************
void setup(void){
    ANSEL = 0;
    ANSELH = 0;
    TRISA = 0;
    TRISB = 0;
    TRISD = 0;
    TRISE = 0;
    PORTA = 0;
    PORTB = 0;
    PORTD = 0;
    osc_config(8);
    I2C_Master_Init(100000);        // Inicializar Comuncaci n I2C
    
    I2C_Master_Start();     //Escritura de datos iniciales
    I2C_Master_Write(0xD0);
    I2C_Master_Write(0x02);
    I2C_Master_Write(0x06);
    I2C_Master_Stop();
    __delay_ms(10);
    
    I2C_Master_Start();
    I2C_Master_Write(0xD1);
    BASURA = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(10);
        
    I2C_Master_Start();
    I2C_Master_Write(0xD0);
    I2C_Master_Write(0x01);
    I2C_Master_Write(0x59);
    I2C_Master_Stop();
    __delay_ms(10);
       
    I2C_Master_Start();
    I2C_Master_Write(0xD1);
    BASURA = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(10);
        
    I2C_Master_Start();
    I2C_Master_Write(0xD0);
    I2C_Master_Write(0x00);
    I2C_Master_Write(0x00);
    I2C_Master_Stop();
    __delay_ms(10);
       
    I2C_Master_Start();
    I2C_Master_Write(0xD1);
    BASURA = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(10);
        
    I2C_Master_Start();
    I2C_Master_Write(0xD0);
    I2C_Master_Write(0x03);
    I2C_Master_Write(0x01);
    I2C_Master_Stop();
    __delay_ms(10);
       
    I2C_Master_Start();
    I2C_Master_Write(0xD1);
    BASURA = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(10);
}
//cadena de caracteres para linea 2 de lcd
const char* conver(void){   //Datos que recivirá la LCD
    if (CERRADO==0){
        char temporal[16];
        temporal[0] = 0x50; //P
        temporal[1] = 0x41; //A
        temporal[2] = 0x52; //R
        temporal[3] = 0x51; //Q
        temporal[4] = 0x55; //U
        temporal[5] = 0x45; //E
        temporal[6] = 0x4F; //O
        temporal[7] = 0x53; //S
        temporal[8] = 0x3A; //:
        temporal[9] = 0X20; //SPC
        temporal[10] = 0x30;
        temporal[11] = NUM; //Parqueos hábiles que recibe del PIC esclavo de parqueos
        temporal[12] = 0x20;
        temporal[13] = 0x20;
        temporal[14] = 0x20;
        temporal[15] = 0x20;
        return temporal;
    }
    else{
        char temporal[16];
        temporal[0] = 0x43; //C
        temporal[1] = 0x45; //E
        temporal[2] = 0x52; //R
        temporal[3] = 0x52; //R
        temporal[4] = 0x41; //A
        temporal[5] = 0x44; //D
        temporal[6] = 0x4F; //O
        temporal[7] = 0x20; //
        temporal[8] = 0x20; //
        temporal[9] = 0X20; //
        temporal[10] = 0x20;
        temporal[11] = 0x20; //
        temporal[12] = 0x20;
        temporal[13] = 0x20;
        temporal[14] = 0x20;
        temporal[15] = 0x20;
        return temporal;
    }
}
//cadena de caracteres para linea 1 de lcd
const char* conver1(void)
{
    char temporal[16];
    temporal[0] = C1; //Caractéres del día
    temporal[1] = C2; 
    temporal[2] = C3; 
    temporal[3] = 0x2E; //.
    temporal[4] = 0x20; //SPC
    temporal[5] = DH; //Deena de hora
    temporal[6] = UH; //Unidad de hora
    temporal[7] = 0x3A; //:
    temporal[8] = DM; //Decena de minutos
    temporal[9] = UM; //Unidad de minutos
    temporal[10] = 0x20;    
    temporal[11] = ((temperatura/10)%10+0x30);    //temperatura decenas
    temporal[12] = ((temperatura%10)+0x30);    //temperatura unidades
    temporal[13] = 0xDF;    //grados
    temporal[14] = 0x43;    //celcius
    temporal[15] = 0x20;
    return temporal;
}
//Conversión de datos para mostrar en pantalla LCD
void LECT1(void){           
    NUM = num_ascii(PARKH); 
    if (DIA == 1){          //Escritura del día
        C1 = 0x4C;
        C2 = 0x75;
        C3 = 0x6E;
    }
    else if(DIA == 2){
        C1 = 0x4D;
        C2 = 0x61;
        C3 = 0x72;
    }
    else if(DIA == 3){
        C1 = 0x4D;
        C2 = 0x69;
        C3 = 0x65;
    }
    else if (DIA == 4){
        C1 = 0x4A;
        C2 = 0x75;
        C3 = 0x65;
    }
    else if (DIA == 5){
        C1 = 0x56;
        C2 = 0x69;
        C3 = 0x65;
    }
    else if (DIA == 6){
        C1 = 0x53;
        C2 = 0x61;
        C3 = 0x62;
    }
    else if (DIA == 7){
        C1 = 0x44;
        C2 = 0x6F;
        C3 = 0x6D;
        CERRADO = 1;
    }
    if (HORA<10){       //Escritura de la hora
        DH = 0x30;
        UH = num_ascii(HORA);
        if (HORA<7){
            PORTB = 0b11000000;
            CERRADO = 1;
        }
        else{
            PORTB = 0;
            if (DIA!=7){
                CERRADO = 0;
            }
        }
    }
    else if (HORA<26){
        DH = 0x31;
        con = HORA-16;
        UH = num_ascii(con);
        if (DIA!=7){
            CERRADO = 0;
        }
        if (con>7){
            PORTB = 0b11000000;
        }
        else{
            PORTB = 0;
        }
    }
    else{
        DH = 0x32;
        con = HORA-32;
        UH = num_ascii(con);
        PORTB = 0b11000000;
        if (con>1){
            CERRADO = 1;
        }
        else{
            if (DIA!=7){
                CERRADO = 0;
            }
        }
    }
    if (MIN<10){            //Escritura de los minutos
        DM = 0x30;
        UM = num_ascii(MIN);
    }
    else if (MIN<26){
        DM = 0x31;
        con = MIN-16;
        UM = num_ascii(con);                
    }
    else if (MIN<42){
        DM = 0x32;
        con = MIN-32;
        UM = num_ascii(con);
    }
    else if (MIN<58){
        DM = 0x33;
        con = MIN-32-16;
        UM = num_ascii(con);
    }
    else if (MIN<74){
        DM = 0x34;
        con = MIN-64;
        UM = num_ascii(con);
    }
    else if (MIN<90){
        DM = 0x35;
        con = MIN-64-16;
        UM = num_ascii(con);
    }
}