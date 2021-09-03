/*------------------------------------------------------------------------------
Archivo: mainsproject.s
Microcontrolador: PIC16F887
Autores: Pablo Herrarte y Andy Bonilla
Compilador: pic-as (v2.30), MPLABX v5.45
    
Programa: pic para sensores de proyecto 1 de Electronica Digital 2
Hardware: PIC16F887
    
Creado: 15 de agosto de 2021    
Descripcion: 
------------------------------------------------------------------------------*/

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT   //configuracion de oscilador interno
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

/*-----------------------------------------------------------------------------
 ----------------------------LIBRERIAS-----------------------------------------
 -----------------------------------------------------------------------------*/
#include <stdint.h>
#include <pic16f887.h>
#include <xc.h>
#include <proc/pic16f887.h>
#include "Osc_config.h"
#include "ADC_CONFIG.h"
#include "I2C.h"
#include "UART.h"
/*-----------------------------------------------------------------------------
 ----------------------- VARIABLES A IMPLEMTENTAR------------------------------
 -----------------------------------------------------------------------------*/

//-------DIRECTIVAS DEL COMPILADOR
#define _XTAL_FREQ 8000000

//-------VARIABLES DE PROGRAMA
unsigned char antirrebote, cuenta_uart;
unsigned char infrarrojo1, infrarrojo2, infrarrojo3, suma_ir;
float conversion1, conversion_total, temperatura_aprox;
uint8_t abierto_cerrado, BASURA, PARQUEO; 
/*-----------------------------------------------------------------------------
 ------------------------ PROTOTIPOS DE FUNCIONES ------------------------------
 -----------------------------------------------------------------------------*/
void setup(void);
void infrarrojos(void);
void mandar_datos(void);
/*-----------------------------------------------------------------------------
 --------------------------- INTERRUPCIONES -----------------------------------
 -----------------------------------------------------------------------------*/
void __interrupt() isr(void) //funcion de interrupciones
{
    //-------INTERRUPCION POR I2C
    if(PIR1bits.SSPIF == 1){    //Le va a mandar la cantidad de parqueos habilitados al master

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
            abierto_cerrado = SSPBUF;                 // Read the previous value to clear the buffer
            SSPCONbits.SSPOV = 0;       // Clear the overflow flag
            SSPCONbits.WCOL = 0;        // Clear the collision bit
            SSPCONbits.CKP = 1;         // Enables SCL (Clock)
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) 
        {
            abierto_cerrado = SSPBUF;                 // Lectura del SSBUF para limpiar el buffer y la bandera BF
            PIR1bits.SSPIF = 0;         // Limpia bandera de interrupci n recepci n/transmisi n SSP
            SSPCONbits.CKP = 1;         // Habilita entrada de pulsos de reloj SCL
            while(!SSPSTATbits.BF);     // Esperar a que la recepci n se complete
            BASURA = SSPBUF;             // Guardar en z el valor del buffer de recepci n
            __delay_us(200);
            
        }
        else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            abierto_cerrado = SSPBUF;
            BF = 0;
            SSPBUF = suma_ir;
            SSPCONbits.CKP = 1;
            __delay_us(200);
            while(SSPSTATbits.BF);
        }
       
        PIR1bits.SSPIF = 0;    
    }
    if (PIR1bits.TXIF)
    {
        cuenta_uart++;      //se suma variable guia
        mandar_datos();     //invoco funcion para mandar uart
        PIR1bits.TXIF=0;    //apago interrupcion
    }
   
}
/*-----------------------------------------------------------------------------
 ----------------------------- MAIN LOOP --------------------------------------
 -----------------------------------------------------------------------------*/
void main(void)
{
   
    setup();
    
    while(1)
    {
        
        //-------FUNCION PARA CONTROL DE SENSORES INFRARROJOS
        infrarrojos();
        
    }
   
}
/*-----------------------------------------------------------------------------
 ---------------------------------- SET UP -----------------------------------
 -----------------------------------------------------------------------------*/
void setup(void)
{
    //-------CONFIGURACION ENTRADAS ANALOGICAS
    ANSEL=0;
    ANSELH=0;
    //-------CONFIGURACION IN/OUT
    //SALIDAS PARA LEDS DE DISPONIBILIDAD
    TRISAbits.TRISA0=0;                 //salida para led verde1
    TRISAbits.TRISA1=0;                 //salida para led rojo1
    TRISAbits.TRISA2=0;                 //salida para led verde2
    TRISAbits.TRISA3=0;                 //salida para led rojo2
    TRISAbits.TRISA4=0;                 //salida para led verde3
    TRISAbits.TRISA5=0;                 //salida para led rojo3
    //ENTRADAS PARA SENSORES
    TRISBbits.TRISB1=1;                 //entrada de sensor IR1
    TRISBbits.TRISB2=1;                 //entrada de sensor IR2
    TRISBbits.TRISB3=1;                 //entrada de sensor IR3
    
    //-------LIMPIEZA DE PUERTOS
    PORTA=0;
    PORTB=0;
    PORTD=0;
    PORTE=0;
    //-------CONFIGURACION DE RELOJ A 4MHz
    osc_config(8);
    //-------CONFIGURACION DE ADC
    uart_config();
    //-------CONFIGURACION DE COMUNICACION I2C
    I2C_Slave_Init(0x50);   //se da direccion 0x50
    //-------CONFIGURACION DE INTERRUPCIONES
    //-------CONFIGURACION DE INTERRUPCIONES
    INTCONbits.GIE=1;           //se habilita interrupciones globales
    INTCONbits.PEIE=1;          //habilitan interrupciones por perifericos
    PIE1bits.TXIE=1;            //enable interrupcion de tx uart    
}
/*-----------------------------------------------------------------------------
 --------------------------------- FUNCIONES ----------------------------------
 -----------------------------------------------------------------------------*/
//-------FUNCION PARA CONTROL DE SENSORES INFRARROJOS
void infrarrojos(void)
{
    //-------PARQUEO 1
    if (PORTBbits.RB1==1)
    {
        PORTAbits.RA0=1;
        PORTAbits.RA1=0;
        infrarrojo1=1;
    }
    if (PORTBbits.RB1==0)
    {
        PORTAbits.RA0=0;
        PORTAbits.RA1=1;
        infrarrojo1=0;
    }
    //-------PARQUEO2
    if (PORTBbits.RB2==1)
    {
        PORTAbits.RA2=1;
        PORTAbits.RA3=0;
        infrarrojo2=1;
    }
    if (PORTBbits.RB2==0)
    {
        PORTAbits.RA2=0;
        PORTAbits.RA3=1;
        infrarrojo2=0;
    }
    //-------PARQUEO3
    if (PORTBbits.RB3==1)
    {
        PORTAbits.RA4=1;
        PORTAbits.RA5=0;
        infrarrojo3=1;
    }
    if (PORTBbits.RB3==0)
    {
        PORTAbits.RA4=0;
        PORTAbits.RA5=1;
        infrarrojo3=0;
    }
    suma_ir=infrarrojo1+infrarrojo2+infrarrojo3; //suma de los parqueos disponibles
   
}
//-------FUNCION PARA CONVERSION DEL ADC

void mandar_datos(void)
{
    switch(cuenta_uart)
    {
        case(1):
            TXREG=(suma_ir+0x30);
            break;
        case(2):
            TXREG=44;
            break;
        case(3):
            TXREG=infrarrojo1+0x30;
            break;
        case(4):
            TXREG=44;
            break;
        case(5):
            TXREG=infrarrojo2+0x30;
            break;
        case(6):
            TXREG=44;
            break;
        case(7):
            TXREG=infrarrojo3+0x30;
            break;
        case(8):
            TXREG=10;               //separador de coma
            cuenta_uart=0;            
            break;
    }
    
}